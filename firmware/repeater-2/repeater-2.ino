#include <Arduino.h>
#include <WiFi.h>
#include <esp_idf_version.h>
#include <esp_now.h>
#include <esp_wifi.h>

// Repeater 2 validates each record, drops duplicates/old records, and forwards
// the newest valid record immediately. It stores no historical data.

namespace Config {
constexpr uint32_t USB_BAUD = 115200;
constexpr uint8_t RADIO_CHANNEL = 1;
constexpr uint8_t RADIO_COPIES = 3;
constexpr uint32_t COPY_GAP_MS = 3;
constexpr size_t MODBUS_SIZE = 7;
constexpr size_t PAYLOAD_SIZE = 14;

const uint8_t PREVIOUS_MAC[6] = {0x20, 0x9B, 0xA9, 0x68, 0xAB, 0x9C};
const uint8_t OWN_MAC[6] = {0x68, 0x09, 0x47, 0x9D, 0x32, 0xE4};
const uint8_t NEXT_MAC[6] = {0x68, 0x09, 0x47, 0x5C, 0x02, 0x9C};
}  // namespace Config

namespace Protocol {
constexpr uint16_t MAGIC = 0x574E;
constexpr uint8_t VERSION = 3;
constexpr uint8_t DATA = 1;

#pragma pack(push, 1)
struct DataFrame {
  uint16_t magic;
  uint8_t version;
  uint8_t type;
  uint64_t recordId;
  uint64_t sampleTimeMs;
  uint8_t payload[Config::PAYLOAD_SIZE];
  uint32_t crc32;
};
#pragma pack(pop)

static_assert(sizeof(DataFrame) == 38, "Unexpected DataFrame size");
}  // namespace Protocol

struct ReceivedFrame {
  uint8_t sender[6];
  int16_t length;
  uint8_t bytes[sizeof(Protocol::DataFrame)];
};

portMUX_TYPE mailboxLock = portMUX_INITIALIZER_UNLOCKED;
ReceivedFrame latestFrame{};
volatile bool latestFrameReady = false;
volatile uint32_t mailboxOverwrites = 0;
uint64_t lastForwardedId = 0;
uint32_t forwarded = 0;
uint32_t duplicates = 0;
uint32_t rejected = 0;
uint32_t radioSendFailures = 0;
uint32_t nextStatsAt = 0;

// Calculates CRC-32/ISO-HDLC for the ESP-NOW application frame.
uint32_t crc32Ieee(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFFUL;
  while (length-- > 0) {
    crc ^= *data++;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      const uint32_t mask = 0U - (crc & 1U);
      crc = (crc >> 1) ^ (0xEDB88320UL & mask);
    }
  }
  return ~crc;
}

// Calculates the standard Modbus RTU CRC-16.
uint16_t modbusCrc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  while (length-- > 0) {
    crc ^= *data++;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001U)
                       : static_cast<uint16_t>(crc >> 1);
    }
  }
  return crc;
}

// Verifies one embedded seven-byte Modbus response.
bool validModbus(const uint8_t frame[Config::MODBUS_SIZE]) {
  if (frame[0] != 0x01 || frame[1] != 0x03 || frame[2] != 0x02) return false;
  const uint16_t crc = modbusCrc16(frame, 5);
  return frame[5] == static_cast<uint8_t>(crc) &&
         frame[6] == static_cast<uint8_t>(crc >> 8);
}

// Copies and validates the outer frame plus both inner Modbus frames.
bool decodeData(const uint8_t *bytes, int length, Protocol::DataFrame &frame) {
  if (!bytes || length != static_cast<int>(sizeof(frame))) return false;
  memcpy(&frame, bytes, sizeof(frame));
  if (frame.magic != Protocol::MAGIC || frame.version != Protocol::VERSION ||
      frame.type != Protocol::DATA || frame.recordId == 0) {
    return false;
  }
  const uint32_t expected =
      crc32Ieee(reinterpret_cast<const uint8_t *>(&frame),
                sizeof(frame) - sizeof(frame.crc32));
  return frame.crc32 == expected && validModbus(frame.payload) &&
         validModbus(frame.payload + Config::MODBUS_SIZE);
}

// Handles millis() wraparound when checking a scheduled time.
bool timeReached(uint32_t now, uint32_t target) {
  return static_cast<int32_t>(now - target) >= 0;
}

// Stops safely if the wrong sketch is uploaded to this ESP32.
void stopWithError(const char *message) {
  Serial.println(message);
  while (true) delay(1000);
}

// Checks the physical board's station MAC.
bool ownMacIsCorrect() {
  uint8_t actual[6]{};
  return esp_wifi_get_mac(WIFI_IF_STA, actual) == ESP_OK &&
         memcmp(actual, Config::OWN_MAC, 6) == 0;
}

// Adds the Master as the next unencrypted ESP-NOW peer.
bool addPeer(const uint8_t mac[6]) {
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = Config::RADIO_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  const esp_err_t result = esp_now_add_peer(&peer);
  return result == ESP_OK || result == ESP_ERR_ESPNOW_EXIST;
}

// Stores only the newest callback frame; an older waiting frame is overwritten.
void storeLatest(const uint8_t *sender, const uint8_t *data, int length) {
  if (!sender || !data || length < 0) return;
  ReceivedFrame frame{};
  memcpy(frame.sender, sender, 6);
  frame.length = static_cast<int16_t>(length);
  size_t copyLength = static_cast<size_t>(length);
  if (copyLength > sizeof(frame.bytes)) copyLength = sizeof(frame.bytes);
  memcpy(frame.bytes, data, copyLength);

  portENTER_CRITICAL(&mailboxLock);
  if (latestFrameReady) ++mailboxOverwrites;
  latestFrame = frame;
  latestFrameReady = true;
  portEXIT_CRITICAL(&mailboxLock);
}

// Moves the latest callback frame into loop() without building a backlog.
bool takeLatest(ReceivedFrame &frame) {
  bool available = false;
  portENTER_CRITICAL(&mailboxLock);
  if (latestFrameReady) {
    frame = latestFrame;
    latestFrameReady = false;
    available = true;
  }
  portEXIT_CRITICAL(&mailboxLock);
  return available;
}

#if ESP_IDF_VERSION_MAJOR >= 5
void onDataReceived(const esp_now_recv_info_t *info, const uint8_t *data,
                    int length) {
  storeLatest(info ? info->src_addr : nullptr, data, length);
}
#else
void onDataReceived(const uint8_t *sender, const uint8_t *data, int length) {
  storeLatest(sender, data, length);
}
#endif

// Sends a few bounded copies to reduce loss without creating a backlog.
bool sendCopies(const Protocol::DataFrame &frame) {
  uint8_t queued = 0;
  for (uint8_t copy = 0; copy < Config::RADIO_COPIES; ++copy) {
    if (esp_now_send(Config::NEXT_MAC,
                     reinterpret_cast<const uint8_t *>(&frame),
                     sizeof(frame)) == ESP_OK) {
      ++queued;
    }
    if (copy + 1 < Config::RADIO_COPIES) delay(Config::COPY_GAP_MS);
  }
  if (queued == 0) ++radioSendFailures;
  return queued != 0;
}

// Accepts only Repeater 1 DATA; newer valid IDs move to Master immediately.
void processReceived(const ReceivedFrame &received) {
  if (memcmp(received.sender, Config::PREVIOUS_MAC, 6) != 0) {
    ++rejected;
    return;
  }

  Protocol::DataFrame frame{};
  if (!decodeData(received.bytes, received.length, frame)) {
    ++rejected;
    return;
  }
  if (lastForwardedId != 0 && frame.recordId <= lastForwardedId) {
    ++duplicates;
    return;
  }
  if (sendCopies(frame)) {
    lastForwardedId = frame.recordId;
    ++forwarded;
  }
}

// Prints local diagnostics once per second; these never enter LabVIEW.
void printStatsWhenDue() {
  const uint32_t now = millis();
  if (!timeReached(now, nextStatsAt)) return;
  nextStatsAt = now + 1000;
  Serial.printf("#R2 forwarded=%lu duplicate=%lu rejected=%lu "
                "radioFail=%lu overwrite=%lu\n",
                static_cast<unsigned long>(forwarded),
                static_cast<unsigned long>(duplicates),
                static_cast<unsigned long>(rejected),
                static_cast<unsigned long>(radioSendFailures),
                static_cast<unsigned long>(mailboxOverwrites));
}

void setup() {
  Serial.begin(Config::USB_BAUD);
  delay(300);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (!ownMacIsCorrect()) stopWithError("FATAL: wrong Repeater 2 MAC.");

  if (esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK ||
      esp_wifi_set_channel(Config::RADIO_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK ||
      esp_now_init() != ESP_OK || !addPeer(Config::NEXT_MAC) ||
      esp_now_register_recv_cb(onDataReceived) != ESP_OK) {
    stopWithError("FATAL: Repeater 2 setup failed.");
  }

  nextStatsAt = millis() + 1000;
  Serial.println("#REPEATER 2 REALTIME READY");
}

void loop() {
  ReceivedFrame received{};
  while (takeLatest(received)) {
    processReceived(received);
  }
  printStatsWhenDue();
  delay(1);
}