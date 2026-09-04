#include <Arduino.h>
#include <WiFi.h>
#include <esp_idf_version.h>
#include <esp_now.h>
#include <esp_wifi.h>

// Master: validates each current radio record and prints it to LabVIEW once.
// Output: D;<Record ID>;<Sensor time ms>;<28 uppercase hex characters>\n

namespace Config {
constexpr uint32_t USB_BAUD = 115200;
constexpr uint8_t RADIO_CHANNEL = 1;
constexpr size_t MODBUS_SIZE = 7;
constexpr size_t PAYLOAD_SIZE = 14;

const uint8_t OWN_MAC[6] = {0x68, 0x09, 0x47, 0x5C, 0x02, 0x9C};
const uint8_t PREVIOUS_MAC[6] = {0x68, 0x09, 0x47, 0x9D, 0x32, 0xE4};
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
uint64_t lastPrintedId = 0;

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

// Writes exactly one LabVIEW line for one new valid Record ID.
void printForLabView(const Protocol::DataFrame &frame) {
  static const char HEX_DIGITS[] = "0123456789ABCDEF";
  char line[96]{};
  const int prefixLength =
      snprintf(line, sizeof(line), "D;%llu;%llu;",
               static_cast<unsigned long long>(frame.recordId),
               static_cast<unsigned long long>(frame.sampleTimeMs));
  if (prefixLength <= 0) return;

  size_t position = static_cast<size_t>(prefixLength);
  if (position + 2U * Config::PAYLOAD_SIZE + 1U > sizeof(line)) return;
  for (size_t i = 0; i < Config::PAYLOAD_SIZE; ++i) {
    line[position++] = HEX_DIGITS[frame.payload[i] >> 4];
    line[position++] = HEX_DIGITS[frame.payload[i] & 0x0F];
  }
  line[position++] = '\n';
  Serial.write(reinterpret_cast<const uint8_t *>(line), position);
}

// Accepts only Repeater 2 DATA and suppresses repeated or older Record IDs.
void processReceived(const ReceivedFrame &received) {
  if (memcmp(received.sender, Config::PREVIOUS_MAC, 6) != 0) return;

  Protocol::DataFrame frame{};
  if (!decodeData(received.bytes, received.length, frame)) return;
  if (lastPrintedId != 0 && frame.recordId <= lastPrintedId) return;

  printForLabView(frame);
  lastPrintedId = frame.recordId;
}

// Discards old A;ID lines if the existing LabVIEW VI still sends them.
void discardLabViewInput() {
  while (Serial.available() > 0) (void)Serial.read();
}

void setup() {
  Serial.begin(Config::USB_BAUD);
  delay(300);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (!ownMacIsCorrect()) stopWithError("FATAL: wrong Master ESP MAC.");

  if (esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK ||
      esp_wifi_set_channel(Config::RADIO_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK ||
      esp_now_init() != ESP_OK ||
      esp_now_register_recv_cb(onDataReceived) != ESP_OK) {
    stopWithError("FATAL: Master setup failed.");
  }

  Serial.println("#MASTER REALTIME READY");
  Serial.println("#FORMAT D;<Record ID>;<Sensor time ms>;<28 HEX>");
}

void loop() {
  discardLabViewInput();

  ReceivedFrame received{};
  while (takeLatest(received)) {
    processReceived(received);
  }
  delay(1);
}