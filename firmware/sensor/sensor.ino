#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_timer.h>
#include <esp_wifi.h>

// Reads both RS485 sensors and immediately sends each valid pair.
// There is deliberately no historical queue: old wind data is never replayed.

namespace Config {
constexpr uint32_t USB_BAUD = 115200;
constexpr uint32_t MODBUS_BAUD = 9600;
constexpr uint8_t RADIO_CHANNEL = 1;
constexpr uint32_t SAMPLE_PERIOD_MS = 150;      // Target: 20 paired samples/s.
constexpr uint32_t RESPONSE_TIMEOUT_MS = 100;
constexpr uint32_t INTER_BYTE_TIMEOUT_MS = 10;
constexpr uint8_t RADIO_COPIES = 3;            // Bounded radio redundancy.
constexpr uint32_t COPY_GAP_MS = 3;

constexpr int DIRECTION_RX = 16;
constexpr int DIRECTION_TX = 17;
constexpr int DIRECTION_DE_RE = 22;
constexpr int SPEED_RX = 18;
constexpr int SPEED_TX = 19;
constexpr int SPEED_DE_RE = 23;

constexpr size_t MODBUS_SIZE = 7;
constexpr size_t PAYLOAD_SIZE = 14;

const uint8_t OWN_MAC[6] = {0x68, 0x09, 0x47, 0x9D, 0x08, 0xE0};
const uint8_t NEXT_MAC[6] = {0x20, 0x9B, 0xA9, 0x68, 0xAB, 0x9C};

// Slave 1, Read Holding Register, register 4, one register, Modbus CRC.
const uint8_t READ_VALUE[8] = {0x01, 0x03, 0x00, 0x04,
                               0x00, 0x01, 0xC5, 0xCB};
}  // namespace Config

namespace Protocol {
constexpr uint16_t MAGIC = 0x574E;             // Identifies a Wind Network frame.
constexpr uint8_t VERSION = 3;                 // Adds Sensor acquisition time.
constexpr uint8_t DATA = 1;

#pragma pack(push, 1)
struct DataFrame {
  uint16_t magic;
  uint8_t version;
  uint8_t type;
  uint64_t recordId;
  uint64_t sampleTimeMs;                       // Milliseconds since Sensor boot.
  uint8_t payload[Config::PAYLOAD_SIZE];        // Speed[7], direction[7].
  uint32_t crc32;
};
#pragma pack(pop)

static_assert(sizeof(DataFrame) == 38, "Unexpected DataFrame size");
}  // namespace Protocol

HardwareSerial SpeedBus(1);
HardwareSerial DirectionBus(2);

uint64_t nextRecordId = 1;
uint32_t nextSampleAt = 0;
uint32_t nextStatsAt = 0;
uint32_t validPairs = 0;
uint32_t invalidPairs = 0;
uint32_t missedSlots = 0;
uint32_t radioSendFailures = 0;
size_t lastSpeedBytes = 0;
size_t lastDirectionBytes = 0;

// Calculates CRC-32/ISO-HDLC for the complete ESP-NOW application frame.
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

// Calculates the Modbus RTU CRC-16 used by each physical sensor.
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

// Verifies the fixed response header and both Modbus CRC bytes.
bool validModbus(const uint8_t frame[Config::MODBUS_SIZE]) {
  if (frame[0] != 0x01 || frame[1] != 0x03 || frame[2] != 0x02) return false;
  const uint16_t crc = modbusCrc16(frame, 5);
  return frame[5] == static_cast<uint8_t>(crc) &&
         frame[6] == static_cast<uint8_t>(crc >> 8);
}

// Handles millis() wraparound when checking a scheduled time.
bool timeReached(uint32_t now, uint32_t target) {
  return static_cast<int32_t>(now - target) >= 0;
}

// Stops safely if the wrong sketch is uploaded to this physical ESP32.
void stopWithError(const char *message) {
  Serial.println(message);
  while (true) delay(1000);
}

// Checks this board's station MAC against the configured Sensor MAC.
bool ownMacIsCorrect() {
  uint8_t actual[6]{};
  return esp_wifi_get_mac(WIFI_IF_STA, actual) == ESP_OK &&
         memcmp(actual, Config::OWN_MAC, 6) == 0;
}

// Adds Repeater 1 as the only ESP-NOW destination.
bool addPeer(const uint8_t mac[6]) {
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = Config::RADIO_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  const esp_err_t result = esp_now_add_peer(&peer);
  return result == ESP_OK || result == ESP_ERR_ESPNOW_EXIST;
}

// Creates a boot epoch in flash so record IDs stay unique after resets.
bool beginRecordIds() {
  Preferences preferences;
  if (!preferences.begin("windlink", false)) return false;

  uint32_t epoch = preferences.getUInt("bootEpoch", 0);
  if (epoch == UINT32_MAX) {
    preferences.end();
    return false;
  }
  ++epoch;
  const size_t written = preferences.putUInt("bootEpoch", epoch);
  preferences.end();
  if (written != sizeof(epoch)) return false;

  nextRecordId = (static_cast<uint64_t>(epoch) << 32) | 1ULL;
  return true;
}

// Returns one monotonically increasing ID for each scheduled sample slot.
uint64_t allocateRecordId() {
  if (static_cast<uint32_t>(nextRecordId) == UINT32_MAX) {
    stopWithError("FATAL: record sequence exhausted; restart the Sensor.");
  }
  return nextRecordId++;
}

// Clears old UART bytes before a new Modbus request.
void clearUart(HardwareSerial &bus) {
  while (bus.available() > 0) (void)bus.read();
}

// Requests one register and returns only a complete, CRC-valid response.
bool readSensor(HardwareSerial &bus, int deRePin,
                uint8_t response[Config::MODBUS_SIZE], size_t &byteCount) {
  clearUart(bus);
  byteCount = 0;

  digitalWrite(deRePin, HIGH);
  delayMicroseconds(100);
  bus.write(Config::READ_VALUE, sizeof(Config::READ_VALUE));
  bus.flush();
  delayMicroseconds(100);
  digitalWrite(deRePin, LOW);

  const uint32_t startedAt = millis();
  uint32_t lastByteAt = startedAt;
  while (millis() - startedAt < Config::RESPONSE_TIMEOUT_MS) {
    while (bus.available() > 0) {
      const int value = bus.read();
      if (value < 0) continue;
      if (byteCount >= Config::MODBUS_SIZE) {
        clearUart(bus);
        return false;
      }
      response[byteCount++] = static_cast<uint8_t>(value);
      lastByteAt = millis();
    }

    if (byteCount == Config::MODBUS_SIZE) return validModbus(response);
    if (byteCount > 0 && millis() - lastByteAt >= Config::INTER_BYTE_TIMEOUT_MS) {
      break;
    }
    delay(1);
  }
  return false;
}

// Sends several copies of the same current record; it never waits on an ACK.
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

// Reads speed then direction, timestamps the pair, and sends it immediately.
void sampleAndSend(uint64_t recordId) {
  const uint64_t sampleTimeMs =
      static_cast<uint64_t>(esp_timer_get_time()) / 1000ULL;
  uint8_t speed[Config::MODBUS_SIZE]{};
  uint8_t direction[Config::MODBUS_SIZE]{};

  const bool speedOk = readSensor(SpeedBus, Config::SPEED_DE_RE,
                                  speed, lastSpeedBytes);
  const bool directionOk = readSensor(DirectionBus, Config::DIRECTION_DE_RE,
                                      direction, lastDirectionBytes);
  if (!speedOk || !directionOk) {
    ++invalidPairs;
    return;
  }

  Protocol::DataFrame frame{};
  frame.magic = Protocol::MAGIC;
  frame.version = Protocol::VERSION;
  frame.type = Protocol::DATA;
  frame.recordId = recordId;
  frame.sampleTimeMs = sampleTimeMs;
  memcpy(frame.payload, speed, Config::MODBUS_SIZE);
  memcpy(frame.payload + Config::MODBUS_SIZE, direction, Config::MODBUS_SIZE);
  frame.crc32 = crc32Ieee(reinterpret_cast<const uint8_t *>(&frame),
                          sizeof(frame) - sizeof(frame.crc32));

  sendCopies(frame);
  ++validPairs;
}

// Prints compact diagnostics on the Sensor's own USB port once per second.
void printStatsWhenDue() {
  const uint32_t now = millis();
  if (!timeReached(now, nextStatsAt)) return;
  nextStatsAt = now + 1000;
  Serial.printf("#SENSOR valid=%lu invalid=%lu missed=%lu radioFail=%lu "
                "rs485=%u/7,%u/7\n",
                static_cast<unsigned long>(validPairs),
                static_cast<unsigned long>(invalidPairs),
                static_cast<unsigned long>(missedSlots),
                static_cast<unsigned long>(radioSendFailures),
                static_cast<unsigned>(lastSpeedBytes),
                static_cast<unsigned>(lastDirectionBytes));
}

void setup() {
  Serial.begin(Config::USB_BAUD);
  delay(300);

  pinMode(Config::SPEED_DE_RE, OUTPUT);
  pinMode(Config::DIRECTION_DE_RE, OUTPUT);
  digitalWrite(Config::SPEED_DE_RE, LOW);
  digitalWrite(Config::DIRECTION_DE_RE, LOW);
  SpeedBus.begin(Config::MODBUS_BAUD, SERIAL_8N1,
                 Config::SPEED_RX, Config::SPEED_TX);
  DirectionBus.begin(Config::MODBUS_BAUD, SERIAL_8N1,
                     Config::DIRECTION_RX, Config::DIRECTION_TX);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (!ownMacIsCorrect()) stopWithError("FATAL: wrong Sensor ESP MAC.");
  if (esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK ||
      esp_wifi_set_channel(Config::RADIO_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK ||
      esp_now_init() != ESP_OK || !addPeer(Config::NEXT_MAC)) {
    stopWithError("FATAL: Sensor radio setup failed.");
  }
  if (!beginRecordIds()) stopWithError("FATAL: record ID storage failed.");

  nextSampleAt = millis();
  nextStatsAt = millis() + 1000;
  Serial.println("#SENSOR REALTIME READY");
}

void loop() {
  const uint32_t now = millis();
  if (timeReached(now, nextSampleAt)) {
    const uint32_t dueSlots = (now - nextSampleAt) / Config::SAMPLE_PERIOD_MS + 1U;
    nextSampleAt += dueSlots * Config::SAMPLE_PERIOD_MS;

    for (uint32_t skipped = 1; skipped < dueSlots; ++skipped) {
      (void)allocateRecordId();
      ++missedSlots;
    }
    sampleAndSend(allocateRecordId());
  }

  printStatsWhenDue();
  delay(1);
}