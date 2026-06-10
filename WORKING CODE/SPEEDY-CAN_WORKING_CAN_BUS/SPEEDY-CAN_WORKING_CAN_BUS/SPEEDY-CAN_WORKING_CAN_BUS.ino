#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <stddef.h>
#include <string.h>

// ESP32 UART2 pins. Wire Speeduino TX -> ESP32 RXD2, Speeduino RX -> ESP32 TXD2,
// and make sure both devices share ground.
static constexpr int ECU_RX_PIN = 16;
static constexpr int ECU_TX_PIN = 17;
static constexpr uint32_t ECU_BAUD = 115200;

static constexpr int CAN_INT_PIN = 4;
static constexpr int CAN_CS_PIN = 5;
static constexpr int CAN_SCK_PIN = 18;
static constexpr int CAN_MISO_PIN = 19;
static constexpr int CAN_MOSI_PIN = 23;

static constexpr uint32_t REQUEST_INTERVAL_MS = 20;
static constexpr uint32_t RESPONSE_TIMEOUT_MS = 100;
static constexpr uint32_t RAW_PACKET_QUIET_MS = 5;
static constexpr size_t BASE_STATUS_BYTES = 75;

static constexpr uint32_t LIVE_TIMEOUT_MS = 100;
static constexpr uint32_t FAST_FRAME_INTERVAL_MS = 20;
static constexpr uint32_t STATE_FRAME_INTERVAL_MS = 50;
static constexpr uint32_t SLOW_FRAME_INTERVAL_MS = 200;
static constexpr uint32_t CAN_HEARTBEAT_INTERVAL_MS = 250;
static constexpr bool CAN_BRIDGE_ENABLED = true;
static constexpr bool CAN_HEARTBEAT_ONLY = false;
static constexpr bool PRINT_OIL_CANDIDATES = false;
static constexpr size_t OIL_PRESSURE_OFFSET = 108;

static constexpr uint8_t STATUS_COMMAND = 'n';
static constexpr uint8_t STATUS_HEADER_2 = 0x32;
static constexpr size_t MAX_STATUS_PAYLOAD = 128;

HardwareSerial EcuSerial(2);
MCP_CAN CAN(CAN_CS_PIN);

struct __attribute__((packed)) SpeeduinoStatuses {
  uint8_t secl;
  uint8_t status1;
  uint8_t engine;
  uint8_t dwell;
  uint16_t MAP;
  uint8_t IAT;
  uint8_t coolant;
  uint8_t batCorrection;
  uint8_t battery10;
  uint8_t O2;
  uint8_t egoCorrection;
  uint8_t iatCorrection;
  uint8_t wueCorrection;
  uint16_t RPM;
  uint8_t TAEamount;
  uint8_t corrections;
  uint8_t ve;
  uint8_t afrTarget;
  uint16_t PW1;
  uint8_t tpsDOT;
  uint8_t advance;
  uint8_t TPS;
  uint16_t loopsPerSecond;
  uint16_t freeRAM;
  uint8_t boostTarget;
  uint8_t boostDuty;
  uint8_t spark;
  uint16_t rpmDOT;
  uint8_t ethanolPct;
  uint8_t flexCorrection;
  uint8_t flexIgnCorrection;
  uint8_t idleLoad;
  uint8_t testOutputs;
  uint8_t O2_2;
  uint8_t baro;
  uint16_t CANin_1;
  uint16_t CANin_2;
  uint16_t CANin_3;
  uint16_t CANin_4;
  uint16_t CANin_5;
  uint16_t CANin_6;
  uint16_t CANin_7;
  uint16_t CANin_8;
  uint16_t CANin_9;
  uint16_t CANin_10;
  uint16_t CANin_11;
  uint16_t CANin_12;
  uint16_t CANin_13;
  uint16_t CANin_14;
  uint16_t CANin_15;
  uint16_t CANin_16;
  uint8_t tpsADC;
  uint8_t getNextError;
  uint8_t launchCorrection;
  uint16_t PW2;
  uint16_t PW3;
  uint16_t PW4;
  uint8_t status3;
  uint8_t engineProtectStatus;
  uint16_t fuelLoad;
  uint16_t ignLoad;
  uint16_t injAngle;
  uint8_t idleDuty;
  uint8_t CLIdleTarget;
  uint8_t mapDOT;
  uint8_t vvt1Angle;
  uint8_t vvt1TargetAngle;
  uint8_t vvt1Duty;
  uint16_t flexBoostCorrection;
  uint8_t baroCorrection;
  uint8_t ASEValue;
  uint16_t vss;
  uint8_t gear;
  uint8_t fuelPressure;
  uint8_t oilPressure;
  uint8_t wmiPW;
  uint8_t status4;
  uint8_t vvt2Angle;
  uint8_t vvt2TargetAngle;
  uint8_t vvt2Duty;
  uint8_t outputsStatus;
  uint8_t fuelTemp;
  uint8_t fuelTempCorrection;
  uint8_t VE1;
  uint8_t VE2;
  uint8_t advance1;
  uint8_t advance2;
  uint8_t nitrous_status;
  uint8_t TS_SD_Status;
};

static_assert(sizeof(SpeeduinoStatuses) == 119, "Unexpected Speeduino status struct size");

SpeeduinoStatuses status;
uint8_t statusPayload[MAX_STATUS_PAYLOAD];
size_t lastPayloadLength = 0;
uint32_t lastStatusMillis = 0;
uint32_t canTxOkCount = 0;
uint32_t canTxFailCount = 0;
byte lastCanTxResult = CAN_OK;

static void drainEcuInput() {
  while (EcuSerial.available() > 0) {
    EcuSerial.read();
  }
}

static bool waitForBytes(size_t count, uint32_t timeoutMs) {
  const uint32_t start = millis();
  while (EcuSerial.available() < static_cast<int>(count)) {
    if (millis() - start >= timeoutMs) {
      return false;
    }
    delay(1);
  }
  return true;
}

static void printBytes(const uint8_t *data, size_t length) {
  for (size_t i = 0; i < length; i++) {
    if (data[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(data[i], HEX);
    if (i + 1 < length) {
      Serial.print(' ');
    }
  }
  Serial.println();
}

static bool copyStatusFromPayload(const uint8_t *payload, size_t payloadLength) {
  if (payloadLength < BASE_STATUS_BYTES) {
    Serial.print(F("Payload too short for base status struct: "));
    Serial.println(payloadLength);
    return false;
  }

  memset(&status, 0, sizeof(status));
  const size_t copyLength = min(payloadLength, sizeof(status));
  memcpy(&status, payload, copyLength);
  lastPayloadLength = payloadLength;
  return true;
}

static bool payloadHasByte(size_t offset) {
  return lastPayloadLength > offset;
}

static uint8_t raw8(size_t offset) {
  return payloadHasByte(offset) ? statusPayload[offset] : 0;
}

static uint16_t raw16LE(size_t offset) {
  return static_cast<uint16_t>(raw8(offset)) |
         (static_cast<uint16_t>(raw8(offset + 1)) << 8);
}

static uint8_t oilPressureRaw() {
  return raw8(OIL_PRESSURE_OFFSET);
}

static void printOilCandidates() {
  if (!PRINT_OIL_CANDIDATES) {
    return;
  }

  Serial.print(F(" OIL_MATCHES="));
  bool found = false;

  for (size_t i = 0; i < lastPayloadLength; i++) {
    const uint8_t value = raw8(i);
    if (value == 42 || value == 62 || value == 105 || value == 210) {
      Serial.print(F("B"));
      Serial.print(i);
      Serial.print('=');
      Serial.print(value);
      Serial.print(' ');
      found = true;
    }
  }

  for (size_t i = 0; i + 1 < lastPayloadLength; i++) {
    const uint16_t value = raw16LE(i);
    if (value >= 415 && value <= 430) {
      Serial.print(F("W"));
      Serial.print(i);
      Serial.print('=');
      Serial.print(value);
      Serial.print(' ');
      found = true;
    }
  }

  if (!found) {
    Serial.print(F("none"));
  }
}

static bool requestStatuses() {
  drainEcuInput();
  EcuSerial.write(STATUS_COMMAND);

  if (!waitForBytes(3, RESPONSE_TIMEOUT_MS)) {
    Serial.print(F("Short/no response to command n header. Bytes available="));
    Serial.println(EcuSerial.available());
    return false;
  }

  const uint8_t header1 = EcuSerial.read();
  const uint8_t header2 = EcuSerial.read();
  const uint8_t thirdByte = EcuSerial.read();

  if (header1 == STATUS_COMMAND && header2 == STATUS_HEADER_2) {
    if (thirdByte == 0 || thirdByte > sizeof(statusPayload)) {
      Serial.println(F("Invalid n payload length"));
      return false;
    }

    if (!waitForBytes(thirdByte, RESPONSE_TIMEOUT_MS)) {
      Serial.print(F("Timed out waiting for n payload. Bytes available="));
      Serial.println(EcuSerial.available());
      return false;
    }

    const size_t bytesRead = EcuSerial.readBytes(statusPayload, thirdByte);
    if (bytesRead != thirdByte) {
      Serial.print(F("n payload read mismatch: "));
      Serial.print(bytesRead);
      Serial.print(F("/"));
      Serial.println(thirdByte);
      return false;
    }

    if (EcuSerial.available() > 0) {
      Serial.print(F("Extra bytes after n payload="));
      Serial.println(EcuSerial.available());
      drainEcuInput();
    }

    const bool copied = copyStatusFromPayload(statusPayload, thirdByte);
    if (copied) {
      lastStatusMillis = millis();
    }
    return copied;
  }

  statusPayload[0] = header1;
  statusPayload[1] = header2;
  statusPayload[2] = thirdByte;

  if (!waitForBytes(BASE_STATUS_BYTES - 3, RESPONSE_TIMEOUT_MS)) {
    Serial.print(F("Short raw n base response. Bytes available="));
    Serial.println(EcuSerial.available());
    return false;
  }

  size_t payloadLength = 3 + EcuSerial.readBytes(statusPayload + 3, BASE_STATUS_BYTES - 3);
  if (payloadLength != BASE_STATUS_BYTES) {
    Serial.print(F("Raw n base read mismatch: "));
    Serial.print(payloadLength);
    Serial.print(F("/"));
    Serial.println(BASE_STATUS_BYTES);
    return false;
  }

  uint32_t lastByteTime = millis();
  const uint32_t start = millis();

  while (payloadLength < min(sizeof(statusPayload), sizeof(SpeeduinoStatuses))) {
    if (EcuSerial.available() > 0) {
      statusPayload[payloadLength++] = EcuSerial.read();
      lastByteTime = millis();
      continue;
    }

    if (millis() - lastByteTime >= RAW_PACKET_QUIET_MS) {
      break;
    }

    if (millis() - start >= RESPONSE_TIMEOUT_MS) {
      break;
    }

    delay(1);
  }

  const bool copied = copyStatusFromPayload(statusPayload, payloadLength);
  if (copied) {
    lastStatusMillis = millis();
  }
  return copied;
}

static void printStatus(const SpeeduinoStatuses &status) {
  Serial.print(F("RPM="));
  Serial.print(raw16LE(14));
  Serial.print(F(" MAP="));
  Serial.print(raw16LE(4));
  Serial.print(F("kPa BARO="));
  Serial.print(raw8(40));
  Serial.print(F("kPa CLT="));
  Serial.print(static_cast<int>(raw8(7)) - 40);
  Serial.print(F("C IAT="));
  Serial.print(static_cast<int>(raw8(6)) - 40);
  Serial.print(F("C TPS="));
  Serial.print(raw8(24));
  Serial.print(F("% BAT="));
  Serial.print(raw8(9) / 10.0f, 1);
  Serial.print(F("V AFR="));
  Serial.print(raw8(10) / 10.0f, 1);
  Serial.print(F(" AFR_TARGET="));
  Serial.print(raw8(19) / 10.0f, 1);
  Serial.print(F(" ADV="));
  Serial.print(raw8(23));
  Serial.print(F(" IDLE="));
  Serial.print(raw8(37));
  Serial.print(F(" VSS="));
  if (payloadHasByte(101)) {
    Serial.print(raw16LE(100));
  } else {
    Serial.print(F("N/A"));
  }
  Serial.print(F(" OIL="));
  Serial.print(oilPressureRaw());
  Serial.print(F(" OIL_SRC_B"));
  Serial.print(OIL_PRESSURE_OFFSET);
  Serial.print('=');
  Serial.print(raw8(OIL_PRESSURE_OFFSET));
  Serial.print(F(" OIL_B104="));
  Serial.print(raw8(104));
  printOilCandidates();
  Serial.print(F(" FUELP="));
  if (payloadHasByte(103)) {
    Serial.print(raw8(103));
  } else {
    Serial.print(F("N/A"));
  }
  Serial.print(F(" CANIN1="));
  Serial.print(raw16LE(41));
  Serial.print(F(" CANIN2="));
  Serial.print(raw16LE(43));
  Serial.print(F(" OUT="));
  if (payloadHasByte(110)) {
    Serial.print(raw8(110));
  } else {
    Serial.print(F("N/A"));
  }
  Serial.print(F(" LEN="));
  Serial.println(lastPayloadLength);
}

static void writeU16LE(uint8_t *data, size_t offset, uint16_t value) {
  data[offset] = value & 0xFF;
  data[offset + 1] = value >> 8;
}

static bool liveDataValid() {
  return lastStatusMillis != 0 && millis() - lastStatusMillis <= LIVE_TIMEOUT_MS;
}

static bool launchActive() {
  return bitRead(raw8(31), 0) || bitRead(raw8(31), 1);
}

static bool fanActive() {
  return payloadHasByte(110) && bitRead(raw8(110), 0);
}

static bool errorPresent() {
  return raw8(74) != 0 || bitRead(raw8(31), 5);
}

static bool sendCanFrame(uint16_t id, const uint8_t *data) {
  lastCanTxResult = CAN.sendMsgBuf(id, 0, 8, const_cast<uint8_t *>(data));
  if (lastCanTxResult == CAN_OK) {
    canTxOkCount++;
    return true;
  }

  canTxFailCount++;
  return false;
}

static const __FlashStringHelper *canSendResultName(byte result) {
  switch (result) {
    case CAN_OK:
      return F("CAN_OK");
    case CAN_FAILINIT:
      return F("CAN_FAILINIT");
    case CAN_FAILTX:
      return F("CAN_FAILTX");
    case CAN_MSGAVAIL:
      return F("CAN_MSGAVAIL");
    case CAN_NOMSG:
      return F("CAN_NOMSG");
    default:
      return F("UNKNOWN");
  }
}

static void sendFastLiveFrame(bool valid) {
  uint8_t data[8] = {};
  if (valid) {
    writeU16LE(data, 0, raw16LE(14));
    writeU16LE(data, 2, raw16LE(100));
    writeU16LE(data, 4, raw16LE(4));
    data[6] = raw8(10);
  }
  sendCanFrame(0x300, data);
}

static void sendFastNumericFrame(bool valid) {
  uint8_t data[8] = {};
  if (valid) {
    data[0] = oilPressureRaw();
    data[1] = raw8(7);
    data[2] = raw8(6);
    data[3] = raw8(9);
  }
  sendCanFrame(0x301, data);
}

static void sendStateFrame(bool valid) {
  uint8_t data[8] = {};
  if (valid) {
    bitSet(data[0], 0);
    if (launchActive()) {
      bitSet(data[0], 1);
    }
    if (fanActive()) {
      bitSet(data[0], 2);
    }
    if (errorPresent()) {
      bitSet(data[0], 3);
    }
  }
  data[1] = raw8(74);
  data[2] = raw8(1);
  data[3] = raw8(2);
  data[4] = payloadHasByte(110) ? raw8(110) : raw8(31);
  sendCanFrame(0x302, data);
}

static void sendSlowTargetsFrame(bool valid) {
  uint8_t data[8] = {};
  if (valid) {
    data[0] = raw8(19);
    data[1] = raw8(29);
    data[2] = raw8(30);
    data[3] = raw8(24);
  }
  sendCanFrame(0x303, data);
}

static void updateCanFrames() {
  static uint32_t lastFastFrame = 0;
  static uint32_t lastStateFrame = 0;
  static uint32_t lastSlowFrame = 0;
  static uint32_t lastHeartbeatFrame = 0;
  static uint8_t heartbeatCounter = 0;

  const uint32_t now = millis();
  const bool valid = liveDataValid();

  if (CAN_BRIDGE_ENABLED && !CAN_HEARTBEAT_ONLY && now - lastFastFrame >= FAST_FRAME_INTERVAL_MS) {
    lastFastFrame = now;
    sendFastLiveFrame(valid);
    sendFastNumericFrame(valid);
  }

  if (CAN_BRIDGE_ENABLED && !CAN_HEARTBEAT_ONLY && now - lastStateFrame >= STATE_FRAME_INTERVAL_MS) {
    lastStateFrame = now;
    sendStateFrame(valid);
  }

  if (CAN_BRIDGE_ENABLED && !CAN_HEARTBEAT_ONLY && now - lastSlowFrame >= SLOW_FRAME_INTERVAL_MS) {
    lastSlowFrame = now;
    sendSlowTargetsFrame(valid);
  }

  if (CAN_BRIDGE_ENABLED && now - lastHeartbeatFrame >= CAN_HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatFrame = now;
    uint8_t data[8] = {
      heartbeatCounter++,
      static_cast<uint8_t>(valid),
      static_cast<uint8_t>(lastPayloadLength),
      static_cast<uint8_t>(canTxFailCount & 0xFF),
      0,
      0,
      0,
      0,
    };
    sendCanFrame(0x7E0, data);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println(F("Speeduino Serial3 probe"));
  Serial.println(F("Polling command 'n' on ESP32 Serial2"));
  Serial.print(F("Full status struct bytes="));
  Serial.println(sizeof(SpeeduinoStatuses));

  EcuSerial.begin(ECU_BAUD, SERIAL_8N1, ECU_RX_PIN, ECU_TX_PIN);
  EcuSerial.setTimeout(RESPONSE_TIMEOUT_MS);

  SPI.begin(CAN_SCK_PIN, CAN_MISO_PIN, CAN_MOSI_PIN, CAN_CS_PIN);
  pinMode(CAN_INT_PIN, INPUT);

  while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println(F("CAN init failed, retrying..."));
    delay(500);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println(F("MCP2515 CAN started at 500 kbps, 8 MHz clock"));
}

void loop() {
  static uint32_t lastRequest = 0;
  if (millis() - lastRequest >= REQUEST_INTERVAL_MS) {
    lastRequest = millis();

    if (requestStatuses()) {
      static uint32_t lastPrint = 0;
      if (millis() - lastPrint >= 500) {
        lastPrint = millis();
        printStatus(status);
        Serial.print(F("CAN_TX_OK="));
        Serial.print(canTxOkCount);
        Serial.print(F(" CAN_TX_FAIL="));
        Serial.print(canTxFailCount);
        Serial.print(F(" LAST_SEND="));
        Serial.print(canSendResultName(lastCanTxResult));
        Serial.print(F("("));
        Serial.print(lastCanTxResult);
        Serial.println(F(")"));
      }
    }
  }
  updateCanFrames();
}
