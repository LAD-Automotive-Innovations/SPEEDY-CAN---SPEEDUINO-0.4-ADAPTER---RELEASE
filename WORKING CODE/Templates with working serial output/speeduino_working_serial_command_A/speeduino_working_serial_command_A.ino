#include <Arduino.h>

// ESP32 UART2 pins. Wire Speeduino TX -> ESP32 RXD2, Speeduino RX -> ESP32 TXD2,
// and make sure both devices share ground.
static constexpr int ECU_RX_PIN = 16;
static constexpr int ECU_TX_PIN = 17;
static constexpr uint32_t ECU_BAUD = 115200;

static constexpr uint8_t SPEEDUINO_COMMAND = 'A';
static constexpr uint32_t REQUEST_INTERVAL_MS = 250;
static constexpr uint32_t RESPONSE_TIMEOUT_MS = 100;

HardwareSerial EcuSerial(2);

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
  uint8_t AEamount;
  uint8_t corrections;
  uint8_t VE;
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
};

struct __attribute__((packed)) SpeeduinoStatusesExtended {
  uint8_t launchCorrection;
  uint16_t PW2;
  uint16_t PW3;
  uint16_t PW4;
  uint8_t status3;
  uint8_t engineProtectStatus;
  uint8_t idleDuty;
  uint8_t CLIdleTarget;
  uint8_t mapDOT;
  uint8_t vss;
  uint8_t gear;
  uint8_t fuelPressure;
  uint8_t oilPressure;
};

static_assert(sizeof(SpeeduinoStatuses) == 75, "Unexpected Speeduino status struct size");

SpeeduinoStatuses status;

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

static bool requestStatuses() {
  drainEcuInput();
  EcuSerial.write(SPEEDUINO_COMMAND);

  if (!waitForBytes(sizeof(SpeeduinoStatuses), RESPONSE_TIMEOUT_MS)) {
    Serial.print(F("Short/no response to command A. Bytes available="));
    Serial.println(EcuSerial.available());
    if (EcuSerial.available() > 0) {
      uint8_t debugBytes[16];
      const size_t bytesToRead = min(static_cast<size_t>(EcuSerial.available()), sizeof(debugBytes));
      const size_t bytesRead = EcuSerial.readBytes(debugBytes, bytesToRead);
      Serial.print(F("First bytes: "));
      printBytes(debugBytes, bytesRead);
    }
    return false;
  }

  const size_t bytesRead = EcuSerial.readBytes(reinterpret_cast<uint8_t *>(&status), sizeof(status));
  if (bytesRead != sizeof(status)) {
    Serial.print(F("Payload read mismatch: "));
    Serial.print(bytesRead);
    Serial.print(F("/"));
    Serial.println(sizeof(status));
    return false;
  }

  if (EcuSerial.available() > 0) {
    Serial.print(F("Extra bytes after 75-byte status struct="));
    Serial.println(EcuSerial.available());
    drainEcuInput();
  }

  return true;
}

static void printStatus(const SpeeduinoStatuses &status) {
  Serial.print(F("RPM="));
  Serial.print(status.RPM);
  Serial.print(F(" MAP="));
  Serial.print(status.MAP);
  Serial.print(F("kPa BARO="));
  Serial.print(status.baro);
  Serial.print(F("kPa CLT="));
  Serial.print(static_cast<int>(status.coolant) - 40);
  Serial.print(F("C IAT="));
  Serial.print(static_cast<int>(status.IAT) - 40);
  Serial.print(F("C TPS="));
  Serial.print(status.TPS);
  Serial.print(F("% BAT="));
  Serial.print(status.battery10 / 10.0f, 1);
  Serial.print(F("V AFR="));
  Serial.print(status.O2 / 10.0f, 1);
  Serial.print(F(" AFR_TARGET="));
  Serial.print(status.afrTarget / 10.0f, 1);
  Serial.print(F(" ADV="));
  Serial.print(status.advance);
  Serial.print(F(" IDLE="));
  Serial.println(status.idleLoad);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println(F("Speeduino Serial3 probe"));
  Serial.println(F("Polling command 'A' on ESP32 Serial2"));
  Serial.print(F("Expecting status struct bytes="));
  Serial.println(sizeof(SpeeduinoStatuses));

  EcuSerial.begin(ECU_BAUD, SERIAL_8N1, ECU_RX_PIN, ECU_TX_PIN);
  EcuSerial.setTimeout(RESPONSE_TIMEOUT_MS);
}

void loop() {
  static uint32_t lastRequest = 0;
  if (millis() - lastRequest < REQUEST_INTERVAL_MS) {
    return;
  }
  lastRequest = millis();

  if (requestStatuses()) {
    printStatus(status);
  }
}
