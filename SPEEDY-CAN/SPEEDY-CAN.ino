```cpp
#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>   // autowp/arduino-mcp2515

// SPEEDY-CAN for Speeduino 0.4 (Mega2560 based)
// ESP32 firmware template: request realtime data from Speeduino over UART,
// decode it into clean internal signals, and publish generic CAN via MCP2515.
//
// HOW THIS FILE IS ORGANIZED:
//   1) Fixed SPEEDY-CAN board support      -> do not normally edit
//   2) Fixed Speeduino serial + decode     -> do not normally edit
//   3) Internal signal layer               -> expand carefully if needed
//   4) User-editable CAN output section    -> main area for cluster/dash changes
//   5) Future lamp output section          -> reserved for MOSFET warning lamps
//
// For most users, the main editable area is the CAN ID section and the frame
// builder functions near the bottom of the file.

// -----------------------------------------------------------------------------
// Serial state
// -----------------------------------------------------------------------------

#define NOTHING_RECEIVED 0
#define A_MESSAGE        1

// -----------------------------------------------------------------------------
// Timing
// -----------------------------------------------------------------------------

#define SerialUpdateRate   30  // Speeduino request rate (Hz)
#define FastCanUpdateRate  50  // Fast engine + fast status frames
#define MediumCanUpdateRate 10 // Medium sensor frame
#define SlowCanUpdateRate   5  // Slow aux frame

static const uint8_t SPEEDY_A_PAYLOAD_LEN = 75;   // payload bytes after leading 'A' for Speeduino legacy A command
static const uint32_t SPEEDY_TIMEOUT_MS   = 500;

// -----------------------------------------------------------------------------
// FIXED SPEEDY-CAN LIBRARY / CAN CONTROLLER SETTINGS
// This section is board-level setup. Normal users should not need to edit it,
// except for MCP2515_CLOCK if their fitted module uses a different crystal.
// -----------------------------------------------------------------------------

static const auto MCP2515_BITRATE = CAN_500KBPS;
static const auto MCP2515_CLOCK   = MCP_8MHZ;   // Users can change this if their fitted module uses a different crystal

// -----------------------------------------------------------------------------
// FIXED SPEEDY-CAN BOARD SUPPORT (ESP32 PINOUT)
// These are the physical board pin assignments for SPEEDY-CAN.
// Normal users should not edit this section.
// -----------------------------------------------------------------------------

static const int SPEEDUINO_RX_PIN = 17;   // ESP32 RX from Mega Serial3 TX (through divider)
static const int SPEEDUINO_TX_PIN = 16;   // ESP32 TX to Mega Serial3 RX

static const int MCP2515_INT_PIN  = 4;    // active-LOW, open-drain interrupt
static const int MCP2515_SCK_PIN  = 18;
static const int MCP2515_MOSI_PIN = 23;
static const int MCP2515_MISO_PIN = 19;
static const int MCP2515_CS_PIN   = 5;    // external pull-up to 3V3 present

static const int LAMP1_PIN = 14;
static const int LAMP2_PIN = 13;
static const int LAMP3_PIN = 21;
static const int LAMP4_PIN = 22;
static const int LAMP5_PIN = 27;
static const int LAMP6_PIN = 26;
static const int LAMP7_PIN = 25;
static const int LAMP8_PIN = 33;

static const int lampPins[8] = {
  LAMP1_PIN, LAMP2_PIN, LAMP3_PIN, LAMP4_PIN,
  LAMP5_PIN, LAMP6_PIN, LAMP7_PIN, LAMP8_PIN
};

HardwareSerial SpeeduinoPort(2);   // UART2 on ESP32 for Speeduino secondary serial
MCP2515 mcp2515(MCP2515_CS_PIN);

// -----------------------------------------------------------------------------
// FIXED RAW SPEEDUINO PACKET MIRROR
// This struct mirrors Speeduino realtime data as directly as possible.
// Keep this broad even if all fields are not used yet.
// Normal users should not need to edit this section.
// -----------------------------------------------------------------------------

struct statuses {
  uint8_t secl;
  uint8_t status1;
  uint8_t engine;
  uint8_t dwell;
  uint16_t MAP;
  uint8_t IAT;
  uint8_t CLT;
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
  int8_t advance;
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
};

statuses currentStatus;

// -----------------------------------------------------------------------------
// INTERNAL SIGNAL LAYER
// All CAN builders and future lamp logic should read from here rather than
// directly from the raw packet mirror above.
// Expand this layer carefully when adding new reusable signals.
// -----------------------------------------------------------------------------

struct normalizedSignals {
  uint16_t rpm;
  int16_t coolantC;
  int16_t intakeC;
  uint8_t throttlePct;
  uint16_t mapKpa;
  uint8_t batteryDeciVolts;
  uint8_t afrRaw;
  uint8_t afrTargetRaw;
  uint8_t ethanolPct;
  uint8_t boostTargetRaw;
  uint8_t boostDutyPct;
  uint8_t sparkRaw;
  uint16_t auxIn1;
  uint16_t auxIn2;
  uint8_t status1Raw;
  uint8_t engineRaw;
  uint8_t testOutputsRaw;
  uint8_t errorCodeRaw;
  bool overtempActive;
  bool celActive;
  bool launchActive;
  bool valid;
};

normalizedSignals liveSignals;

// -----------------------------------------------------------------------------
// USER-EDITABLE CEL CONFIGURATION
// Configure which conditions can illuminate the CEL. The final CEL result feeds
// both the physical CEL lamp output and the CAN warning indicator.
//
// Notes:
//   - Comms fault, error code, overtemp, and overboost are enabled by default.
//   - AFR deviation CEL is disabled by default until the AFR source / scaling is
//     properly defined for the target installation.
//   - Overboost uses MAP in absolute kPa.
// -----------------------------------------------------------------------------

enum IndicatorPattern : uint8_t {
  PATTERN_SOLID = 0,
  PATTERN_FLASH_1HZ,
  PATTERN_FLASH_3HZ,
  PATTERN_FLASH_FAST
};

enum AuxSignalSource : uint8_t {
  AUX_SOURCE_NONE = 0,
  AUX_SOURCE_CANIN1,
  AUX_SOURCE_CANIN2
};

struct celLogicConfig {
  bool enableCommsFault;
  bool enableErrorCode;
  bool enableOvertemp;
  int16_t overtempThresholdC;

  bool enableOverboost;
  uint16_t overboostThresholdKpa;
  uint16_t overboostMinRpm;
  uint8_t overboostMinThrottlePct;
  uint32_t overboostDelayMs;

  bool enableOilPressureFault;
  AuxSignalSource oilPressureSource;
  uint16_t oilPressureMinRaw;
  uint16_t oilPressureMinRpm;
  uint32_t oilPressureDelayMs;

  bool enableLeanFault;
  uint8_t leanDeviationThresholdRaw;
  uint16_t leanMinRpm;
  uint8_t leanMinThrottlePct;
  uint32_t leanDelayMs;

  bool enableRichFault;
  uint8_t richDeviationThresholdRaw;
  uint16_t richMinRpm;
  uint8_t richMinThrottlePct;
  uint32_t richDelayMs;

  bool enableBatteryFault;
  uint8_t undervoltThresholdDeciV;
  uint8_t overvoltThresholdDeciV;
  uint32_t batteryDelayMs;
  uint32_t batteryStartupIgnoreMs;

  bool enableLaunchOnCel;
};

struct celRuntimeState {
  bool commFaultActive;
  bool errorCodeActive;
  bool overtempActive;
  bool overboostActive;
  bool oilPressureActive;
  bool leanFaultActive;
  bool richFaultActive;
  bool batteryFaultActive;
  bool launchCelActive;
  bool finalCelActive;
  uint16_t reasonBits;
  IndicatorPattern outputPattern;
  uint32_t overboostStartMs;
  uint32_t oilPressureStartMs;
  uint32_t leanFaultStartMs;
  uint32_t richFaultStartMs;
  uint32_t batteryFaultStartMs;
};

static const uint16_t CEL_REASON_COMMS_FAULT = (1U << 0);
static const uint16_t CEL_REASON_ERROR_CODE  = (1U << 1);
static const uint16_t CEL_REASON_OVERTEMP    = (1U << 2);
static const uint16_t CEL_REASON_OVERBOOST   = (1U << 3);
static const uint16_t CEL_REASON_OIL_PRESS   = (1U << 4);
static const uint16_t CEL_REASON_LEAN        = (1U << 5);
static const uint16_t CEL_REASON_RICH        = (1U << 6);
static const uint16_t CEL_REASON_BATTERY     = (1U << 7);
static const uint16_t CEL_REASON_LAUNCH      = (1U << 8);

celLogicConfig celConfig = {
  true,              // enableCommsFault
  true,              // enableErrorCode
  true,              // enableOvertemp
  115,               // overtempThresholdC

  true,              // enableOverboost
  220,               // overboostThresholdKpa (absolute MAP)
  2000,              // overboostMinRpm
  40,                // overboostMinThrottlePct
  250,               // overboostDelayMs

  false,             // enableOilPressureFault (enable when source/scaling is defined)
  AUX_SOURCE_NONE,   // oilPressureSource
  0,                 // oilPressureMinRaw
  1500,              // oilPressureMinRpm
  250,               // oilPressureDelayMs

  false,             // enableLeanFault (enable when AFR scaling is validated)
  15,                // leanDeviationThresholdRaw
  2000,              // leanMinRpm
  40,                // leanMinThrottlePct
  500,               // leanDelayMs

  false,             // enableRichFault (enable when AFR scaling is validated)
  15,                // richDeviationThresholdRaw
  2000,              // richMinRpm
  40,                // richMinThrottlePct
  500,               // richDelayMs

  true,              // enableBatteryFault
  110,               // undervoltThresholdDeciV
  150,               // overvoltThresholdDeciV
  1000,              // batteryDelayMs
  5000,              // batteryStartupIgnoreMs

  false              // enableLaunchOnCel
};

celRuntimeState celState;

// -----------------------------------------------------------------------------
// USER-EDITABLE WARNING / LAMP CONFIGURATION
// This is the main place to configure startup lamp test behavior, default lamp
// assignments, flash patterns, and CAN warning indicator routing.
//
// Default SPEEDY-CAN lamp layout:
//   Lamp 1 = CEL
//   Lamp 2 = Oil pressure warning
//   Lamp 3 = Launch active (flashing)
//   Lamp 4 = Fan active
//   Lamp 5-8 = Optional user-assigned outputs
//
// Enabled warning routes participate in startup prove-out.
// Optional disabled routes remain off unless the user enables them.
// -----------------------------------------------------------------------------

enum WarningSourceId : uint8_t {
  WARN_SRC_CEL = 0,
  WARN_SRC_OIL_PRESSURE,
  WARN_SRC_LAUNCH,
  WARN_SRC_FAN_ACTIVE,
  WARN_SRC_OPTION_5,
  WARN_SRC_OPTION_6,
  WARN_SRC_OPTION_7,
  WARN_SRC_OPTION_8,
  WARN_SRC_COUNT
};

struct warningRoutingConfig {
  bool enabled;
  uint8_t lampIndex;        // 0-7 for Lamp 1-8
  uint8_t canByteIndex;     // byte index within canAuxStatus
  uint8_t canBitIndex;      // bit index within that byte
  IndicatorPattern pattern;
};

struct warningRuntimeState {
  bool active[WARN_SRC_COUNT];
  bool lampTestActive;
};

static const uint32_t STARTUP_LAMP_TEST_MS = 3000;

warningRoutingConfig warningRoutes[WARN_SRC_COUNT] = {
  { true, 0, 0, 0, PATTERN_SOLID },         // CEL -> Lamp 1, CAN status byte 0 bit 0 (pattern resolved by CEL logic)
  { true, 1, 0, 1, PATTERN_FLASH_FAST },    // Oil pressure -> Lamp 2, CAN status byte 0 bit 1
  { true, 2, 0, 2, PATTERN_FLASH_1HZ },     // Launch -> Lamp 3, CAN status byte 0 bit 2
  { true, 3, 0, 3, PATTERN_SOLID },         // Fan active -> Lamp 4, CAN status byte 0 bit 3
  { false, 4, 0, 4, PATTERN_SOLID },        // Optional Lamp 5 / bit 4
  { false, 5, 0, 5, PATTERN_SOLID },        // Optional Lamp 6 / bit 5
  { false, 6, 0, 6, PATTERN_SOLID },        // Optional Lamp 7 / bit 6
  { false, 7, 0, 7, PATTERN_SOLID }         // Optional Lamp 8 / bit 7
};

warningRuntimeState warningState;
uint32_t startupTimeMs;

// -----------------------------------------------------------------------------
// Raw buffers and state
// -----------------------------------------------------------------------------

uint8_t SpeedyResponse[100];

bool data_error;
bool newData;
bool doRequest;

uint8_t SerialState;
uint8_t currentCommand;

uint32_t oldtime;
uint32_t lastSerialRequestMs;
uint32_t lastFastCanMs;
uint32_t lastMediumCanMs;
uint32_t lastSlowCanMs;

// -----------------------------------------------------------------------------
// USER-EDITABLE CAN IDS
// SPEEDY-CAN native sender-node CAN map.
// Users can change these IDs to suit a target cluster or receiver if needed.
// -----------------------------------------------------------------------------

static const uint16_t CAN_ID_FAST_ENGINE    = 0x300;
static const uint16_t CAN_ID_FAST_STATUS    = 0x301;
static const uint16_t CAN_ID_MEDIUM_SENSORS = 0x302;
static const uint16_t CAN_ID_SLOW_AUX       = 0x303;

struct can_frame canFastEngine;
struct can_frame canFastStatus;
struct can_frame canMediumSensors;
struct can_frame canSlowAux;

// -----------------------------------------------------------------------------
// HELPERS / INTERNAL UTILITIES
// These support the fixed decode path and generic packing helpers.
// Normal users usually do not need to edit this section.
// -----------------------------------------------------------------------------

static inline uint16_t u16FromLe(const uint8_t *buf, uint8_t lsbIndex) {
  return (uint16_t(buf[lsbIndex + 1]) << 8) | uint16_t(buf[lsbIndex]);
}

static inline void packU16Le(struct can_frame &frame, uint8_t startIndex, uint16_t value) {
  frame.data[startIndex]     = lowByte(value);
  frame.data[startIndex + 1] = highByte(value);
}

static inline uint8_t tempCToOffsetByte(int16_t tempC) {
  return (uint8_t)constrain(tempC + 40, 0, 255);
}

static inline void writeCanBit(struct can_frame &frame, uint8_t byteIndex, uint8_t bitIndex, bool state) {
  if (state) {
    frame.data[byteIndex] |= (1U << bitIndex);
  } else {
    frame.data[byteIndex] &= ~(1U << bitIndex);
  }
}

static inline bool startupLampTestActive() {
  return (millis() - startupTimeMs) < STARTUP_LAMP_TEST_MS;
}

static inline bool patternOutputActive(IndicatorPattern pattern) {
  if (startupLampTestActive()) {
    return true;
  }

  switch (pattern) {
    case PATTERN_FLASH_1HZ:
      return ((millis() / 500UL) % 2UL) != 0;

    case PATTERN_FLASH_3HZ:
      return ((millis() / 166UL) % 2UL) != 0;

    case PATTERN_FLASH_FAST:
      return ((millis() / 125UL) % 2UL) != 0;

    case PATTERN_SOLID:
    default:
      return true;
  }
}

static inline bool heldForMs(bool condition, uint32_t &startTimeMs, uint32_t delayMs) {
  if (!condition) {
    startTimeMs = 0;
    return false;
  }

  if (startTimeMs == 0) {
    startTimeMs = millis();
  }

  return (millis() - startTimeMs) >= delayMs;
}

static inline uint16_t getAuxSourceRaw(AuxSignalSource source) {
  switch (source) {
    case AUX_SOURCE_CANIN1:
      return liveSignals.auxIn1;

    case AUX_SOURCE_CANIN2:
      return liveSignals.auxIn2;

    case AUX_SOURCE_NONE:
    default:
      return 0;
  }
}

void initCanFrames() {
  memset(&canFastEngine, 0, sizeof(canFastEngine));
  memset(&canFastStatus, 0, sizeof(canFastStatus));
  memset(&canMediumSensors, 0, sizeof(canMediumSensors));
  memset(&canSlowAux, 0, sizeof(canSlowAux));

  canFastEngine.can_id  = CAN_ID_FAST_ENGINE;
  canFastEngine.can_dlc = 8;

  canFastStatus.can_id  = CAN_ID_FAST_STATUS;
  canFastStatus.can_dlc = 8;

  canMediumSensors.can_id  = CAN_ID_MEDIUM_SENSORS;
  canMediumSensors.can_dlc = 8;

  canSlowAux.can_id  = CAN_ID_SLOW_AUX;
  canSlowAux.can_dlc = 8;
}

void clearSignals() {
  memset(&liveSignals, 0, sizeof(liveSignals));
  liveSignals.valid = false;
}

void clearCelState() {
  memset(&celState, 0, sizeof(celState));
}

void clearWarningState() {
  memset(&warningState, 0, sizeof(warningState));
  warningState.lampTestActive = startupLampTestActive();
}

void clearCanPayloads() {
  memset(canFastEngine.data, 0, sizeof(canFastEngine.data));
  memset(canFastStatus.data, 0, sizeof(canFastStatus.data));
  memset(canMediumSensors.data, 0, sizeof(canMediumSensors.data));
  memset(canSlowAux.data, 0, sizeof(canSlowAux.data));
}

void initState() {
  startupTimeMs = millis();

  clearSignals();
  clearCelState();
  clearWarningState();
  initCanFrames();
  clearCanPayloads();

  data_error = false;
  newData = false;
  doRequest = true;

  SerialState = NOTHING_RECEIVED;
  currentCommand = 0;

  oldtime = millis();
  lastSerialRequestMs = 0;
  lastFastCanMs = 0;
  lastMediumCanMs = 0;
  lastSlowCanMs = 0;
}

void invalidateLiveData() {
  data_error = true;
  newData = false;
  clearSignals();
  updateWarningLogic();
  clearCanPayloads();
}

// -----------------------------------------------------------------------------
// FIXED SPEEDUINO REQUEST PATH
// This is the one-way request from ESP32 to Speeduino over the secondary serial link.
// Normal users should not need to edit this section.
// -----------------------------------------------------------------------------

void requestData() {
  if (!doRequest) {
    return;
  }

  SpeeduinoPort.write('A');
  doRequest = false;
  lastSerialRequestMs = millis();
}

// -----------------------------------------------------------------------------
// FIXED LEGACY A-PACKET DECODE
// This decodes the legacy Speeduino 'A' realtime packet into currentStatus.
// Keep packet alignment stable. Normal users should not edit this unless they
// are intentionally changing protocol support.
// -----------------------------------------------------------------------------

void decodeLegacyAPacket() {
  currentStatus.secl              = SpeedyResponse[0];
  currentStatus.status1           = SpeedyResponse[1];
  currentStatus.engine            = SpeedyResponse[2];
  currentStatus.dwell             = SpeedyResponse[3];
  currentStatus.MAP               = u16FromLe(SpeedyResponse, 4);
  currentStatus.IAT               = SpeedyResponse[6];
  currentStatus.CLT               = SpeedyResponse[7];
  currentStatus.batCorrection     = SpeedyResponse[8];
  currentStatus.battery10         = SpeedyResponse[9];
  currentStatus.O2                = SpeedyResponse[10];
  currentStatus.egoCorrection     = SpeedyResponse[11];
  currentStatus.iatCorrection     = SpeedyResponse[12];
  currentStatus.wueCorrection     = SpeedyResponse[13];
  currentStatus.RPM               = u16FromLe(SpeedyResponse, 14);
  currentStatus.AEamount          = SpeedyResponse[16];
  currentStatus.corrections       = SpeedyResponse[17];
  currentStatus.VE                = SpeedyResponse[18];
  currentStatus.afrTarget         = SpeedyResponse[19];
  currentStatus.PW1               = u16FromLe(SpeedyResponse, 20);
  currentStatus.tpsDOT            = SpeedyResponse[22];
  currentStatus.advance           = (int8_t)SpeedyResponse[23];
  currentStatus.TPS               = SpeedyResponse[24];
  currentStatus.loopsPerSecond    = u16FromLe(SpeedyResponse, 25);
  currentStatus.freeRAM           = u16FromLe(SpeedyResponse, 27);
  currentStatus.boostTarget       = SpeedyResponse[29];
  currentStatus.boostDuty         = SpeedyResponse[30];
  currentStatus.spark             = SpeedyResponse[31];
  currentStatus.rpmDOT            = u16FromLe(SpeedyResponse, 32);
  currentStatus.ethanolPct        = SpeedyResponse[34];
  currentStatus.flexCorrection    = SpeedyResponse[35];
  currentStatus.flexIgnCorrection = SpeedyResponse[36];
  currentStatus.idleLoad          = SpeedyResponse[37];
  currentStatus.testOutputs       = SpeedyResponse[38];
  currentStatus.O2_2              = SpeedyResponse[39];
  currentStatus.baro              = SpeedyResponse[40];
  currentStatus.CANin_1           = u16FromLe(SpeedyResponse, 41);
  currentStatus.CANin_2           = u16FromLe(SpeedyResponse, 43);
  currentStatus.CANin_3           = u16FromLe(SpeedyResponse, 45);
  currentStatus.CANin_4           = u16FromLe(SpeedyResponse, 47);
  currentStatus.CANin_5           = u16FromLe(SpeedyResponse, 49);
  currentStatus.CANin_6           = u16FromLe(SpeedyResponse, 51);
  currentStatus.CANin_7           = u16FromLe(SpeedyResponse, 53);
  currentStatus.CANin_8           = u16FromLe(SpeedyResponse, 55);
  currentStatus.CANin_9           = u16FromLe(SpeedyResponse, 57);
  currentStatus.CANin_10          = u16FromLe(SpeedyResponse, 59);
  currentStatus.CANin_11          = u16FromLe(SpeedyResponse, 61);
  currentStatus.CANin_12          = u16FromLe(SpeedyResponse, 63);
  currentStatus.CANin_13          = u16FromLe(SpeedyResponse, 65);
  currentStatus.CANin_14          = u16FromLe(SpeedyResponse, 67);
  currentStatus.CANin_15          = u16FromLe(SpeedyResponse, 69);
  currentStatus.CANin_16          = u16FromLe(SpeedyResponse, 71);
  currentStatus.tpsADC            = SpeedyResponse[73];
  currentStatus.getNextError      = SpeedyResponse[74];
  currentStatus.launchCorrection  = 0;  // not part of legacy A payload in this template
}

// -----------------------------------------------------------------------------
// INTERNAL SIGNAL NORMALIZATION
// Convert the raw packet mirror into clean internal signals.
// Future CAN builders and lamp outputs should use this layer.
// -----------------------------------------------------------------------------

void updateSignalsFromStatus() {
  clearSignals();

  liveSignals.rpm             = currentStatus.RPM;
  liveSignals.coolantC        = int16_t(currentStatus.CLT) - 40;
  liveSignals.intakeC         = int16_t(currentStatus.IAT) - 40;
  liveSignals.throttlePct     = (uint8_t)map(currentStatus.TPS, 0, 200, 0, 100);
  liveSignals.mapKpa          = currentStatus.MAP;
  liveSignals.batteryDeciVolts= currentStatus.battery10;
  liveSignals.afrRaw          = currentStatus.O2;
  liveSignals.afrTargetRaw    = currentStatus.afrTarget;
  liveSignals.ethanolPct      = currentStatus.ethanolPct;
  liveSignals.boostTargetRaw  = currentStatus.boostTarget;
  liveSignals.boostDutyPct    = currentStatus.boostDuty;
  liveSignals.sparkRaw        = currentStatus.spark;
  liveSignals.auxIn1          = currentStatus.CANin_1;
  liveSignals.auxIn2          = currentStatus.CANin_2;
  liveSignals.status1Raw      = currentStatus.status1;
  liveSignals.engineRaw       = currentStatus.engine;
  liveSignals.testOutputsRaw  = currentStatus.testOutputs;
  liveSignals.errorCodeRaw    = currentStatus.getNextError;

  // Generic first-pass logic.
  // CEL itself is resolved in updateCelLogic().
  liveSignals.overtempActive  = (liveSignals.coolantC >= celConfig.overtempThresholdC);
  liveSignals.celActive       = false;
  liveSignals.launchActive    = (currentStatus.launchCorrection > 0);
  liveSignals.valid           = true;
}

void updateCelLogic() {
  celState.commFaultActive = false;
  celState.errorCodeActive = false;
  celState.overtempActive  = false;
  celState.overboostActive = false;
  celState.oilPressureActive = false;
  celState.leanFaultActive = false;
  celState.richFaultActive = false;
  celState.batteryFaultActive = false;
  celState.launchCelActive = false;
  celState.finalCelActive  = false;
  celState.reasonBits      = 0;
  celState.outputPattern   = PATTERN_SOLID;

  if (celConfig.enableCommsFault && !liveSignals.valid) {
    celState.commFaultActive = true;
  }

  if (!liveSignals.valid) {
    celState.overboostStartMs = 0;
    celState.oilPressureStartMs = 0;
    celState.leanFaultStartMs = 0;
    celState.richFaultStartMs = 0;
    celState.batteryFaultStartMs = 0;
  } else {
    if (celConfig.enableErrorCode && (liveSignals.errorCodeRaw != 0)) {
      celState.errorCodeActive = true;
    }

    if (celConfig.enableOvertemp && liveSignals.overtempActive) {
      celState.overtempActive = true;
    }

    const bool overboostCondition = celConfig.enableOverboost &&
                                    (liveSignals.rpm >= celConfig.overboostMinRpm) &&
                                    (liveSignals.throttlePct >= celConfig.overboostMinThrottlePct) &&
                                    (liveSignals.mapKpa >= celConfig.overboostThresholdKpa);

    celState.overboostActive = heldForMs(overboostCondition,
                                         celState.overboostStartMs,
                                         celConfig.overboostDelayMs);

    const uint16_t oilPressureRaw = getAuxSourceRaw(celConfig.oilPressureSource);
    const bool oilPressureCondition = celConfig.enableOilPressureFault &&
                                      (celConfig.oilPressureSource != AUX_SOURCE_NONE) &&
                                      (liveSignals.rpm >= celConfig.oilPressureMinRpm) &&
                                      (oilPressureRaw <= celConfig.oilPressureMinRaw);

    celState.oilPressureActive = heldForMs(oilPressureCondition,
                                           celState.oilPressureStartMs,
                                           celConfig.oilPressureDelayMs);

    const int afrDeviation = int(liveSignals.afrRaw) - int(liveSignals.afrTargetRaw);
    const bool leanCondition = celConfig.enableLeanFault &&
                               (liveSignals.rpm >= celConfig.leanMinRpm) &&
                               (liveSignals.throttlePct >= celConfig.leanMinThrottlePct) &&
                               (afrDeviation >= int(celConfig.leanDeviationThresholdRaw));

    const bool richCondition = celConfig.enableRichFault &&
                               (liveSignals.rpm >= celConfig.richMinRpm) &&
                               (liveSignals.throttlePct >= celConfig.richMinThrottlePct) &&
                               ((-afrDeviation) >= int(celConfig.richDeviationThresholdRaw));

    celState.leanFaultActive = heldForMs(leanCondition,
                                         celState.leanFaultStartMs,
                                         celConfig.leanDelayMs);

    celState.richFaultActive = heldForMs(richCondition,
                                         celState.richFaultStartMs,
                                         celConfig.richDelayMs);

    const bool batteryCondition = celConfig.enableBatteryFault &&
                                  ((millis() - startupTimeMs) >= celConfig.batteryStartupIgnoreMs) &&
                                  ((liveSignals.batteryDeciVolts <= celConfig.undervoltThresholdDeciV) ||
                                   (liveSignals.batteryDeciVolts >= celConfig.overvoltThresholdDeciV));

    celState.batteryFaultActive = heldForMs(batteryCondition,
                                            celState.batteryFaultStartMs,
                                            celConfig.batteryDelayMs);

    if (celConfig.enableLaunchOnCel && liveSignals.launchActive) {
      celState.launchCelActive = true;
    }
  }

  if (celState.commFaultActive) {
    celState.reasonBits |= CEL_REASON_COMMS_FAULT;
  }
  if (celState.errorCodeActive) {
    celState.reasonBits |= CEL_REASON_ERROR_CODE;
  }
  if (celState.overtempActive) {
    celState.reasonBits |= CEL_REASON_OVERTEMP;
  }
  if (celState.overboostActive) {
    celState.reasonBits |= CEL_REASON_OVERBOOST;
  }
  if (celState.oilPressureActive) {
    celState.reasonBits |= CEL_REASON_OIL_PRESS;
  }
  if (celState.leanFaultActive) {
    celState.reasonBits |= CEL_REASON_LEAN;
  }
  if (celState.richFaultActive) {
    celState.reasonBits |= CEL_REASON_RICH;
  }
  if (celState.batteryFaultActive) {
    celState.reasonBits |= CEL_REASON_BATTERY;
  }
  if (celState.launchCelActive) {
    celState.reasonBits |= CEL_REASON_LAUNCH;
  }

  celState.finalCelActive = (celState.reasonBits != 0);

  if (celState.oilPressureActive) {
    celState.outputPattern = PATTERN_FLASH_FAST;
  } else if (celState.overtempActive) {
    celState.outputPattern = PATTERN_FLASH_3HZ;
  } else if (celState.launchCelActive) {
    celState.outputPattern = PATTERN_FLASH_1HZ;
  } else {
    celState.outputPattern = PATTERN_SOLID;
  }

  liveSignals.celActive = celState.finalCelActive;
}

void updateWarningLogic() {
  clearWarningState();
  updateCelLogic();

  warningState.active[WARN_SRC_CEL]          = liveSignals.celActive;
  warningState.active[WARN_SRC_OIL_PRESSURE] = celState.oilPressureActive;
  warningState.active[WARN_SRC_LAUNCH]       = liveSignals.launchActive;
  warningState.active[WARN_SRC_FAN_ACTIVE]   = false;  // Placeholder until fan status is defined
  warningState.active[WARN_SRC_OPTION_5]     = false;
  warningState.active[WARN_SRC_OPTION_6]     = false;
  warningState.active[WARN_SRC_OPTION_7]     = false;
  warningState.active[WARN_SRC_OPTION_8]     = false;

  warningState.lampTestActive = startupLampTestActive();
}

// -----------------------------------------------------------------------------
// FIXED PACKET VALIDATION + SIGNAL UPDATE
// This is part of the core data-ingest path and should stay stable.
// -----------------------------------------------------------------------------

void processData() {
  data_error = false;

  decodeLegacyAPacket();

  // Basic sanity checks for live data.
  if (currentStatus.RPM >= 8000) {
    data_error = true;
    Serial.print("Error. RPM received: ");
    Serial.println(currentStatus.RPM);
  }

  if (currentStatus.CLT >= 182) {
    data_error = true;
    Serial.print("Error. CLT received: ");
    Serial.println(currentStatus.CLT);
  }

  if (currentStatus.TPS >= 201) {
    data_error = true;
    Serial.print("Error. TPS received: ");
    Serial.println(currentStatus.TPS);
  }

  if (data_error) {
    invalidateLiveData();
    return;
  }

  updateSignalsFromStatus();
  updateWarningLogic();
  newData = true;
}

// -----------------------------------------------------------------------------
// FIXED LEGACY A-PACKET RECEIVE
// Read one complete legacy A-message payload (75 bytes after the leading 'A' header).
// -----------------------------------------------------------------------------

void handleA() {
  data_error = false;

  for (uint8_t i = 0; i < SPEEDY_A_PAYLOAD_LEN; i++) {
    int incoming = SpeeduinoPort.read();
    if (incoming < 0) {
      data_error = true;
      break;
    }
    SpeedyResponse[i] = (uint8_t)incoming;
  }

  if (!data_error) {
    processData();
  } else {
    invalidateLiveData();
  }

  doRequest = true;
  oldtime = millis();
  SerialState = NOTHING_RECEIVED;
}

// -----------------------------------------------------------------------------
// FIXED SPEEDUINO HEADER RECEIVE
// Read the message type coming back from Speeduino.
// -----------------------------------------------------------------------------

void readSerialHeader() {
  currentCommand = SpeeduinoPort.read();

  switch (currentCommand) {
    case 'A':
      SerialState = A_MESSAGE;
      break;

    default:
      Serial.print("Unexpected serial header: ");
      Serial.println((char)currentCommand);
      SerialState = NOTHING_RECEIVED;
      break;
  }
}

// -----------------------------------------------------------------------------
// QUICK ADAPTATION GUIDE FOR USERS
//
// To adapt this template for your own cluster / dash / CAN device:
//   1) Change the CAN IDs in the USER-EDITABLE CAN IDS section if needed.
//   2) Edit buildFastEngineFrame() and buildFastStatusFrame() for high-priority data.
//   3) Edit buildMediumSensorFrame() and buildSlowAuxFrame() for slower signals.
//   4) Leave the fixed board-support and Speeduino decode sections alone unless
//      you are intentionally changing protocol support or board hardware.
//   5) Keep liveSignals as the common source of truth for all outgoing data.
//
// Tip: Treat liveSignals as the common source of truth. Map from liveSignals into
// the outgoing CAN bytes you need, rather than reading directly from currentStatus.
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// USER-EDITABLE OUTPUT BUILDERS
// Keep these separate so SPEEDY-CAN can grow cleanly into:
//   1) cluster CAN
//   2) aux / gauge CAN
//   3) physical lamp outputs
//
// This is the main editable logic area for end users.
// Change payload layout, scaling, and meaning here to suit the target vehicle.
// -----------------------------------------------------------------------------

void buildFastEngineFrame() {
  memset(canFastEngine.data, 0, sizeof(canFastEngine.data));

  if (!liveSignals.valid) {
    return;
  }

  // SPEEDY-CAN 0x300 Fast Engine Frame @ FastCanUpdateRate
  // Byte 0-1: RPM (uint16 LE)
  // Byte 2-3: Vehicle speed, 0.1 MPH default (uint16 LE) - placeholder 0 until VSS is defined
  // Byte 4-5: MAP absolute kPa (uint16 LE)
  // Byte 6: AFR raw from Speeduino
  // Byte 7: AFR target raw from Speeduino
  packU16Le(canFastEngine, 0, liveSignals.rpm);
  packU16Le(canFastEngine, 2, 0);  // Placeholder until VSS / speed signal is defined
  packU16Le(canFastEngine, 4, liveSignals.mapKpa);
  canFastEngine.data[6] = liveSignals.afrRaw;
  canFastEngine.data[7] = liveSignals.afrTargetRaw;
}

void buildFastStatusFrame() {
  memset(canFastStatus.data, 0, sizeof(canFastStatus.data));

  // SPEEDY-CAN 0x301 Fast Status Frame @ FastCanUpdateRate
  // Byte 0: Warning indicator bitfield (route/pattern driven)
  // Byte 1: Live data valid flag
  // Byte 2: Raw error code
  // Byte 3: CEL pattern enum
  // Byte 4-5: CEL reason bitfield (uint16 LE)
  // Byte 6: Speeduino status1 raw
  // Byte 7: Speeduino engine raw
  canFastStatus.data[0] = 0x00;
  canFastStatus.data[1] = liveSignals.valid ? 1 : 0;
  canFastStatus.data[2] = liveSignals.errorCodeRaw;
  canFastStatus.data[3] = uint8_t(celState.outputPattern);
  packU16Le(canFastStatus, 4, celState.reasonBits);
  canFastStatus.data[6] = liveSignals.status1Raw;
  canFastStatus.data[7] = liveSignals.engineRaw;

  for (uint8_t warningId = 0; warningId < WARN_SRC_COUNT; warningId++) {
    if (!warningRoutes[warningId].enabled) {
      continue;
    }

    const bool outputActive = warningState.lampTestActive || warningState.active[warningId];
    const IndicatorPattern pattern = (warningId == WARN_SRC_CEL) ? celState.outputPattern
                                                                 : warningRoutes[warningId].pattern;
    writeCanBit(canFastStatus,
                warningRoutes[warningId].canByteIndex,
                warningRoutes[warningId].canBitIndex,
                outputActive && patternOutputActive(pattern));
  }
}

void buildMediumSensorFrame() {
  memset(canMediumSensors.data, 0, sizeof(canMediumSensors.data));

  if (!liveSignals.valid) {
    return;
  }

  const uint16_t oilPressureRaw = getAuxSourceRaw(celConfig.oilPressureSource);

  // SPEEDY-CAN 0x302 Medium Sensor Frame @ MediumCanUpdateRate
  // Byte 0: Coolant temp, C offset +40
  // Byte 1: Intake air temp, C offset +40
  // Byte 2: Battery voltage, deci-volts
  // Byte 3: Oil pressure warning active
  // Byte 4-5: Oil pressure raw / scaled source value (uint16 LE)
  // Byte 6-7: Reserved for future sensor expansion
  canMediumSensors.data[0] = tempCToOffsetByte(liveSignals.coolantC);
  canMediumSensors.data[1] = tempCToOffsetByte(liveSignals.intakeC);
  canMediumSensors.data[2] = liveSignals.batteryDeciVolts;
  canMediumSensors.data[3] = celState.oilPressureActive ? 1 : 0;
  packU16Le(canMediumSensors, 4, oilPressureRaw);
}

void buildSlowAuxFrame() {
  memset(canSlowAux.data, 0, sizeof(canSlowAux.data));

  if (!liveSignals.valid) {
    return;
  }

  // SPEEDY-CAN 0x303 Slow Aux Frame @ SlowCanUpdateRate
  // Byte 0: Throttle percent
  // Byte 1: Ethanol percent
  // Byte 2: Boost target raw
  // Byte 3: Boost duty raw
  // Byte 4-5: Aux input 1 raw (uint16 LE)
  // Byte 6-7: Aux input 2 raw (uint16 LE)
  canSlowAux.data[0] = liveSignals.throttlePct;
  canSlowAux.data[1] = liveSignals.ethanolPct;
  canSlowAux.data[2] = liveSignals.boostTargetRaw;
  canSlowAux.data[3] = liveSignals.boostDutyPct;
  packU16Le(canSlowAux, 4, liveSignals.auxIn1);
  packU16Le(canSlowAux, 6, liveSignals.auxIn2);
}

void updateLampOutputs() {
  for (uint8_t warningId = 0; warningId < WARN_SRC_COUNT; warningId++) {
    if (!warningRoutes[warningId].enabled) {
      continue;
    }

    const uint8_t lampIndex = warningRoutes[warningId].lampIndex;
    const bool outputActive = warningState.lampTestActive || warningState.active[warningId];
    const IndicatorPattern pattern = (warningId == WARN_SRC_CEL) ? celState.outputPattern
                                                                 : warningRoutes[warningId].pattern;
    const bool lampOn = outputActive && patternOutputActive(pattern);

    if (lampIndex < 8) {
      digitalWrite(lampPins[lampIndex], lampOn ? HIGH : LOW);
    }
  }
}

void sendFastCanFrames() {
  buildFastEngineFrame();
  buildFastStatusFrame();

  mcp2515.sendMessage(&canFastEngine);
  mcp2515.sendMessage(&canFastStatus);
}

void sendMediumCanFrame() {
  buildMediumSensorFrame();
  mcp2515.sendMessage(&canMediumSensors);
}

void sendSlowCanFrame() {
  buildSlowAuxFrame();
  mcp2515.sendMessage(&canSlowAux);
}

void updateOutputs() {
  updateLampOutputs();

  // Toggle built-in LED so activity is visible during bring-up.
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

// -----------------------------------------------------------------------------
// FIXED SPEEDY-CAN MCP2515 INITIALIZATION
// Normal users should not need to edit this section.
// -----------------------------------------------------------------------------

void initCanController() {
  SPI.begin(MCP2515_SCK_PIN, MCP2515_MISO_PIN, MCP2515_MOSI_PIN, MCP2515_CS_PIN);
  mcp2515.reset();
  mcp2515.setBitrate(MCP2515_BITRATE, MCP2515_CLOCK);
  mcp2515.setNormalMode();
}

// -----------------------------------------------------------------------------
// FIXED SETUP
// Board initialization and fixed interface bring-up.
// -----------------------------------------------------------------------------

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(MCP2515_INT_PIN, INPUT);
  pinMode(MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MCP2515_CS_PIN, HIGH);

  for (uint8_t i = 0; i < 8; i++) {
    pinMode(lampPins[i], OUTPUT);
    digitalWrite(lampPins[i], LOW);
  }

  Serial.begin(115200);   // Debug
  SpeeduinoPort.begin(115200, SERIAL_8N1, SPEEDUINO_RX_PIN, SPEEDUINO_TX_PIN);

  initState();
  initCanController();

  // INT is active-LOW. Use a FALLING interrupt if/when interrupt-driven CAN is added.
}

// -----------------------------------------------------------------------------
// FIXED MAIN LOOP
// Core scheduler for Speeduino polling, receive handling, CAN transmit, and timeout handling.
// -----------------------------------------------------------------------------

void loop() {
  // Request Speeduino data at the chosen rate.
  if (millis() - lastSerialRequestMs >= (1000UL / SerialUpdateRate)) {
    requestData();
  }

  // Serial receive state machine.
  switch (SerialState) {
    case NOTHING_RECEIVED:
      if (SpeeduinoPort.available() > 0) {
        readSerialHeader();
      }
      break;

    case A_MESSAGE:
      if (SpeeduinoPort.available() >= SPEEDY_A_PAYLOAD_LEN) {
        handleA();
      }
      break;

    default:
      SerialState = NOTHING_RECEIVED;
      break;
  }

  // Publish CAN on separate schedules based on signal importance.
  if (millis() - lastFastCanMs >= (1000UL / FastCanUpdateRate)) {
    lastFastCanMs = millis();
    sendFastCanFrames();
    updateOutputs();
  }

  if (millis() - lastMediumCanMs >= (1000UL / MediumCanUpdateRate)) {
    lastMediumCanMs = millis();
    sendMediumCanFrame();
  }

  if (millis() - lastSlowCanMs >= (1000UL / SlowCanUpdateRate)) {
    lastSlowCanMs = millis();
    sendSlowCanFrame();
  }

  // Serial timeout handling.
  if ((millis() - oldtime) > SPEEDY_TIMEOUT_MS) {
    Serial.println("Timeout from Speeduino");
    oldtime = millis();
    doRequest = true;
    invalidateLiveData();
  }
}
```
