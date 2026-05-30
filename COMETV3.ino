#include <Arduino.h>
#include <Wire.h>
#include <LittleFS.h>

#include <Adafruit_MPL3115A2.h>
#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_Sensor.h>

#include <math.h>

// ============================================================================
// COMET Custom RP2040 Flight Computer
// Compact Onboard Management for Ejection Timing
// Serial RFD900x Telemetry + LittleFS Binary Flight Logger
//
// Hardware:
//   - RP2040 custom board
//   - MPL3115A2 barometer
//   - LSM6DSO32 IMU
//   - Main MOSFET on GPIO 2
//   - Drogue MOSFET on GPIO 3
//   - Buzzer on GPIO 6
//   - Mode switch on GPIO 21, external 10k pullup, pressed = LOW
//   - RGB LED on GPIO 22 / 24 / 25
//   - Battery divider on A2
//
// Logger:
//   - Uses LittleFS on the same 128 Mbit flash chip the RP2040 boots from
//   - 3 safe binary flight slots
//   - 10 Hz fixed logging
//   - 1 hour maximum per flight
//   - Does not overwrite COMPLETE slots unless marked DOWNLOADED or erased
//
// IMPORTANT:
//   Use Earle Philhower Arduino-Pico core.
//   Choose a board/filesystem layout with enough LittleFS space.
// ============================================================================

// ============================================================================
// Safety / Testing
// ============================================================================

constexpr bool PYRO_OUTPUTS_ENABLED = true;  // Keep false for bench testing
constexpr bool TEST_MODE = true;              // Enables USB serial commands

// ============================================================================
// Custom Board Pins
// ============================================================================

constexpr uint8_t MAIN_PIN         = 2;
constexpr uint8_t DROGUE_PIN       = 3;
constexpr uint8_t BUZZER_PIN       = 6;
constexpr uint8_t MODE_SWITCH_PIN  = 21;

constexpr uint8_t RGB_RED_PIN      = 22;
constexpr uint8_t RGB_GREEN_PIN    = 24;
constexpr uint8_t RGB_BLUE_PIN     = 25;

constexpr uint8_t BATTERY_PIN      = A2;

// ============================================================================
// RFD900x UART
// ============================================================================
// RFD900x RX connects to RP2040 TX.
// RFD900x TX connects to RP2040 RX.

constexpr uint8_t RFD_TX_PIN = 0;
constexpr uint8_t RFD_RX_PIN = 1;
constexpr uint32_t RFD_BAUD  = 57600;

// ============================================================================
// I2C / Sensors
// ============================================================================

constexpr uint8_t IMU_ADDR = 0x6B;

Adafruit_MPL3115A2 mpl;
Adafruit_LSM6DSO32 imu;

bool mplOK = false;
bool imuOK = false;

// ============================================================================
// Buzzer
// ============================================================================

constexpr uint16_t BUZZER_FREQ_HZ = 4750;

// Tuned COMET sound frequencies.
// 4750 Hz tested loud/clear on the current buzzer without being as painful as 6000 Hz.
constexpr uint16_t BUZZER_NICE_LOW_HZ   = 4000;
constexpr uint16_t BUZZER_NICE_MID_HZ   = 4300;
constexpr uint16_t BUZZER_NICE_HIGH_HZ  = 4550;
constexpr uint16_t BUZZER_ALERT_HZ      = 4750;
constexpr uint16_t BUZZER_FAILURE_HZ    = 6000;

// ============================================================================
// Battery Divider
// ============================================================================

constexpr float ADC_REF_VOLTAGE = 3.3f;
constexpr float R_TOP = 330000.0f;
constexpr float R_BOTTOM = 100000.0f;
constexpr float BATTERY_DIVIDER_RATIO = ((R_TOP + R_BOTTOM) / R_BOTTOM);

// ============================================================================
// Flight Parameters
// ============================================================================
// Runtime-editable parameters are lower-case variables so the Python GUI can
// update them with SET <PARAM> <VALUE>. They reset to these defaults on reboot.

float     mainAltM                 = 200.0f;
float     mainArmMarginM           = 20.0f;

constexpr uint32_t  POWER_STAB_MS  = 2000;
uint32_t  flightLockoutMs          = 10000;
uint32_t  drogueBackupMs           = 15000;
uint32_t  mainBackupMs             = 25000;

constexpr float     ASCENT_POS_VZ_THRESH       = 2.0f;
constexpr uint32_t  ASCENT_POS_VZ_DWELL_MS     = 250;

constexpr float     APOGEE_NEG_VZ_THRESH       = -1.5f;
constexpr uint32_t  APOGEE_NEG_VZ_DWELL_MS     = 250;

constexpr float     LAUNCH_ACCEL_DELTA_G       = 1.5f;
constexpr uint32_t  LAUNCH_ACCEL_DWELL_MS      = 150;
constexpr uint32_t  BASELINE_MIN_MS            = 1000;

constexpr float     MAX_FALL_SPEED_FOR_DROGUE  = -100.0f; // this is bad should make this much higher closer to 100
constexpr uint32_t  DROGUE_INEFFECTIVE_MS      = 1000; // this should also be increased to about a second 

// ============================================================================
// Pyro Timing
// ============================================================================

constexpr uint16_t PYRO_PULSE_MS = 300; // should confirm that this is a small enough dwell to not casue perm damg

// ============================================================================
// Telemetry
// ============================================================================

constexpr uint32_t TELEMETRY_PERIOD_MS = 100;  // 10 Hz

// ============================================================================
// Sensor Scheduling
// ============================================================================
// The old loop read every sensor every pass. The MPL3115A2 and battery averaging
// can block long enough to slow telemetry well below 10 Hz. These timers keep
// the IMU, barometer, battery, logger, and telemetry on separate schedules.

constexpr uint32_t IMU_READ_PERIOD_MS     = 10;    // 100 Hz target
constexpr uint32_t BARO_READ_PERIOD_MS    = 100;   // 10 Hz target if this can be increased that would be cool
constexpr uint32_t BATTERY_READ_PERIOD_MS = 1000;  // 1 Hz target

uint32_t lastImuReadMs = 0;
uint32_t lastBaroReadMs = 0;
uint32_t lastBatteryReadMs = 0;

// ============================================================================
// Flight State Machine
// ============================================================================

enum class FlightState : uint8_t {
  BOOT,
  IDLE,
  ASCENT_LOCKOUT,
  ASCENT,
  APOGEE_DETECT,
  DROGUE_DEPLOYED,
  MAIN_DEPLOYED,
  LANDED,
  FAULT
};

FlightState state = FlightState::BOOT;

// ============================================================================
// Human-Readable Logged Events
// ============================================================================
// Event codes are packed into the upper byte of the existing uint16_t
// eventFlags field. This keeps FlightLogRecord at 32 bytes while DUMPCSV can
// append an easy-to-read event_note column.

enum LogEventCode : uint8_t {
  LOG_EVENT_NONE = 0,
  LOG_EVENT_BOOT_READY = 1,
  LOG_EVENT_RESET = 2,
  LOG_EVENT_LAUNCH_ACCEL = 3,
  LOG_EVENT_FORCED_LAUNCH = 4,
  LOG_EVENT_ASCENT_LOCKOUT_END = 5,
  LOG_EVENT_APOGEE_NEG_TREND = 6,
  LOG_EVENT_DROGUE_APOGEE = 7,
  LOG_EVENT_DROGUE_TIMER_BACKUP = 8,
  LOG_EVENT_MAIN_ARMED_ALTITUDE = 9,
  LOG_EVENT_MAIN_ALTITUDE_CROSS = 10,
  LOG_EVENT_MAIN_TIMER_BACKUP = 11,
  LOG_EVENT_MAIN_DROGUE_INEFFECTIVE = 12,
  LOG_EVENT_SERIAL_TEST_DROGUE = 13,
  LOG_EVENT_SERIAL_TEST_MAIN = 14,
  LOG_EVENT_UNKNOWN = 255
};

uint8_t pendingLogEventCode = LOG_EVENT_NONE;

// ============================================================================
// Runtime Variables
// ============================================================================

uint32_t t_boot = 0;
uint32_t t_launch = 0;
uint32_t t_drogue_fire = 0;
uint32_t t_main_fire = 0;

bool drogueFired = false;
bool mainFired = false;
bool mainArmed = false;

float altRef_m = 0.0f;
float alt_m = 0.0f;
float launchAlt_m = 0.0f;
float altAGL_m = 0.0f;
float maxAltAGL_m = 0.0f;
float vz_mps = 0.0f;

float ax_mps2 = 0.0f;
float ay_mps2 = 0.0f;
float az_mps2 = 0.0f;

float gx_dps = 0.0f;
float gy_dps = 0.0f;
float gz_dps = 0.0f;

float imuTemp_C = 0.0f;
float mplTemp_C = 0.0f;
float pressure_Pa = 0.0f;
float batt_V = 0.0f;

float accelMag_g = 0.0f;
float accelMagBaseline_g = 1.0f;
bool accelBaselineValid = false;

uint32_t baselineStartMs = 0;
uint32_t launchAccelStartMs = 0;

// ============================================================================
// RGB Mode
// ============================================================================

int rgbMode = 0;

// ============================================================================
// Vz Ring Buffer
// ============================================================================

const int VZ_WIN = 8;

struct AltSample {
  uint32_t t;
  float alt;
};

AltSample ring[VZ_WIN];
int ringHead = 0;
int ringCount = 0;

// ============================================================================
// Non-blocking Buzzer
// ============================================================================

enum class BuzzerMode : uint8_t {
  IDLE,
  RAPID,
  TONE
};

BuzzerMode buzMode = BuzzerMode::IDLE;
bool buzState = false;
uint32_t buzNextToggle = 0;
uint32_t buzStopAt = 0;
uint16_t buzFreqHz = BUZZER_FREQ_HZ;

constexpr uint16_t BEEP_ON_MS  = 50;
constexpr uint16_t BEEP_OFF_MS = 50;

// ============================================================================
// Non-blocking Pyro
// ============================================================================

enum class PyroKind : uint8_t {
  NONE,
  DROGUE,
  MAIN
};

struct PyroSequencer {
  PyroKind kind = PyroKind::NONE;
  uint8_t pin = 255;
  bool active = false;
  uint32_t t0 = 0;
};

PyroSequencer pyro;

// ============================================================================
// Logger Config
// ============================================================================

constexpr bool     LOGGING_ENABLED        = true;
constexpr uint32_t LOG_RATE_HZ            = 10;
constexpr uint32_t LOG_PERIOD_MS          = 1000UL / LOG_RATE_HZ;
constexpr uint32_t LOG_DURATION_SECONDS   = 3600UL;
constexpr uint32_t LOG_MAX_RECORDS        = LOG_DURATION_SECONDS * LOG_RATE_HZ;
// Do NOT rewrite the LittleFS header during flight at a high rate.
// Closing/flushing/reopening the LittleFS file every few seconds can cause
// multi-second stalls while flash sectors are erased/reclaimed.
// We only flush data occasionally and write the final header when STOPLOG,
// one-hour limit, or another clean finish occurs.
constexpr uint32_t LOG_DATA_FLUSH_RECORDS = LOG_RATE_HZ * 60;  // flush data every 60 s

constexpr uint8_t  LOG_SLOT_COUNT         = 3;
constexpr uint32_t LOG_MAGIC              = 0x41555241;   // "AURA"
constexpr uint16_t LOG_VERSION            = 1;

const char* LOG_SLOT_FILES[LOG_SLOT_COUNT] = {
  "/flight0.bin",
  "/flight1.bin",
  "/flight2.bin"
};

// ============================================================================
// Persistent COMET Settings
// ============================================================================
// These settings live in LittleFS next to the flight logs. They are only
// rewritten when the user changes the selected altitude mode, not continuously
// during flight.

const char* COMET_SETTINGS_FILE = "/comet_settings.bin";
constexpr uint32_t SETTINGS_MAGIC   = 0x434D4554UL;  // "CMET"
constexpr uint16_t SETTINGS_VERSION = 1;

struct __attribute__((packed)) CometSettings {
  uint32_t magic;
  uint16_t version;
  uint8_t  mainAltMode;
  uint8_t  reserved;
  uint32_t crc;
};

// ============================================================================
// Logger Slot Status
// ============================================================================

enum SlotStatus : uint8_t {
  SLOT_EMPTY      = 0,
  SLOT_RECORDING  = 1,
  SLOT_COMPLETE   = 2,
  SLOT_DOWNLOADED = 3,
  SLOT_BAD        = 4
};

// ============================================================================
// Packed Binary Log Header
// ============================================================================

struct __attribute__((packed)) FlightLogHeader {
  uint32_t magic;
  uint16_t version;
  uint8_t  slotIndex;
  uint8_t  status;

  uint32_t flightNumber;
  uint32_t startMillis;
  uint32_t completedMillis;

  uint32_t recordSize;
  uint32_t maxRecords;
  uint32_t recordCount;

  uint32_t headerSize;
  uint32_t headerCrc;

  uint8_t  downloaded;
  uint8_t  reserved[23];
};

// ============================================================================
// Packed Binary Log Record
// Exactly 32 bytes.
// ============================================================================

struct __attribute__((packed)) FlightLogRecord {
  uint32_t t_ms;

  int32_t  alt_cm;
  int16_t  vz_cms;

  int16_t  ax_cg;
  int16_t  ay_cg;
  int16_t  az_cg;

  int16_t  gx_cdps;
  int16_t  gy_cdps;
  int16_t  gz_cdps;

  uint16_t batt_mv;
  int16_t  temp_cC;

  uint8_t  state;
  uint8_t  flags;

  uint16_t eventFlags;
  uint16_t crc;
};

static_assert(sizeof(FlightLogRecord) == 32, "FlightLogRecord must be 32 bytes");

// ============================================================================
// Logger Runtime
// ============================================================================

File logFile;

FlightLogHeader activeHeader;
bool loggerMounted = false;
bool loggerActive = false;
bool loggerFull = false;
bool downloadMode = false;

int8_t activeSlot = -1;
uint32_t lastLogMs = 0;
uint32_t nextFlightNumber = 1;

// Logger-full user interface
constexpr uint32_t FULL_SLOT_HOLD_CLEAR_MS = 5000;
constexpr uint32_t FULL_YELLOW_BLINK_MS    = 250;
uint32_t fullButtonHoldStartMs = 0;
uint32_t fullBlinkLastMs = 0;
bool fullBlinkOn = false;

// ============================================================================
// Utility Helpers
// ============================================================================

static inline bool finitef_safe(float v) {
  return !(isnan(v) || isinf(v));
}

static inline float safe0(float v) {
  return finitef_safe(v) ? v : 0.0f;
}

static inline float gFromMps2(float a) {
  return a / 9.80665f;
}

static inline float radToDps(float r) {
  return r * 180.0f / PI;
}

int16_t clampI16(long v) {
  if (v > 32767L) return 32767;
  if (v < -32768L) return -32768;
  return (int16_t)v;
}

uint16_t clampU16(long v) {
  if (v > 65535L) return 65535;
  if (v < 0L) return 0;
  return (uint16_t)v;
}

const char* stateStr(FlightState s) {
  switch (s) {
    case FlightState::BOOT:            return "BOOT";
    case FlightState::IDLE:            return "IDLE";
    case FlightState::ASCENT_LOCKOUT:  return "ASCENT_LOCKOUT";
    case FlightState::ASCENT:          return "ASCENT";
    case FlightState::APOGEE_DETECT:   return "APOGEE_DETECT";
    case FlightState::DROGUE_DEPLOYED: return "DROGUE_DEPLOYED";
    case FlightState::MAIN_DEPLOYED:   return "MAIN_DEPLOYED";
    case FlightState::LANDED:          return "LANDED";
    case FlightState::FAULT:           return "FAULT";
  }

  return "UNKNOWN";
}

uint8_t logEventCodeFromName(const char* eventName) {
  if (!eventName || eventName[0] == '\0' || strcmp(eventName, "-") == 0) return LOG_EVENT_NONE;

  if (strcmp(eventName, "BOOT_READY") == 0) return LOG_EVENT_BOOT_READY;
  if (strcmp(eventName, "RESET") == 0) return LOG_EVENT_RESET;
  if (strcmp(eventName, "LAUNCH_ACCEL") == 0) return LOG_EVENT_LAUNCH_ACCEL;
  if (strcmp(eventName, "FORCED_LAUNCH") == 0) return LOG_EVENT_FORCED_LAUNCH;
  if (strcmp(eventName, "ASCENT_LOCKOUT_END") == 0) return LOG_EVENT_ASCENT_LOCKOUT_END;
  if (strcmp(eventName, "APOGEE_NEG_TREND") == 0) return LOG_EVENT_APOGEE_NEG_TREND;
  if (strcmp(eventName, "APOGEE") == 0) return LOG_EVENT_DROGUE_APOGEE;
  if (strcmp(eventName, "TIMER_BACKUP_DROGUE") == 0) return LOG_EVENT_DROGUE_TIMER_BACKUP;
  if (strcmp(eventName, "MAIN_ARMED_ALTITUDE") == 0) return LOG_EVENT_MAIN_ARMED_ALTITUDE;
  if (strcmp(eventName, "MAIN_AGL_CROSS") == 0) return LOG_EVENT_MAIN_ALTITUDE_CROSS;
  if (strcmp(eventName, "TIMER_BACKUP_MAIN_AFTER_DROGUE") == 0) return LOG_EVENT_MAIN_TIMER_BACKUP;
  if (strcmp(eventName, "DROGUE_INEFFECTIVE_FALLRATE") == 0) return LOG_EVENT_MAIN_DROGUE_INEFFECTIVE;
  if (strcmp(eventName, "SERIAL_TEST_DROGUE") == 0) return LOG_EVENT_SERIAL_TEST_DROGUE;
  if (strcmp(eventName, "SERIAL_TEST_MAIN") == 0) return LOG_EVENT_SERIAL_TEST_MAIN;

  return LOG_EVENT_UNKNOWN;
}

const char* logEventNote(uint8_t code) {
  switch (code) {
    case LOG_EVENT_NONE:                    return "";
    case LOG_EVENT_BOOT_READY:              return "BOOT ready";
    case LOG_EVENT_RESET:                   return "RESET flight state to IDLE";
    case LOG_EVENT_LAUNCH_ACCEL:            return "LAUNCH detected by accelerometer threshold";
    case LOG_EVENT_FORCED_LAUNCH:           return "LAUNCH forced by serial command";
    case LOG_EVENT_ASCENT_LOCKOUT_END:      return "ASCENT lockout ended by timer";
    case LOG_EVENT_APOGEE_NEG_TREND:        return "APOGEE detected by negative vertical speed trend";
    case LOG_EVENT_DROGUE_APOGEE:           return "DROGUE fired after apogee detection";
    case LOG_EVENT_DROGUE_TIMER_BACKUP:     return "DROGUE fired by timer backup";
    case LOG_EVENT_MAIN_ARMED_ALTITUDE:     return "MAIN armed after exceeding selected altitude plus margin";
    case LOG_EVENT_MAIN_ALTITUDE_CROSS:     return "MAIN fired by altitude crossing";
    case LOG_EVENT_MAIN_TIMER_BACKUP:       return "MAIN fired by timer backup after drogue";
    case LOG_EVENT_MAIN_DROGUE_INEFFECTIVE: return "MAIN fired because drogue descent rate looked too fast";
    case LOG_EVENT_SERIAL_TEST_DROGUE:      return "DROGUE fired by serial test command";
    case LOG_EVENT_SERIAL_TEST_MAIN:        return "MAIN fired by serial test command";
    case LOG_EVENT_UNKNOWN:                 return "EVENT occurred but name was not recognized";
    default:                                return "";
  }
}

// Forward declarations for persistent settings.
bool saveCometSettings();
bool loadCometSettings();

// ============================================================================
// RGB / Main Altitude Mode / Buzzer
// ============================================================================
// The RGB LED on this board is active-low/common-anode.  The rest of the
// firmware uses normal logical colors, so setRGB(1, 1, 0) still means yellow.
// The MODE button cycles the main deployment altitude through ROYGBIV presets.

constexpr bool RGB_ACTIVE_LOW = true;

struct MainAltitudeMode {
  const char* name;
  float altitude_m;
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

// ROYGBIV main deployment altitude presets.
// MODE button cycles through these values.
const MainAltitudeMode MAIN_ALT_MODES[] = {
  {"RED",    800.0f, 255,   0,   0},
  {"ORANGE", 700.0f, 255,  70,   0},
  {"YELLOW", 600.0f, 255, 255,   0},
  {"GREEN",  500.0f,   0, 255,   0},
  {"BLUE",   400.0f,   0,   0, 255},
  {"INDIGO", 300.0f,  75,   0, 130},
  {"VIOLET", 200.0f, 160,   0, 255},
};

constexpr uint8_t MAIN_ALT_MODE_COUNT =
  sizeof(MAIN_ALT_MODES) / sizeof(MAIN_ALT_MODES[0]);

void setRGBLevel(uint8_t r, uint8_t g, uint8_t b) {
  if (RGB_ACTIVE_LOW) {
    analogWrite(RGB_RED_PIN,   255 - r);
    analogWrite(RGB_GREEN_PIN, 255 - g);
    analogWrite(RGB_BLUE_PIN,  255 - b);
  } else {
    analogWrite(RGB_RED_PIN,   r);
    analogWrite(RGB_GREEN_PIN, g);
    analogWrite(RGB_BLUE_PIN,  b);
  }
}

void setRGB(bool r, bool g, bool b) {
  setRGBLevel(r ? 255 : 0, g ? 255 : 0, b ? 255 : 0);
}

void setMainAltitudeMode(uint8_t mode, bool announce = true, bool saveToFs = false) {
  if (mode >= MAIN_ALT_MODE_COUNT) {
    mode = 0;
  }

  rgbMode = mode;

  const MainAltitudeMode& m = MAIN_ALT_MODES[rgbMode];
  mainAltM = m.altitude_m;
  setRGBLevel(m.r, m.g, m.b);

  if (announce) {
    Serial.print("MAIN ALT MODE: ");
    Serial.print(rgbMode);
    Serial.print(" ");
    Serial.print(m.name);
    Serial.print(" MAIN_ALT=");
    Serial.print(mainAltM, 0);
    Serial.println(" m");
  }

  if (saveToFs) {
    if (saveCometSettings()) {
      Serial.println("SETTINGS SAVED: main altitude mode");
    } else {
      Serial.println("SETTINGS SAVE FAILED: main altitude mode not persisted");
    }
  }
}

// Keep this function name because the rest of the sketch already calls it.
// It now applies the selected main-altitude preset color.
void updateRGBMode() {
  setMainAltitudeMode((uint8_t)rgbMode, false, false);
}

int findMainAltitudeMode(const String& value) {
  String v = value;
  v.trim();
  v.toUpperCase();

  for (uint8_t i = 0; i < MAIN_ALT_MODE_COUNT; i++) {
    if (v == MAIN_ALT_MODES[i].name) {
      return i;
    }
  }

  bool numeric = v.length() > 0;
  for (uint16_t i = 0; i < v.length(); i++) {
    if (!isDigit(v[i])) {
      numeric = false;
      break;
    }
  }

  if (numeric) {
    int mode = v.toInt();
    if (mode >= 0 && mode < MAIN_ALT_MODE_COUNT) {
      return mode;
    }
  }

  return -1;
}

void beepFreqBlocking(uint16_t freqHz, int onMs, int offMs = 0) {
  tone(BUZZER_PIN, freqHz);
  delay(onMs);
  noTone(BUZZER_PIN);

  if (offMs > 0) {
    delay(offMs);
  }
}

void beepBlocking(int times, int onMs = 100, int offMs = 100) {
  for (int i = 0; i < times; i++) {
    beepFreqBlocking(BUZZER_FREQ_HZ, onMs, offMs);
  }
}

void playStartupMelody() {
  // Selected "Smooth Rise" startup melody from the standalone test sketch.
  beepFreqBlocking(4000, 90, 35);
  beepFreqBlocking(4300, 90, 35);
  beepFreqBlocking(4550, 110, 35);
  beepFreqBlocking(4750, 180, 0);
}

void playSensorOkSound() {
  // Two clean beeps: sensors verified.
  beepFreqBlocking(BUZZER_ALERT_HZ, 110, 120);
  beepFreqBlocking(BUZZER_ALERT_HZ, 110, 0);
}

void playSensorFailureSound() {
  // Alternating failure alarm for about 5 seconds.
  const uint32_t durationMs = 5000;
  const uint16_t onMs = 140;
  const uint16_t offMs = 90;

  uint32_t startMs = millis();
  bool high = false;

  while (millis() - startMs < durationMs) {
    high = !high;
    beepFreqBlocking(high ? BUZZER_FAILURE_HZ : BUZZER_NICE_LOW_HZ, onMs, offMs);
  }

  noTone(BUZZER_PIN);
}

void startToneBeep(uint16_t freqHz, uint32_t durationMs) {
  uint32_t now = millis();

  buzMode = BuzzerMode::TONE;
  buzState = true;
  buzFreqHz = freqHz;
  buzStopAt = now + durationMs;

  tone(BUZZER_PIN, buzFreqHz);
}

void startRapidBeep(uint16_t seconds) {
  uint32_t now = millis();

  buzMode = BuzzerMode::RAPID;
  buzState = true;
  buzFreqHz = BUZZER_ALERT_HZ;

  tone(BUZZER_PIN, buzFreqHz);

  buzNextToggle = now + BEEP_ON_MS;
  buzStopAt = now + (uint32_t)seconds * 1000UL;
}

void startLaunchDetectedSound() {
  // Rapid constant 4750 Hz beeping for launch detect.
  startRapidBeep(3);
}

void startDrogueFiredSound() {
  // Long 4750 Hz pyro-confirmation tone.
  startToneBeep(BUZZER_ALERT_HZ, 850);
}

void startMainFiredSound() {
  // Slightly longer 4750 Hz pyro-confirmation tone.
  startToneBeep(BUZZER_ALERT_HZ, 1200);
}

void stopBeep() {
  buzMode = BuzzerMode::IDLE;
  buzState = false;
  noTone(BUZZER_PIN);
}

void serviceBuzzer(uint32_t now) {
  if (buzMode == BuzzerMode::IDLE) return;

  if ((int32_t)(now - buzStopAt) >= 0) {
    stopBeep();
    return;
  }

  if (buzMode == BuzzerMode::TONE) {
    // Continuous tone until buzStopAt.
    return;
  }

  if (buzMode == BuzzerMode::RAPID && (int32_t)(now - buzNextToggle) >= 0) {
    buzState = !buzState;

    if (buzState) {
      tone(BUZZER_PIN, buzFreqHz);
      buzNextToggle = now + BEEP_ON_MS;
    } else {
      noTone(BUZZER_PIN);
      buzNextToggle = now + BEEP_OFF_MS;
    }
  }
}

void handleModeSwitch() {
  // When the logger is full, the MODE button is reserved for clearing
  // the oldest saved flight slot. See serviceLoggerFullUI().
  if (loggerFull) return;

  static bool buttonWasPressed = false;
  static unsigned long lastPressTime = 0;

  bool pressed = digitalRead(MODE_SWITCH_PIN) == LOW;
  unsigned long now = millis();

  if (pressed && !buttonWasPressed && (now - lastPressTime > 150)) {
    buttonWasPressed = true;
    lastPressTime = now;

    rgbMode++;
    if (rgbMode >= MAIN_ALT_MODE_COUNT) {
      rgbMode = 0;
    }

    setMainAltitudeMode((uint8_t)rgbMode, true, true);

    // One short confirmation beep instead of a one-second rapid pattern.
    beepBlocking(1, 40, 20);
  }

  if (!pressed) {
    buttonWasPressed = false;
  }
}

// ============================================================================
// Battery / I2C Helpers
// ============================================================================

float readBatteryVoltage(int samples = 8) {
  uint32_t sum = 0;

  for (int i = 0; i < samples; i++) {
    sum += analogRead(BATTERY_PIN);
    delayMicroseconds(500);
  }

  float adcCounts = sum / (float)samples;
  float adcVoltage = (adcCounts / 4095.0f) * ADC_REF_VOLTAGE;

  return adcVoltage * BATTERY_DIVIDER_RATIO;
}

uint8_t readRegister(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);

  uint8_t err = Wire.endTransmission(false);
  if (err != 0) return 0xFF;

  Wire.requestFrom(addr, (uint8_t)1);

  if (Wire.available()) return Wire.read();

  return 0xFF;
}


// ============================================================================
// Fast MPL3115A2 Altimeter Access
// ============================================================================
// The Adafruit_MPL3115A2 getAltitude()/getTemperature() helpers can block for
// hundreds of milliseconds depending on oversampling/one-shot conversion state.
// These direct register reads keep the main loop responsive enough for 10 Hz
// telemetry and 10 Hz binary logging.

constexpr uint8_t MPL_ADDR        = 0x60;
constexpr uint8_t MPL_STATUS      = 0x00;
constexpr uint8_t MPL_OUT_P_MSB   = 0x01;
constexpr uint8_t MPL_PT_DATA_CFG = 0x13;
constexpr uint8_t MPL_CTRL_REG1   = 0x26;

bool writeRegister8(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readBurst(uint8_t addr, uint8_t startReg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(startReg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t got = Wire.requestFrom(addr, len);
  if (got != len) {
    return false;
  }

  for (uint8_t i = 0; i < len; i++) {
    if (!Wire.available()) return false;
    buf[i] = Wire.read();
  }

  return true;
}

bool configureMPLFastAltimeter() {
  // Put the part in standby before changing CTRL_REG1/PT_DATA_CFG.
  uint8_t ctrl = readRegister(MPL_ADDR, MPL_CTRL_REG1);
  if (ctrl == 0xFF) return false;

  if (!writeRegister8(MPL_ADDR, MPL_CTRL_REG1, ctrl & ~0x01)) return false;
  delay(10);

  // Enable data event flags for pressure/altitude and temperature.
  if (!writeRegister8(MPL_ADDR, MPL_PT_DATA_CFG, 0x07)) return false;

  // ALT=1 altimeter mode, OS=0 fastest oversampling, SBYB=1 active.
  // 0x81 = 1000 0001b.
  if (!writeRegister8(MPL_ADDR, MPL_CTRL_REG1, 0x81)) return false;
  delay(20);

  return true;
}

bool readMPLFast(float& altitude_m, float& temp_C) {
  uint8_t b[5];

  // OUT_P_MSB/CSB/LSB contain altitude in altimeter mode.
  // OUT_T_MSB/LSB contain temperature.
  if (!readBurst(MPL_ADDR, MPL_OUT_P_MSB, b, sizeof(b))) {
    return false;
  }

  // Altitude is a signed 20-bit fixed-point value with 4 fractional bits.
  // Build it as a sign-extended int32 and divide by 16.
  int32_t rawAlt = ((int32_t)b[0] << 24) | ((int32_t)b[1] << 16) | ((int32_t)(b[2] & 0xF0) << 8);
  rawAlt >>= 12;
  altitude_m = rawAlt / 16.0f;

  // Temperature is signed 12-bit fixed-point with 4 fractional bits.
  int16_t rawTemp = ((int16_t)b[3] << 8) | b[4];
  rawTemp >>= 4;
  temp_C = rawTemp / 16.0f;

  return true;
}

void scanI2C() {
  Serial.println();
  Serial.println("I2C Scan:");

  int found = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();

    if (err == 0) {
      Serial.print("Found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);

      if (addr == 0x60) Serial.print("  MPL3115A2");
      if (addr == 0x6B) Serial.print("  LSM6DSO32 possible");

      Serial.println();
      found++;
    }
  }

  if (found == 0) Serial.println("No I2C devices found.");

  Serial.println();
}

// ============================================================================
// Pyro
// ============================================================================

bool pyroActive() {
  return pyro.active;
}

void startPyro(PyroKind kind, uint8_t pin) {
  if (pyroActive()) return;

  pyro.kind = kind;
  pyro.pin = pin;
  pyro.active = true;
  pyro.t0 = millis();

  digitalWrite(pin, LOW);

  if (PYRO_OUTPUTS_ENABLED) {
    digitalWrite(pin, HIGH);
  } else {
    Serial.println("PYRO_OUTPUTS_ENABLED is false. MOSFET not energized.");
  }
}

void servicePyro(uint32_t now) {
  if (!pyroActive()) return;

  if (now - pyro.t0 >= PYRO_PULSE_MS) {
    digitalWrite(pyro.pin, LOW);

    pyro.kind = PyroKind::NONE;
    pyro.pin = 255;
    pyro.active = false;
    pyro.t0 = 0;
  }
}

// ============================================================================
// Checksums
// ============================================================================

uint16_t checksum16(const uint8_t* data, size_t len) {
  uint16_t sum = 0xA5A5;

  for (size_t i = 0; i < len; i++) {
    sum = (uint16_t)((sum << 1) | (sum >> 15));
    sum ^= data[i];
  }

  return sum;
}

uint32_t checksum32(const uint8_t* data, size_t len) {
  uint32_t sum = 0xA5A5A5A5UL;

  for (size_t i = 0; i < len; i++) {
    sum = (sum << 5) | (sum >> 27);
    sum ^= data[i];
  }

  return sum;
}

uint32_t settingsCrcCalc(CometSettings s) {
  s.crc = 0;
  return checksum32((const uint8_t*)&s, sizeof(CometSettings));
}

bool settingsValid(const CometSettings& s) {
  if (s.magic != SETTINGS_MAGIC) return false;
  if (s.version != SETTINGS_VERSION) return false;
  if (s.mainAltMode >= MAIN_ALT_MODE_COUNT) return false;

  return s.crc == settingsCrcCalc(s);
}

bool saveCometSettings() {
  if (!loggerMounted) {
    return false;
  }

  CometSettings s;
  memset(&s, 0, sizeof(s));

  s.magic = SETTINGS_MAGIC;
  s.version = SETTINGS_VERSION;
  s.mainAltMode = (uint8_t)rgbMode;
  s.reserved = 0;
  s.crc = settingsCrcCalc(s);

  File f = LittleFS.open(COMET_SETTINGS_FILE, "w");
  if (!f) {
    return false;
  }

  size_t n = f.write((const uint8_t*)&s, sizeof(s));
  f.flush();
  f.close();

  return n == sizeof(s);
}

bool loadCometSettings() {
  if (!loggerMounted) {
    return false;
  }

  if (!LittleFS.exists(COMET_SETTINGS_FILE)) {
    return false;
  }

  File f = LittleFS.open(COMET_SETTINGS_FILE, "r");
  if (!f) {
    return false;
  }

  CometSettings s;
  memset(&s, 0, sizeof(s));

  size_t n = f.read((uint8_t*)&s, sizeof(s));
  f.close();

  if (n != sizeof(s) || !settingsValid(s)) {
    Serial.println("SETTINGS INVALID: using default altitude mode.");
    return false;
  }

  setMainAltitudeMode(s.mainAltMode, true, false);
  Serial.println("SETTINGS LOADED: main altitude mode restored");

  return true;
}

uint32_t headerCrcCalc(FlightLogHeader h) {
  h.headerCrc = 0;
  return checksum32((const uint8_t*)&h, sizeof(FlightLogHeader));
}

bool headerValid(const FlightLogHeader& h) {
  if (h.magic != LOG_MAGIC) return false;
  if (h.version != LOG_VERSION) return false;
  if (h.recordSize != sizeof(FlightLogRecord)) return false;
  if (h.headerSize != sizeof(FlightLogHeader)) return false;

  return h.headerCrc == headerCrcCalc(h);
}

const char* slotStatusName(uint8_t s) {
  switch (s) {
    case SLOT_EMPTY:      return "EMPTY";
    case SLOT_RECORDING:  return "RECORDING";
    case SLOT_COMPLETE:   return "COMPLETE";
    case SLOT_DOWNLOADED: return "DOWNLOADED";
    case SLOT_BAD:        return "BAD";
    default:              return "UNKNOWN";
  }
}

// ============================================================================
// Logger Header Read/Write
// ============================================================================

bool readSlotHeader(uint8_t slot, FlightLogHeader& h) {
  memset(&h, 0, sizeof(h));

  if (slot >= LOG_SLOT_COUNT) return false;
  if (!LittleFS.exists(LOG_SLOT_FILES[slot])) return false;

  File f = LittleFS.open(LOG_SLOT_FILES[slot], "r");
  if (!f) return false;

  if (f.size() < sizeof(FlightLogHeader)) {
    f.close();
    return false;
  }

  size_t n = f.read((uint8_t*)&h, sizeof(FlightLogHeader));
  f.close();

  return n == sizeof(FlightLogHeader) && headerValid(h);
}

bool writeSlotHeader(uint8_t slot, FlightLogHeader& h) {
  if (slot >= LOG_SLOT_COUNT) return false;

  h.magic = LOG_MAGIC;
  h.version = LOG_VERSION;
  h.slotIndex = slot;
  h.recordSize = sizeof(FlightLogRecord);
  h.maxRecords = LOG_MAX_RECORDS;
  h.headerSize = sizeof(FlightLogHeader);
  h.headerCrc = headerCrcCalc(h);

  File f = LittleFS.open(LOG_SLOT_FILES[slot], "r+");

  if (!f) {
    f = LittleFS.open(LOG_SLOT_FILES[slot], "w+");
  }

  if (!f) return false;

  f.seek(0);
  size_t n = f.write((const uint8_t*)&h, sizeof(FlightLogHeader));
  f.flush();
  f.close();

  return n == sizeof(FlightLogHeader);
}

void makeEmptySlot(uint8_t slot) {
  if (slot >= LOG_SLOT_COUNT) return;

  LittleFS.remove(LOG_SLOT_FILES[slot]);

  FlightLogHeader h;
  memset(&h, 0, sizeof(h));

  h.slotIndex = slot;
  h.status = SLOT_EMPTY;
  h.flightNumber = 0;
  h.startMillis = 0;
  h.completedMillis = 0;
  h.recordCount = 0;
  h.downloaded = 0;

  writeSlotHeader(slot, h);
}

// ============================================================================
// Logger Slot Management
// ============================================================================

void listLogSlots(Stream& out = Serial) {
  out.println();
  out.println("===== FLIGHT LOG SLOTS =====");

  for (uint8_t i = 0; i < LOG_SLOT_COUNT; i++) {
    FlightLogHeader h;
    bool ok = readSlotHeader(i, h);

    out.print("SLOT ");
    out.print(i);
    out.print(": ");

    if (!ok) {
      out.println("EMPTY/UNINITIALIZED");
      continue;
    }

    out.print(slotStatusName(h.status));
    out.print(" flight=");
    out.print(h.flightNumber);
    out.print(" records=");
    out.print(h.recordCount);
    out.print("/");
    out.print(h.maxRecords);
    out.print(" duration_s=");
    out.print(h.recordCount / LOG_RATE_HZ);
    out.print(" downloaded=");
    out.print(h.downloaded ? "YES" : "NO");
    out.print(" file=");
    out.print(LOG_SLOT_FILES[i]);

    File f = LittleFS.open(LOG_SLOT_FILES[i], "r");
    if (f) {
      out.print(" bytes=");
      out.print(f.size());
      f.close();
    }

    out.println();
  }

  out.println("============================");
  out.println();
}

void recoverInterruptedSlots() {
  nextFlightNumber = 1;

  for (uint8_t i = 0; i < LOG_SLOT_COUNT; i++) {
    FlightLogHeader h;

    if (!readSlotHeader(i, h)) {
      makeEmptySlot(i);
      continue;
    }

    if (h.status == SLOT_RECORDING) {
      File f = LittleFS.open(LOG_SLOT_FILES[i], "r");

      if (f) {
        uint32_t possibleRecords = 0;

        if (f.size() > sizeof(FlightLogHeader)) {
          possibleRecords = (f.size() - sizeof(FlightLogHeader)) / sizeof(FlightLogRecord);
        }

        f.close();

        h.recordCount = possibleRecords;
        h.status = possibleRecords > 0 ? SLOT_COMPLETE : SLOT_EMPTY;
        h.completedMillis = 0;

        writeSlotHeader(i, h);
      }
    }

    if (h.flightNumber >= nextFlightNumber) {
      nextFlightNumber = h.flightNumber + 1;
    }
  }
}

int8_t chooseSafeSlotForLogging() {
  for (uint8_t i = 0; i < LOG_SLOT_COUNT; i++) {
    FlightLogHeader h;

    if (!readSlotHeader(i, h)) return i;
    if (h.status == SLOT_EMPTY) return i;
  }

  int8_t bestSlot = -1;
  uint32_t oldestFlight = 0xFFFFFFFFUL;

  for (uint8_t i = 0; i < LOG_SLOT_COUNT; i++) {
    FlightLogHeader h;

    if (!readSlotHeader(i, h)) continue;

    if (h.status == SLOT_DOWNLOADED && h.flightNumber < oldestFlight) {
      oldestFlight = h.flightNumber;
      bestSlot = i;
    }
  }

  return bestSlot;
}

int8_t findOldestSavedSlot() {
  int8_t bestSlot = -1;
  uint32_t oldestFlight = 0xFFFFFFFFUL;

  for (uint8_t i = 0; i < LOG_SLOT_COUNT; i++) {
    FlightLogHeader h;

    if (!readSlotHeader(i, h)) continue;

    // Do not clear truly empty slots. Pick the oldest saved/recoverable flight.
    if (h.status == SLOT_COMPLETE || h.status == SLOT_DOWNLOADED || h.status == SLOT_RECORDING) {
      uint32_t flightNum = h.flightNumber;
      if (flightNum < oldestFlight) {
        oldestFlight = flightNum;
        bestSlot = i;
      }
    }
  }

  return bestSlot;
}

bool clearOldestFlightSlot() {
  if (!loggerMounted) {
    Serial.println("CLEAR_OLDEST: LittleFS not mounted.");
    return false;
  }

  int8_t slot = findOldestSavedSlot();

  if (slot < 0) {
    Serial.println("CLEAR_OLDEST: no saved slot found to clear.");
    return false;
  }

  Serial.print("CLEAR_OLDEST: erasing slot ");
  Serial.println(slot);

  LittleFS.remove(LOG_SLOT_FILES[slot]);
  makeEmptySlot((uint8_t)slot);

  loggerFull = false;
  fullButtonHoldStartMs = 0;
  fullBlinkOn = false;
  updateRGBMode();

  return true;
}

void serviceLoggerFullUI(uint32_t now) {
  if (!loggerFull || loggerActive) return;

  // Blink yellow while there is no safe logging slot.
  if (now - fullBlinkLastMs >= FULL_YELLOW_BLINK_MS) {
    fullBlinkLastMs = now;
    fullBlinkOn = !fullBlinkOn;
    setRGB(fullBlinkOn, fullBlinkOn, 0);
  }

  bool pressed = digitalRead(MODE_SWITCH_PIN) == LOW;

  if (pressed) {
    if (fullButtonHoldStartMs == 0) {
      fullButtonHoldStartMs = now;
      Serial.println("LOGGER FULL: hold MODE for 5 seconds to clear oldest slot.");
    }

    if (now - fullButtonHoldStartMs >= FULL_SLOT_HOLD_CLEAR_MS) {
      Serial.println("LOGGER FULL: clearing oldest slot now.");
      startRapidBeep(1);

      if (clearOldestFlightSlot()) {
        listLogSlots();
        bool ok = startNewLogIfSafe();
        Serial.print("AUTO START AFTER CLEAR: ");
        Serial.println(ok ? "OK" : "FAILED");
      } else {
        loggerFull = true;
      }

      fullButtonHoldStartMs = 0;
    }
  } else {
    fullButtonHoldStartMs = 0;
  }
}

// ============================================================================
// Logger Start / Stop
// ============================================================================

bool beginFlightLogger() {
  loggerMounted = LittleFS.begin();

  if (!loggerMounted) {
    Serial.println("LittleFS mount failed. Trying format...");
    loggerMounted = LittleFS.format();

    if (loggerMounted) {
      loggerMounted = LittleFS.begin();
    }
  }

  if (!loggerMounted) {
    Serial.println("LOGGER ERROR: LittleFS unavailable.");
    return false;
  }

  recoverInterruptedSlots();
  listLogSlots();

  return true;
}

bool startNewLogIfSafe() {
  if (!LOGGING_ENABLED) return false;
  if (!loggerMounted) return false;

  if (downloadMode) {
    Serial.println("Download mode active. Logger will not start.");
    return false;
  }

  activeSlot = chooseSafeSlotForLogging();

  if (activeSlot < 0) {
    loggerFull = true;
    loggerActive = false;
    fullButtonHoldStartMs = 0;
    fullBlinkLastMs = 0;
    fullBlinkOn = false;

    Serial.println("LOGGER FULL: no EMPTY or DOWNLOADED slots available.");
    Serial.println("RGB will blink YELLOW. Hold MODE for 5 seconds to clear the oldest saved flight slot.");

    return false;
  }

  LittleFS.remove(LOG_SLOT_FILES[activeSlot]);

  memset(&activeHeader, 0, sizeof(activeHeader));

  activeHeader.slotIndex = activeSlot;
  activeHeader.status = SLOT_RECORDING;
  activeHeader.flightNumber = nextFlightNumber++;
  activeHeader.startMillis = millis();
  activeHeader.completedMillis = 0;
  activeHeader.recordCount = 0;
  activeHeader.downloaded = 0;

  if (!writeSlotHeader(activeSlot, activeHeader)) {
    Serial.println("LOGGER ERROR: failed to write new slot header.");
    loggerActive = false;
    return false;
  }

  logFile = LittleFS.open(LOG_SLOT_FILES[activeSlot], "a");

  if (!logFile) {
    Serial.println("LOGGER ERROR: failed to open active log file.");
    loggerActive = false;
    return false;
  }

  loggerActive = true;
  loggerFull = false;
  lastLogMs = millis();

  Serial.print("LOGGER STARTED: slot=");
  Serial.print(activeSlot);
  Serial.print(" flight=");
  Serial.println(activeHeader.flightNumber);

  return true;
}

void finishActiveLog(const char* reason = "COMPLETE") {
  if (!loggerActive) return;

  if (logFile) {
    logFile.flush();
    logFile.close();
  }

  activeHeader.status = SLOT_COMPLETE;
  activeHeader.completedMillis = millis();

  writeSlotHeader(activeSlot, activeHeader);

  loggerActive = false;

  Serial.print("LOGGER COMPLETE: slot=");
  Serial.print(activeSlot);
  Serial.print(" records=");
  Serial.print(activeHeader.recordCount);
  Serial.print(" reason=");
  Serial.println(reason ? reason : "-");

  listLogSlots();
}

// ============================================================================
// Logger Record Packing
// ============================================================================

uint8_t makeLogFlags() {
  uint8_t flags = 0;

  if (drogueFired)  flags |= (1 << 0);
  if (mainFired)    flags |= (1 << 1);
  if (mainArmed)    flags |= (1 << 2);
  if (pyroActive()) flags |= (1 << 3);
  if (mplOK)        flags |= (1 << 4);
  if (imuOK)        flags |= (1 << 5);
  if (t_launch > 0) flags |= (1 << 6);
  if (loggerFull)   flags |= (1 << 7);

  return flags;
}

uint16_t makeEventFlags() {
  uint16_t e = 0;

  if (state == FlightState::ASCENT_LOCKOUT)   e |= (1 << 0);
  if (state == FlightState::APOGEE_DETECT)    e |= (1 << 1);
  if (state == FlightState::DROGUE_DEPLOYED)  e |= (1 << 2);
  if (state == FlightState::MAIN_DEPLOYED)    e |= (1 << 3);
  if (loggerFull)                             e |= (1 << 4);
  if (pyroActive())                           e |= (1 << 5);

  // Upper byte carries the human-readable event code for CSV export.
  e |= ((uint16_t)pendingLogEventCode << 8);

  return e;
}

FlightLogRecord makeCurrentLogRecord() {
  FlightLogRecord r;
  memset(&r, 0, sizeof(r));

  float outAlt_m = (t_launch > 0) ? altAGL_m : alt_m;

  float ax_g = ax_mps2 / 9.80665f;
  float ay_g = ay_mps2 / 9.80665f;
  float az_g = az_mps2 / 9.80665f;

  r.t_ms    = millis();

  r.alt_cm  = (int32_t)lroundf(outAlt_m * 100.0f);
  r.vz_cms  = clampI16(lroundf(vz_mps * 100.0f));

  r.ax_cg   = clampI16(lroundf(ax_g * 100.0f));
  r.ay_cg   = clampI16(lroundf(ay_g * 100.0f));
  r.az_cg   = clampI16(lroundf(az_g * 100.0f));

  r.gx_cdps = clampI16(lroundf(gx_dps * 100.0f));
  r.gy_cdps = clampI16(lroundf(gy_dps * 100.0f));
  r.gz_cdps = clampI16(lroundf(gz_dps * 100.0f));

  r.batt_mv = clampU16(lroundf(batt_V * 1000.0f));
  r.temp_cC = clampI16(lroundf((mplOK ? mplTemp_C : imuTemp_C) * 100.0f));

  r.state   = (uint8_t)state;
  r.flags   = makeLogFlags();

  r.eventFlags = makeEventFlags();

  r.crc = 0;
  r.crc = checksum16((const uint8_t*)&r, sizeof(FlightLogRecord));

  return r;
}

void updateActiveHeaderSafely() {
  if (!loggerActive) return;

  if (logFile) {
    logFile.flush();
    logFile.close();
  }

  writeSlotHeader(activeSlot, activeHeader);

  logFile = LittleFS.open(LOG_SLOT_FILES[activeSlot], "a");

  if (!logFile) {
    Serial.println("LOGGER ERROR: failed to reopen file after header update.");
    loggerActive = false;
  }
}

void serviceFlightLogger(uint32_t now) {
  if (!loggerActive) return;

  if (now - lastLogMs < LOG_PERIOD_MS) return;
  lastLogMs += LOG_PERIOD_MS;

  if (activeHeader.recordCount >= LOG_MAX_RECORDS) {
    finishActiveLog("ONE_HOUR_LIMIT");
    return;
  }

  FlightLogRecord r = makeCurrentLogRecord();

  size_t n = logFile.write((const uint8_t*)&r, sizeof(r));

  if (n != sizeof(r)) {
    Serial.println("LOGGER ERROR: write failed.");
    finishActiveLog("WRITE_FAIL");
    return;
  }

  activeHeader.recordCount++;

  // The pending event has now been captured in this row's eventFlags upper byte.
  pendingLogEventCode = LOG_EVENT_NONE;

  // Avoid periodic header rewrites during flight. LittleFS header updates
  // caused noticeable multi-second pauses at exact record-count intervals.
  // Flush the file data occasionally instead; finishActiveLog() writes the
  // final record count and slot status cleanly when the log is stopped.
  if ((activeHeader.recordCount % LOG_DATA_FLUSH_RECORDS) == 0) {
    logFile.flush();
  }
}

// ============================================================================
// Download / Management Commands
// ============================================================================

void dumpSlotCsv(uint8_t slot) {
  if (slot >= LOG_SLOT_COUNT) {
    Serial.println("ERR: invalid slot");
    return;
  }

  if (loggerActive && activeSlot == slot) {
    Serial.println("ERR: slot is actively recording. Use STOPLOG first if safe.");
    return;
  }

  FlightLogHeader h;

  if (!readSlotHeader(slot, h)) {
    Serial.println("ERR: slot empty or invalid");
    return;
  }

  if (h.recordCount == 0) {
    Serial.println("ERR: no records");
    return;
  }

  File f = LittleFS.open(LOG_SLOT_FILES[slot], "r");

  if (!f) {
    Serial.println("ERR: could not open slot");
    return;
  }

  f.seek(sizeof(FlightLogHeader));

  Serial.println();
  Serial.print("BEGIN_CSV SLOT ");
  Serial.println(slot);

  Serial.println(
    "t_ms,state,state_name,flags,eventFlags,eventCode,"
    "alt_m,vz_mps,"
    "ax_g,ay_g,az_g,"
    "gx_dps,gy_dps,gz_dps,"
    "batt_V,temp_C,crc_ok,event_note"
  );

  for (uint32_t i = 0; i < h.recordCount; i++) {
    FlightLogRecord r;

    size_t n = f.read((uint8_t*)&r, sizeof(r));
    if (n != sizeof(r)) break;

    uint16_t oldCrc = r.crc;
    r.crc = 0;
    uint16_t calc = checksum16((const uint8_t*)&r, sizeof(r));

    uint8_t eventCode = (uint8_t)((r.eventFlags >> 8) & 0xFF);

    Serial.print(r.t_ms);
    Serial.print(',');
    Serial.print(r.state);
    Serial.print(',');
    Serial.print(stateStr((FlightState)r.state));
    Serial.print(',');
    Serial.print(r.flags);
    Serial.print(',');
    Serial.print(r.eventFlags);
    Serial.print(',');
    Serial.print(eventCode);
    Serial.print(',');

    Serial.print(r.alt_cm / 100.0f, 2);
    Serial.print(',');
    Serial.print(r.vz_cms / 100.0f, 2);
    Serial.print(',');

    Serial.print(r.ax_cg / 100.0f, 2);
    Serial.print(',');
    Serial.print(r.ay_cg / 100.0f, 2);
    Serial.print(',');
    Serial.print(r.az_cg / 100.0f, 2);
    Serial.print(',');

    Serial.print(r.gx_cdps / 100.0f, 2);
    Serial.print(',');
    Serial.print(r.gy_cdps / 100.0f, 2);
    Serial.print(',');
    Serial.print(r.gz_cdps / 100.0f, 2);
    Serial.print(',');

    Serial.print(r.batt_mv / 1000.0f, 3);
    Serial.print(',');
    Serial.print(r.temp_cC / 100.0f, 2);
    Serial.print(',');

    Serial.print(oldCrc == calc ? 1 : 0);
    Serial.print(',');
    Serial.println(logEventNote(eventCode));

    if ((i % 64) == 0) {
      delay(1);
    }
  }

  Serial.print("END_CSV SLOT ");
  Serial.println(slot);
  Serial.println();

  f.close();
}

void dumpSlotBinary(uint8_t slot) {
  if (slot >= LOG_SLOT_COUNT) {
    Serial.println("ERR: invalid slot");
    return;
  }

  if (loggerActive && activeSlot == slot) {
    Serial.println("ERR: slot is actively recording. Use STOPLOG first if safe.");
    return;
  }

  FlightLogHeader h;

  if (!readSlotHeader(slot, h)) {
    Serial.println("ERR: slot empty or invalid");
    return;
  }

  File f = LittleFS.open(LOG_SLOT_FILES[slot], "r");

  if (!f) {
    Serial.println("ERR: could not open slot");
    return;
  }

  Serial.print("BEGIN_BIN SLOT ");
  Serial.print(slot);
  Serial.print(" BYTES ");
  Serial.println(f.size());

  while (f.available()) {
    uint8_t buf[64];
    int n = f.read(buf, sizeof(buf));

    if (n <= 0) break;

    Serial.write(buf, n);
    delay(1);
  }

  Serial.println();
  Serial.print("END_BIN SLOT ");
  Serial.println(slot);

  f.close();
}

void markSlotDownloaded(uint8_t slot) {
  if (slot >= LOG_SLOT_COUNT) {
    Serial.println("ERR: invalid slot");
    return;
  }

  if (loggerActive && activeSlot == slot) {
    Serial.println("ERR: cannot mark active recording slot");
    return;
  }

  FlightLogHeader h;

  if (!readSlotHeader(slot, h)) {
    Serial.println("ERR: slot empty or invalid");
    return;
  }

  if (h.status != SLOT_COMPLETE && h.status != SLOT_RECORDING) {
    Serial.println("ERR: only COMPLETE/RECORDING slots can be marked downloaded");
    return;
  }

  h.status = SLOT_DOWNLOADED;
  h.downloaded = 1;

  if (writeSlotHeader(slot, h)) {
    Serial.print("OK: slot marked DOWNLOADED: ");
    Serial.println(slot);
  } else {
    Serial.println("ERR: failed to update slot");
  }
}

void eraseSlot(uint8_t slot) {
  if (slot >= LOG_SLOT_COUNT) {
    Serial.println("ERR: invalid slot");
    return;
  }

  if (loggerActive && activeSlot == slot) {
    Serial.println("ERR: cannot erase active recording slot");
    return;
  }

  LittleFS.remove(LOG_SLOT_FILES[slot]);
  makeEmptySlot(slot);

  Serial.print("OK: erased slot ");
  Serial.println(slot);
}

void formatAllLogs() {
  if (loggerActive) {
    Serial.println("ERR: cannot format while logging");
    return;
  }

  for (uint8_t i = 0; i < LOG_SLOT_COUNT; i++) {
    LittleFS.remove(LOG_SLOT_FILES[i]);
    makeEmptySlot(i);
  }

  Serial.println("OK: all log slots erased");
}

void printLogStatus() {
  Serial.println();
  Serial.println("===== LOGGER STATUS =====");
  Serial.print("loggerMounted: "); Serial.println(loggerMounted ? "YES" : "NO");
  Serial.print("loggerActive: "); Serial.println(loggerActive ? "YES" : "NO");
  Serial.print("loggerFull: "); Serial.println(loggerFull ? "YES" : "NO");
  Serial.print("downloadMode: "); Serial.println(downloadMode ? "YES" : "NO");
  Serial.print("activeSlot: "); Serial.println(activeSlot);
  Serial.print("recordCount: "); Serial.println((unsigned long)activeHeader.recordCount);
  Serial.print("nextFlightNumber: "); Serial.println((unsigned long)nextFlightNumber);
  Serial.print("MODE_SWITCH_PIN raw: "); Serial.println(digitalRead(MODE_SWITCH_PIN));
  Serial.print("mainAltMode: "); Serial.print(rgbMode);
  if (rgbMode >= 0 && rgbMode < MAIN_ALT_MODE_COUNT) {
    Serial.print(" ");
    Serial.print(MAIN_ALT_MODES[rgbMode].name);
  }
  Serial.print(" mainAltM: "); Serial.println(mainAltM, 1);

  if (loggerMounted) {
    FSInfo info;
    LittleFS.info(info);
    Serial.print("LittleFS total bytes: "); Serial.println((unsigned long)info.totalBytes);
    Serial.print("LittleFS used bytes: "); Serial.println((unsigned long)info.usedBytes);
  }

  Serial.println("=========================");
  Serial.println();
}

void printParams() {
  Serial.println();
  Serial.println("===== COMET PARAMETERS =====");
  if (rgbMode >= 0 && rgbMode < MAIN_ALT_MODE_COUNT) {
    Serial.print("PARAM MAIN_ALT_MODE ");
    Serial.print(rgbMode);
    Serial.print(' ');
    Serial.println(MAIN_ALT_MODES[rgbMode].name);
  }
  Serial.print("PARAM MAIN_ALT "); Serial.println(mainAltM, 2);
  Serial.print("PARAM MAIN_ARM_MARGIN "); Serial.println(mainArmMarginM, 2);
  Serial.print("PARAM DROGUE_BACKUP_MS "); Serial.println((unsigned long)drogueBackupMs);
  Serial.print("PARAM MAIN_BACKUP_MS "); Serial.println((unsigned long)mainBackupMs);
  Serial.print("PARAM LOCKOUT_MS "); Serial.println((unsigned long)flightLockoutMs);
  Serial.println("============================");
  Serial.println();
}

bool setParam(const String& name, const String& value) {
  float fv = value.toFloat();
  uint32_t uv = (uint32_t)value.toInt();

  if (name == "MAIN_ALT_MODE") {
    int mode = findMainAltitudeMode(value);
    if (mode < 0) return false;
    setMainAltitudeMode((uint8_t)mode, true, true);
  } else if (name == "MAIN_ALT") {
    if (fv < 1.0f || fv > 10000.0f) return false;
    mainAltM = fv;
  } else if (name == "MAIN_ARM_MARGIN") {
    if (fv < 0.0f || fv > 1000.0f) return false;
    mainArmMarginM = fv;
  } else if (name == "DROGUE_BACKUP_MS") {
    if (uv < 1000UL || uv > 600000UL) return false;
    drogueBackupMs = uv;
  } else if (name == "MAIN_BACKUP_MS") {
    if (uv < 1000UL || uv > 600000UL) return false;
    mainBackupMs = uv;
  } else if (name == "LOCKOUT_MS") {
    if (uv < 0UL || uv > 120000UL) return false;
    flightLockoutMs = uv;
  } else {
    return false;
  }

  Serial.print("OK SET ");
  Serial.print(name);
  Serial.print(' ');
  Serial.println(value);
  return true;
}

void printLoggerHelp() {
  Serial.println();
  Serial.println("Logger commands:");
  Serial.println("  LIST                 - list flight slots");
  Serial.println("  DUMPCSV <0|1|2>      - dump slot as CSV with state_name and event_note columns");
  Serial.println("  DUMPBIN <0|1|2>      - dump raw binary slot over USB");
  Serial.println("  MARKDOWNLOADED <n>   - mark slot safe for reuse");
  Serial.println("  ERASE <n>            - erase one slot");
  Serial.println("  FORMATLOG            - erase all slots");
  Serial.println("  STOPLOG              - safely close active log");
  Serial.println("  STARTLOG             - start a new log if a safe slot exists");
  Serial.println("  LOGSTATUS            - print logger status");
  Serial.println("  GETPARAMS            - print editable flight parameters");
  Serial.println("  SET MAIN_ALT_MODE <0-6|COLOR> - select and save ROYGBIV main altitude mode");
  Serial.println("  SET <PARAM> <VALUE>  - update editable parameter");
  Serial.println("  LOGHELP              - show logger help");
  Serial.println();
}

bool handleLoggerCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "LIST") {
    listLogSlots();
    return true;
  }

  if (cmd.startsWith("DUMPCSV")) {
    int slot = cmd.substring(7).toInt();
    dumpSlotCsv((uint8_t)slot);
    return true;
  }

  if (cmd.startsWith("DUMPBIN")) {
    int slot = cmd.substring(7).toInt();
    dumpSlotBinary((uint8_t)slot);
    return true;
  }

  if (cmd.startsWith("MARKDOWNLOADED")) {
    int slot = cmd.substring(14).toInt();
    markSlotDownloaded((uint8_t)slot);
    return true;
  }

  if (cmd.startsWith("ERASE")) {
    int slot = cmd.substring(5).toInt();
    eraseSlot((uint8_t)slot);
    return true;
  }

  if (cmd == "FORMATLOG") {
    formatAllLogs();
    return true;
  }

  if (cmd == "STOPLOG") {
    if (!loggerActive) {
      Serial.println("STOPLOG: no active log to stop.");
      printLogStatus();
      return true;
    }
    finishActiveLog("USER_STOP");
    return true;
  }

  if (cmd == "STARTLOG") {
    if (loggerActive) {
      Serial.println("STARTLOG: logger is already active.");
      printLogStatus();
      return true;
    }
    if (!loggerMounted) {
      Serial.println("STARTLOG: LittleFS is not mounted.");
      printLogStatus();
      return true;
    }
    if (downloadMode) {
      Serial.println("STARTLOG: overriding downloadMode for manual start.");
      downloadMode = false;
    }
    bool ok = startNewLogIfSafe();
    Serial.print("STARTLOG result: ");
    Serial.println(ok ? "OK" : "FAILED");
    return true;
  }

  if (cmd == "LOGSTATUS") {
    printLogStatus();
    return true;
  }

  if (cmd == "GETPARAMS") {
    printParams();
    return true;
  }

  if (cmd.startsWith("SET ")) {
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);

    if (firstSpace < 0 || secondSpace < 0) {
      Serial.println("ERR: use SET <PARAM> <VALUE>");
      return true;
    }

    String name = cmd.substring(firstSpace + 1, secondSpace);
    String value = cmd.substring(secondSpace + 1);
    name.trim();
    value.trim();

    if (!setParam(name, value)) {
      Serial.print("ERR: invalid parameter or value: ");
      Serial.print(name);
      Serial.print(' ');
      Serial.println(value);
    }
    return true;
  }

  if (cmd == "LOGHELP") {
    printLoggerHelp();
    return true;
  }

  return false;
}

// Forward declaration: readSensors() can report MAIN_ARMED before the
// sendTelemetry() function body appears later in this sketch.
void sendTelemetry(const char* eventName);

// ============================================================================
// Sensor Read + Vz
// ============================================================================

void updateVz(float alt, uint32_t tnow) {
  ring[ringHead] = {tnow, alt};
  ringHead = (ringHead + 1) % VZ_WIN;

  if (ringCount < VZ_WIN) ringCount++;

  if (ringCount >= 2) {
    int oldest = (ringHead + VZ_WIN - ringCount) % VZ_WIN;

    const AltSample &a = ring[oldest];
    const AltSample &b = ring[(ringHead + VZ_WIN - 1) % VZ_WIN];

    float dt = (b.t - a.t) / 1000.0f;

    if (dt > 0.0f) {
      vz_mps = (b.alt - a.alt) / dt;
    }
  }
}

bool readSensors() {
  const uint32_t now = millis();
  bool baroUpdated = false;

  // ---------------- IMU: fast update ----------------
  if (imuOK && (now - lastImuReadMs >= IMU_READ_PERIOD_MS)) {
    lastImuReadMs += IMU_READ_PERIOD_MS;
    if (now - lastImuReadMs > IMU_READ_PERIOD_MS * 2) {
      lastImuReadMs = now;
    }

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    imu.getEvent(&accel, &gyro, &temp);

    ax_mps2 = safe0(accel.acceleration.x);
    ay_mps2 = safe0(accel.acceleration.y);
    az_mps2 = safe0(accel.acceleration.z);

    gx_dps = safe0(radToDps(gyro.gyro.x));
    gy_dps = safe0(radToDps(gyro.gyro.y));
    gz_dps = safe0(radToDps(gyro.gyro.z));

    imuTemp_C = safe0(temp.temperature);

    float ax_g = gFromMps2(ax_mps2);
    float ay_g = gFromMps2(ay_mps2);
    float az_g = gFromMps2(az_mps2);

    accelMag_g = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
  }

  // ---------------- Barometer: non-blocking 10 Hz update ----------------
  // Direct register read avoids the slow blocking Adafruit getAltitude() path.
  if (mplOK && (now - lastBaroReadMs >= BARO_READ_PERIOD_MS)) {
    lastBaroReadMs += BARO_READ_PERIOD_MS;
    if (now - lastBaroReadMs > BARO_READ_PERIOD_MS * 2) {
      lastBaroReadMs = now;
    }

    float absAlt = 0.0f;
    float tempC = 0.0f;

    if (readMPLFast(absAlt, tempC)) {
      alt_m = safe0(absAlt - altRef_m);
      mplTemp_C = safe0(tempC);
      baroUpdated = true;
    }
  }

  // ---------------- AGL and main arming ----------------
  if (t_launch > 0) {
    altAGL_m = alt_m - launchAlt_m;

    if (altAGL_m > maxAltAGL_m) {
      maxAltAGL_m = altAGL_m;
    }

    if (!mainArmed && maxAltAGL_m > (mainAltM + mainArmMarginM)) {
      mainArmed = true;
      Serial.println("MAIN ARMED: exceeded main altitude plus margin.");
      sendTelemetry("MAIN_ARMED_ALTITUDE");
    }
  } else {
    altAGL_m = 0.0f;
    maxAltAGL_m = 0.0f;
    mainArmed = false;
  }

  // ---------------- Battery: slow update ----------------
  if (now - lastBatteryReadMs >= BATTERY_READ_PERIOD_MS) {
    lastBatteryReadMs += BATTERY_READ_PERIOD_MS;
    if (now - lastBatteryReadMs > BATTERY_READ_PERIOD_MS * 2) {
      lastBatteryReadMs = now;
    }

    batt_V = safe0(readBatteryVoltage(4));
  }

  // Only update vertical speed when a new barometer sample exists.
  if (baroUpdated) {
    float vzAlt = (t_launch > 0) ? altAGL_m : alt_m;
    updateVz(vzAlt, now);
  }

  return true;
}

bool calibrateAccelBaselineOnBoot(uint16_t samples = 60, uint16_t sampleDelayMs = 10) {
  if (!imuOK) return false;

  Serial.println("Calibrating accelerometer launch baseline on boot...");

  float sum_g = 0.0f;
  uint16_t good = 0;

  for (uint16_t i = 0; i < samples; i++) {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    imu.getEvent(&accel, &gyro, &temp);

    ax_mps2 = safe0(accel.acceleration.x);
    ay_mps2 = safe0(accel.acceleration.y);
    az_mps2 = safe0(accel.acceleration.z);

    gx_dps = safe0(radToDps(gyro.gyro.x));
    gy_dps = safe0(radToDps(gyro.gyro.y));
    gz_dps = safe0(radToDps(gyro.gyro.z));

    imuTemp_C = safe0(temp.temperature);

    float ax_g = gFromMps2(ax_mps2);
    float ay_g = gFromMps2(ay_mps2);
    float az_g = gFromMps2(az_mps2);
    float mag_g = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

    if (finitef_safe(mag_g) && mag_g > 0.1f && mag_g < 40.0f) {
      sum_g += mag_g;
      good++;
    }

    delay(sampleDelayMs);
  }

  if (good < 5) {
    Serial.println("ACCEL BASELINE ERROR: not enough valid IMU samples.");
    accelBaselineValid = false;
    return false;
  }

  accelMagBaseline_g = sum_g / (float)good;
  accelMag_g = accelMagBaseline_g;
  accelBaselineValid = true;
  baselineStartMs = millis();
  launchAccelStartMs = 0;

  Serial.print("ACCEL BASELINE LOCKED: ");
  Serial.print(accelMagBaseline_g, 3);
  Serial.println(" g");

  return true;
}

void updateAccelBaseline(uint32_t now) {
  // Baseline is intentionally locked once at boot so it cannot chase hand motion
  // or launch acceleration. This fallback only sets it once if boot calibration
  // somehow did not complete.
  if (!imuOK) return;
  if (accelBaselineValid) return;
  if (state != FlightState::IDLE) return;
  if (now - baselineStartMs < BASELINE_MIN_MS) return;

  accelMagBaseline_g = accelMag_g;
  accelBaselineValid = true;

  Serial.print("ACCEL BASELINE FALLBACK LOCKED: ");
  Serial.print(accelMagBaseline_g, 3);
  Serial.println(" g");
}

bool launchDetectAccel(uint32_t now) {
  if (!imuOK || !accelBaselineValid) return false;
  if (state != FlightState::IDLE) return false;

  float delta_g = accelMag_g - accelMagBaseline_g;
  bool trig = delta_g >= LAUNCH_ACCEL_DELTA_G;

  if (trig) {
    if (launchAccelStartMs == 0) launchAccelStartMs = now;
    return (now - launchAccelStartMs) >= LAUNCH_ACCEL_DWELL_MS;
  }

  launchAccelStartMs = 0;
  return false;
}

bool positiveTrend(uint32_t dwell_ms) {
  static uint32_t posStart = 0;

  if (vz_mps > ASCENT_POS_VZ_THRESH) {
    if (posStart == 0) posStart = millis();
    return (millis() - posStart) >= dwell_ms;
  }

  posStart = 0;
  return false;
}

bool negativeTrend(uint32_t dwell_ms) {
  static uint32_t negStart = 0;

  if (vz_mps < APOGEE_NEG_VZ_THRESH) {
    if (negStart == 0) negStart = millis();
    return (millis() - negStart) >= dwell_ms;
  }

  negStart = 0;
  return false;
}

// ============================================================================
// Telemetry
// ============================================================================

void sendTelemetry(const char* eventName = nullptr) {
  const uint32_t now = millis();

  uint8_t code = logEventCodeFromName(eventName);
  if (code != LOG_EVENT_NONE) {
    pendingLogEventCode = code;
  }

  float outAlt_m = (t_launch > 0) ? altAGL_m : alt_m;

  char pkt[420];

  // Keep this line parser-friendly for the COMET Python GUI:
  // DATA:<t>:KEY:VALUE:KEY:VALUE...
  // Pressure is omitted to keep packets shorter and closer to true 10 Hz.
  snprintf(pkt, sizeof(pkt),
    "DATA:%lu:"
    "STATE:%s:"
    "EVENT:%s:"
    "ALT:%.2f:"
    "VZ:%.2f:"
    "MAXALT:%.2f:"
    "BATT:%.2f:"
    "AX:%.2f:"
    "AY:%.2f:"
    "AZ:%.2f:"
    "AMAG:%.2f:"
    "GX:%.2f:"
    "GY:%.2f:"
    "GZ:%.2f:"
    "TEMP:%.2f:"
    "DROGUE:%u:"
    "MAIN:%u:"
    "MAINARM:%u:"
    "MODE:%d:"
    "MAINALT:%.0f:"
    "PYRO:%u:"
    "LOG:%u:"
    "SLOT:%d:"
    "REC:%lu",
    (unsigned long)now,
    stateStr(state),
    eventName ? eventName : "-",
    safe0(outAlt_m),
    safe0(vz_mps),
    safe0(maxAltAGL_m),
    safe0(batt_V),
    safe0(ax_mps2),
    safe0(ay_mps2),
    safe0(az_mps2),
    safe0(accelMag_g),
    safe0(gx_dps),
    safe0(gy_dps),
    safe0(gz_dps),
    safe0(mplOK ? mplTemp_C : imuTemp_C),
    drogueFired ? 1 : 0,
    mainFired ? 1 : 0,
    mainArmed ? 1 : 0,
    rgbMode,
    safe0(mainAltM),
    pyroActive() ? 1 : 0,
    loggerActive ? 1 : 0,
    activeSlot,
    (unsigned long)activeHeader.recordCount
  );

  Serial.println(pkt);
  Serial1.println(pkt);
}

// ============================================================================
// Deployment
// ============================================================================

void deployDrogue(const char* reason, bool force = false) {
  if (drogueFired) return;
  if (pyroActive()) return;

  if (!force && vz_mps > 0.0f) {
    Serial.println("Drogue blocked: vehicle still ascending.");
    return;
  }

  drogueFired = true;
  t_drogue_fire = millis();

  Serial.print(">>> DROGUE DEPLOY CMD: ");
  Serial.println(reason ? reason : "-");

  startDrogueFiredSound();
  sendTelemetry(reason ? reason : "DROGUE_DEPLOY");

  startPyro(PyroKind::DROGUE, DROGUE_PIN);
}

void deployMain(const char* reason, bool force = false) {
  if (mainFired) return;
  if (pyroActive()) return;

  if (!force && vz_mps > 0.0f) {
    Serial.println("Main blocked: vehicle still ascending.");
    return;
  }

  mainFired = true;
  t_main_fire = millis();

  Serial.print(">>> MAIN DEPLOY CMD: ");
  Serial.println(reason ? reason : "-");

  startMainFiredSound();
  sendTelemetry(reason ? reason : "MAIN_DEPLOY");

  startPyro(PyroKind::MAIN, MAIN_PIN);
}

// ============================================================================
// Test / Serial Commands
// ============================================================================

void printHelp() {
  Serial.println();
  Serial.println("Flight commands:");
  Serial.println("  HELP       - show commands");
  Serial.println("  STATUS     - print one telemetry packet");
  Serial.println("  DROGUE     - test drogue command path");
  Serial.println("  MAIN       - test main command path");
  Serial.println("  LAUNCH     - force launch detect for bench test");
  Serial.println("  RESET      - reset flight state to IDLE");
  Serial.println("  BEEP       - short buzzer chirp for GUI/button feedback");
  Serial.println("  SET MAIN_ALT_MODE <0-6|COLOR> - save selected main altitude preset");
  Serial.println("  MODE BTN   - cycle main altitude preset: RED 800, ORANGE 700, YELLOW 600,");
  Serial.println("               GREEN 500, BLUE 400, INDIGO 300, VIOLET 200 meters");
  Serial.println();
}

void resetFlightState() {
  drogueFired = false;
  mainFired = false;
  mainArmed = false;

  t_launch = 0;
  t_drogue_fire = 0;
  t_main_fire = 0;

  launchAlt_m = 0.0f;
  altAGL_m = 0.0f;
  maxAltAGL_m = 0.0f;

  ringHead = 0;
  ringCount = 0;
  vz_mps = 0.0f;

  // Keep the boot-time launch baseline locked. RESET should reset the flight
  // state, not retrain the launch detector while the board may be moving.
  launchAccelStartMs = 0;

  state = FlightState::IDLE;

  Serial.println("Flight state reset to IDLE.");
  sendTelemetry("RESET");
}

void forceLaunch() {
  uint32_t now = millis();

  launchAlt_m = alt_m;
  altAGL_m = 0.0f;
  maxAltAGL_m = 0.0f;
  mainArmed = false;

  t_launch = now;
  state = FlightState::ASCENT_LOCKOUT;

  startLaunchDetectedSound();
  sendTelemetry("FORCED_LAUNCH");
}

void handleSerialInput() {
  static String cmd;

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      cmd.trim();

      if (cmd.length() > 0) {
        String upper = cmd;
        upper.toUpperCase();

        if (handleLoggerCommand(upper)) {
          cmd = "";
          return;
        }

        if (upper == "HELP") {
          printHelp();
          printLoggerHelp();
        } else if (upper == "STATUS") {
          sendTelemetry("STATUS");
        } else if (upper == "BEEP") {
          beepFreqBlocking(BUZZER_ALERT_HZ, 45, 20);
          Serial.println("OK BEEP");
        } else if (upper == "DROGUE") {
          deployDrogue("SERIAL_TEST_DROGUE", true);
        } else if (upper == "MAIN") {
          deployMain("SERIAL_TEST_MAIN", true);
        } else if (upper == "LAUNCH") {
          forceLaunch();
        } else if (upper == "RESET") {
          resetFlightState();
        } else {
          Serial.print("Unknown command: ");
          Serial.println(cmd);
          printHelp();
          printLoggerHelp();
        }
      }

      cmd = "";
    } else {
      cmd += c;
    }
  }
}

// ============================================================================
// Setup
// ============================================================================

void setup() {
  pinMode(MAIN_PIN, OUTPUT);
  pinMode(DROGUE_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(MODE_SWITCH_PIN, INPUT);

  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);

  pinMode(BATTERY_PIN, INPUT);

  digitalWrite(MAIN_PIN, LOW);
  digitalWrite(DROGUE_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  setRGB(0, 0, 0);

  analogReadResolution(12);

  Serial.begin(115200);

  uint32_t serialStart = millis();
  while (!Serial && (millis() - serialStart < 1500)) {
    delay(10);
  }

  Serial1.setTX(RFD_TX_PIN);
  Serial1.setRX(RFD_RX_PIN);
  Serial1.begin(RFD_BAUD);

  delay(500);

  Serial.println();
  Serial.println("================================================");
  Serial.println("COMET - COMPACT ONBOARD MANAGEMENT FOR EJECTION TIMING");
  Serial.println("RP2040 SERIAL + BINARY LOGGER SYSTEM - BOOT BASELINE ONLY");
  Serial.println("================================================");

  Serial.print("PYRO_OUTPUTS_ENABLED: ");
  Serial.println(PYRO_OUTPUTS_ENABLED ? "TRUE" : "FALSE");

  Serial.print("RFD900x UART baud: ");
  Serial.println(RFD_BAUD);

  Serial.println("Power stabilizing...");
  delay(POWER_STAB_MS);

  playStartupMelody();

  Wire.begin();
  Wire.setClock(400000);
  delay(200);

  scanI2C();

  Serial.println("Checking MPL3115A2...");
  mplOK = mpl.begin();

  Serial.print("MPL3115A2 status: ");
  Serial.println(mplOK ? "OK" : "FAIL");

  if (mplOK) {
    Serial.println("Configuring MPL3115A2 fast altimeter mode...");
    if (!configureMPLFastAltimeter()) {
      Serial.println("MPL3115A2 fast configuration failed; deployment logic disabled.");
      mplOK = false;
    } else {
      delay(150);

      float sumAlt = 0.0f;
      float lastTemp = 0.0f;
      int samples = 0;

      for (int i = 0; i < 10; i++) {
        float a = 0.0f;
        float t = 0.0f;
        if (readMPLFast(a, t)) {
          sumAlt += a;
          lastTemp = t;
          samples++;
        }
        delay(20);
      }

      if (samples > 0) {
        altRef_m = sumAlt / samples;
        mplTemp_C = lastTemp;
      } else {
        Serial.println("MPL3115A2 baseline read failed; deployment logic disabled.");
        mplOK = false;
      }

      Serial.print("Altitude reference: ");
      Serial.print(altRef_m, 3);
      Serial.println(" m");
    }
  }

  Serial.println("Checking LSM6DSO32 at 0x6B...");

  uint8_t whoami = readRegister(IMU_ADDR, 0x0F);

  Serial.print("IMU WHO_AM_I: 0x");
  if (whoami < 16) Serial.print("0");
  Serial.println(whoami, HEX);

  imuOK = imu.begin_I2C(IMU_ADDR);

  Serial.print("LSM6DSO32 status: ");
  Serial.println(imuOK ? "OK" : "FAIL");

  if (imuOK) {
    imu.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);
    imu.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
    imu.setGyroDataRate(LSM6DS_RATE_104_HZ);

    delay(100);
    calibrateAccelBaselineOnBoot();
  }

  if (!mplOK) {
    state = FlightState::FAULT;
    setRGB(1, 0, 0);
    playSensorFailureSound();

    Serial.println("FAULT: Barometer failed. Deployment logic disabled.");
  } else if (mplOK && imuOK) {
    // Default to the lowest main deployment altitude until saved settings load.
    // VIOLET = 200 m.
    setMainAltitudeMode(6, true, false);
    playSensorOkSound();
    Serial.println("Sensors OK. Full launch detection enabled.");
  } else if (mplOK && !imuOK) {
    // Keep the main deployment default conservative even if IMU failed.
    // VIOLET = 200 m.
    setMainAltitudeMode(6, true, false);
    playSensorFailureSound();
    Serial.println("Warning: IMU failed. Baro-only launch detection available.");
  }

  // MODE is no longer used to enter download mode at boot.
  // If the logger is full, COMET will blink yellow and the user can hold
  // MODE for 5 seconds to clear the oldest saved flight slot.
  downloadMode = false;

  if (beginFlightLogger()) {
    if (state != FlightState::FAULT) {
      if (!loadCometSettings()) {
        Serial.println("No saved altitude mode found. Saving current default mode.");
        saveCometSettings();
      }

      startNewLogIfSafe();
    }
  }

  t_boot = millis();
  baselineStartMs = t_boot;

  if (state != FlightState::FAULT) {
    state = FlightState::IDLE;
    Serial.println("System Ready - State: IDLE");
  }

  printHelp();
  printLoggerHelp();

  sendTelemetry("BOOT_READY");
}

// ============================================================================
// Main Loop
// ============================================================================

void loop() {
  const uint32_t now = millis();

  serviceBuzzer(now);
  servicePyro(now);
  serviceLoggerFullUI(now);
  handleModeSwitch();

  if (TEST_MODE) {
    handleSerialInput();
  }

  if (state == FlightState::FAULT) {
    digitalWrite(MAIN_PIN, LOW);
    digitalWrite(DROGUE_PIN, LOW);
    delay(50);
    return;
  }

  readSensors();
  serviceFlightLogger(now);

  if (state == FlightState::IDLE) {
    updateAccelBaseline(now);
  }

  const bool drogueTimerElapsed = (t_launch > 0) && (now - t_launch >= drogueBackupMs);
  const bool mainTimerElapsed   = (t_launch > 0) && (now - t_launch >= mainBackupMs);

  switch (state) {
    case FlightState::IDLE: {
      bool launchAccel = launchDetectAccel(now);

      // Do NOT auto-launch from barometric vertical speed.
      // The MPL3115A2 can occasionally produce a small altitude step while sitting
      // still, which creates a fake positive Vz spike. Launch detection is now
      // accelerometer-only; use the LAUNCH serial command for bench testing.
      if (launchAccel) {
        launchAlt_m = alt_m;
        altAGL_m = 0.0f;
        maxAltAGL_m = 0.0f;
        mainArmed = false;

        // Reset Vz estimator at launch so pre-launch baro noise does not carry
        // into the flight state machine.
        ringHead = 0;
        ringCount = 0;
        vz_mps = 0.0f;

        t_launch = now;
        state = FlightState::ASCENT_LOCKOUT;

        startLaunchDetectedSound();
        sendTelemetry("LAUNCH_ACCEL");
      }

      break;
    }

    case FlightState::ASCENT_LOCKOUT: {
      if (now - t_launch >= flightLockoutMs) {
        state = FlightState::ASCENT;
        sendTelemetry("ASCENT_LOCKOUT_END");
      }

      break;
    }

    case FlightState::ASCENT: {
      if (negativeTrend(APOGEE_NEG_VZ_DWELL_MS)) {
        state = FlightState::APOGEE_DETECT;
        sendTelemetry("APOGEE_NEG_TREND");
      }

      if (!drogueFired &&
          drogueTimerElapsed &&
          (now - t_launch >= flightLockoutMs)) {
        deployDrogue("TIMER_BACKUP_DROGUE", true);
        state = FlightState::DROGUE_DEPLOYED;
      }

      break;
    }

    case FlightState::APOGEE_DETECT: {
      if (!drogueFired &&
          vz_mps <= 0.0f &&
          (now - t_launch >= flightLockoutMs)) {
        deployDrogue("APOGEE", false);
        state = FlightState::DROGUE_DEPLOYED;
      }

      if (!drogueFired &&
          drogueTimerElapsed &&
          (now - t_launch >= flightLockoutMs)) {
        deployDrogue("TIMER_BACKUP_DROGUE", true);
        state = FlightState::DROGUE_DEPLOYED;
      }

      break;
    }

    case FlightState::DROGUE_DEPLOYED: {
      static uint32_t downFastStart = 0;

      if (vz_mps <= MAX_FALL_SPEED_FOR_DROGUE) {
        if (downFastStart == 0) downFastStart = now;

        if (!mainFired &&
            (now - downFastStart >= DROGUE_INEFFECTIVE_MS) &&
            (now - t_launch >= flightLockoutMs)) {
          deployMain("DROGUE_INEFFECTIVE_FALLRATE", true);
          state = FlightState::MAIN_DEPLOYED;
          break;
        }
      } else {
        downFastStart = 0;
      }

      if (!mainFired &&
          mainArmed &&
          altAGL_m <= mainAltM &&
          vz_mps <= 0.0f &&
          (now - t_launch >= flightLockoutMs)) {
        deployMain("MAIN_AGL_CROSS", true);
        state = FlightState::MAIN_DEPLOYED;
        break;
      }

      if (!mainFired &&
          drogueFired &&
          mainTimerElapsed &&
          (now - t_launch >= flightLockoutMs)) {
        deployMain("TIMER_BACKUP_MAIN_AFTER_DROGUE", true);
        state = FlightState::MAIN_DEPLOYED;
        break;
      }

      break;
    }

    case FlightState::MAIN_DEPLOYED:
    case FlightState::LANDED:
    case FlightState::BOOT:
    case FlightState::FAULT:
    default:
      break;
  }

  static uint32_t lastTelemetry = 0;

  if (now - lastTelemetry >= TELEMETRY_PERIOD_MS) {
    sendTelemetry(nullptr);

    // Maintain a stable cadence instead of drifting later every loop.
    lastTelemetry += TELEMETRY_PERIOD_MS;

    // If something caused a long delay, resync without trying to burst packets.
    if (now - lastTelemetry > TELEMETRY_PERIOD_MS * 2) {
      lastTelemetry = now;
    }
  }

  if (!pyroActive()) {
    digitalWrite(MAIN_PIN, LOW);
    digitalWrite(DROGUE_PIN, LOW);
  }
}
