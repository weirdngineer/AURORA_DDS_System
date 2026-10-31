#include <Arduino.h>
#include "SimpleRecoveryLogic.h"
#include "OptionalBackupTimers.h"
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
//   - Up to 20 rolling flight logs, plus last 3 static power-session logs
//   - 10 Hz flight logging, with 10 seconds of prelaunch RAM records
//   - 15 seconds stable landing detection; auto-recycle oldest completed flight
//   - 80% of LittleFS partition used as storage budget
//
// IMPORTANT:
//   Use Earle Philhower Arduino-Pico core.
//   Choose a board/filesystem layout with enough LittleFS space.
// ============================================================================
// MERGED FIRING-STABILITY BUILD - 2026-08-13
//
// Basis:
//   - Retains the newest COMET features from the 2945-line branch.
//   - Restores the proven deployment-buzzer behavior from the known-good
//     2525-line branch: 1 s rapid 4000 Hz beep before each pyro pulse.
//   - Leaves the known-good 300 ms non-blocking pyro sequencer intact.
//   - Defers settings-file writes until safe preflight IDLE conditions.
//   - Blocks non-flight-critical filesystem/UI actions during flight.
//   - Gives servicePyro() first priority in loop().
//
// Bench-test BOTH channels repeatedly with representative igniter loads before
// relying on this build in flight.
// ============================================================================

// ============================================================================
// Safety / Testing
// ============================================================================

constexpr bool PYRO_OUTPUTS_ENABLED = false;  // DEVELOPMENT / BENCH ONLY. Do not enable without validation.
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
constexpr float R_TOP = 330000.0f;  // VERIFY this matches the physically populated divider resistor
constexpr float R_BOTTOM = 100000.0f;
constexpr float BATTERY_DIVIDER_RATIO = ((R_TOP + R_BOTTOM) / R_BOTTOM);

// ============================================================================
// Flight Parameters
// ============================================================================
// Runtime-editable parameters are lower-case variables so the Python GUI can
// update them with SET <PARAM> <VALUE>. They default to these values on first
// boot, then persist to LittleFS after SET commands.

float     mainAltM                 = 200.0f;  // Active configured main altitude; hard floor in SimpleRecoveryLogic.h
float     mainArmMarginM           = 20.0f;

// Runtime-adjustable apogee vertical-speed threshold. Negative means descending.
float     apogeeNegVzThreshMps      = -1.5f;

// Detection-source enables. All are ON by default.
// BARO controls barometric apogee/main logic.
// ACCEL controls automatic accelerometer launch detection.
// TIMER controls drogue/main timer backups.
bool      baroMethodsEnabled        = true;
bool      accelMethodsEnabled       = true;
bool      timerMethodsEnabled       = false;

constexpr uint32_t  POWER_STAB_MS  = 2000;
uint32_t  flightLockoutMs          = 10000;
uint32_t  drogueBackupMs           = 15000;
uint32_t  mainBackupMs             = 25000;

constexpr float     ASCENT_POS_VZ_THRESH       = 2.0f;
constexpr uint32_t  ASCENT_POS_VZ_DWELL_MS     = 250;

constexpr uint32_t  APOGEE_NEG_VZ_DWELL_MS     = 250;

constexpr float     LAUNCH_ACCEL_DELTA_G       = 1.2f;
constexpr uint32_t  LAUNCH_ACCEL_DWELL_MS      = 100;
constexpr uint32_t  BASELINE_MIN_MS            = 1000;

constexpr float     MAX_FALL_SPEED_FOR_DROGUE  = -100.0f; // RETAINED from newest branch; verify flight-safety intent separately
constexpr uint32_t  DROGUE_INEFFECTIVE_MS      = 1000; // RETAINED from newest branch; verify flight-safety intent separately

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
// Only IDLE, ASCENT, DROGUE_DEPLOYED, MAIN_DEPLOYED are used in automatic recovery.
comet_recovery::Logic recoveryLogic;
bool pendingDrogueRequest = false;
bool pendingMainRequest = false;
const char* pendingDrogueReason = "APOGEE";
const char* pendingMainReason = "MAIN_AGL_CROSS";
uint32_t latestBaroReadMs = 0;
uint32_t latestImuReadMs = 0;
uint32_t processedBaroReadMs = 0;
uint32_t processedImuReadMs = 0;

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
  LOG_EVENT_LAUNCH_BARO = 15,
  LOG_EVENT_APOGEE_DROP = 16,
  LOG_EVENT_APOGEE_REVERSAL = 17,
  LOG_EVENT_MAIN_HARD_FLOOR = 18,
  LOG_EVENT_LANDED_STABLE = 19,
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

// Forward declarations for regular C++ and Arduino-Pico builds.
bool pyroActive();
bool startNewLogIfSafe();

// ============================================================================
// Dual-category rolling logger
// 100 x 32-byte RAM records (10 s at 10 Hz) before launch.
// 20 bounded flight archive entries, evict oldest COMPLETE as required.
// Three capped per-power-cycle static ring files at 1 Hz (last 30 minutes).
// 80% cap applies to the *LittleFS partition*, not the physical boot flash.
// ============================================================================
constexpr bool LOGGING_ENABLED = true;
constexpr uint32_t LOG_RATE_HZ = 10;
constexpr uint32_t LOG_PERIOD_MS = 100;
constexpr uint16_t PRELAUNCH_RECORDS = 100;
constexpr uint8_t LOG_SLOT_COUNT = 20;
constexpr uint8_t STATIC_SLOT_COUNT = 3;
constexpr uint16_t STATIC_MAX_RECORDS = 1800;
constexpr uint32_t STATIC_LOG_PERIOD_MS = 1000;
constexpr uint32_t LOG_DATA_FLUSH_RECORDS = 300;
constexpr uint32_t LOG_MAGIC = 0x41555241UL;
constexpr uint16_t LOG_VERSION = 2;
constexpr uint8_t LOG_KIND_FLIGHT = 1;
constexpr uint8_t LOG_KIND_STATIC = 2;
constexpr uint32_t LANDING_STABLE_MS = 15000UL;
constexpr float LANDING_MAX_ALT_DRIFT_M = 2.0f;
constexpr float LANDING_ACCEL_TOL_G = 0.25f;
constexpr float LANDING_GYRO_MAX_DPS = 15.0f;

const char* const LOG_SLOT_FILES[LOG_SLOT_COUNT] = {
  "/flight00.bin", "/flight01.bin", "/flight02.bin", "/flight03.bin", "/flight04.bin",
  "/flight05.bin", "/flight06.bin", "/flight07.bin", "/flight08.bin", "/flight09.bin",
  "/flight10.bin", "/flight11.bin", "/flight12.bin", "/flight13.bin", "/flight14.bin",
  "/flight15.bin", "/flight16.bin", "/flight17.bin", "/flight18.bin", "/flight19.bin"
};
const char* const STATIC_SLOT_FILES[STATIC_SLOT_COUNT] = {
  "/static0.bin", "/static1.bin", "/static2.bin"
};

// ============================================================================
// Persistent COMET Settings
// ============================================================================
// These settings live in LittleFS next to the flight logs. They are only
// rewritten when the user changes the selected altitude mode, not continuously
// during flight.

const char* COMET_SETTINGS_FILE = "/comet_settings.bin";
constexpr uint32_t SETTINGS_MAGIC   = 0x434D4554UL;  // "CMET"
constexpr uint16_t SETTINGS_VERSION = 4;

// Settings writes are intentionally deferred. The known-good firing branch did
// not perform synchronous settings-file writes from MODE/SET interactions.
bool settingsDirty = false;
uint32_t settingsDirtySinceMs = 0;
constexpr uint32_t SETTINGS_SAVE_SETTLE_MS = 500;

// Each ROYGBIV color is now a complete flight profile rather than only a main
// altitude preset. Values remain stored internally in SI units and milliseconds.
// Persistent profile layout remains binary-compatible with v3. Timer enable and
// drogue/main backup delays are active again; obsolete apogee/lockout fields
// remain stored only for legacy profile compatibility. Old v3 files are read
// with all timers forced OFF, requiring an explicit re-enable through the GUI.
struct __attribute__((packed)) FlightProfile {
  float mainAltM;
  float mainArmMarginM;
  float apogeeNegVzThreshMps;

  uint32_t flightLockoutMs;
  uint32_t drogueBackupMs;
  uint32_t mainBackupMs;

  uint8_t baroEnabled;
  uint8_t accelEnabled;
  uint8_t timerEnabled;
  uint8_t reserved;
};

FlightProfile flightProfiles[7] = {
  {800.0f, 20.0f, -1.5f, 10000, 15000, 25000, 1, 1, 0, 0}, // RED
  {700.0f, 20.0f, -1.5f, 10000, 15000, 25000, 1, 1, 0, 0}, // ORANGE
  {600.0f, 20.0f, -1.5f, 10000, 15000, 25000, 1, 1, 0, 0}, // YELLOW
  {500.0f, 20.0f, -1.5f, 10000, 15000, 25000, 1, 1, 0, 0}, // GREEN
  {400.0f, 20.0f, -1.5f, 10000, 15000, 25000, 1, 1, 0, 0}, // BLUE
  {300.0f, 20.0f, -1.5f, 10000, 15000, 25000, 1, 1, 0, 0}, // INDIGO
  {200.0f, 20.0f, -1.5f, 10000, 15000, 25000, 1, 1, 0, 0}  // VIOLET
};

struct __attribute__((packed)) CometSettings {
  uint32_t magic;
  uint16_t version;

  uint8_t  mainAltMode;
  uint8_t  reserved0;

  FlightProfile profiles[7];

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
File staticFile;
FlightLogHeader activeHeader = {};
FlightLogHeader activeStaticHeader = {};
bool loggerMounted = false;
bool loggerActive = false;  // Flight recording active
bool staticLoggerActive = false;
bool loggerFull = false;
bool downloadMode = false;
int8_t activeSlot = -1;
int8_t activeStaticSlot = -1;
uint32_t lastLogMs = 0;
uint32_t lastStaticLogMs = 0;
uint32_t nextFlightNumber = 1;
uint32_t nextStaticNumber = 1;
uint16_t staticWriteIndex = 0;
FlightLogRecord prelaunchRing[PRELAUNCH_RECORDS];
FlightLogRecord pyroDeferredRecords[16];
uint8_t pyroDeferredCount = 0;
uint16_t prelaunchHead = 0;
uint16_t prelaunchCount = 0;
uint32_t landingStableStartMs = 0;
float landingStableReferenceAlt = 0.0f;
uint32_t baroLaunchStartMs = 0;
uint32_t baroApogeeStartMs = 0;
uint32_t lastBaroSuccessMs = 0;
constexpr uint32_t FULL_SLOT_HOLD_CLEAR_MS = 5000;
constexpr uint32_t FULL_YELLOW_BLINK_MS = 250;
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
  if (strcmp(eventName, "LAUNCH_BARO") == 0) return LOG_EVENT_LAUNCH_BARO;
  if (strcmp(eventName, "APOGEE_DROP") == 0) return LOG_EVENT_APOGEE_DROP;
  if (strcmp(eventName, "APOGEE_REVERSAL") == 0) return LOG_EVENT_APOGEE_REVERSAL;
  if (strcmp(eventName, "MAIN_HARD_FLOOR") == 0) return LOG_EVENT_MAIN_HARD_FLOOR;
  if (strcmp(eventName, "LANDED_STABLE") == 0) return LOG_EVENT_LANDED_STABLE;
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
    case LOG_EVENT_LAUNCH_BARO:             return "LAUNCH detected by 30m barometric rise";
    case LOG_EVENT_APOGEE_DROP:            return "APOGEE confirmed by sustained altitude drop";
    case LOG_EVENT_APOGEE_REVERSAL:        return "APOGEE confirmed by ascent-to-descent trend reversal";
    case LOG_EVENT_MAIN_HARD_FLOOR:         return "MAIN commanded by fixed 200m descent backstop";
    case LOG_EVENT_LANDED_STABLE:           return "Flight log ended after 15 seconds of stability";
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
bool settingsWriteAllowed();
void markSettingsDirty();
void serviceDeferredSettingsSave(uint32_t now);

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

void applyFlightProfile(uint8_t mode) {
  if (mode >= MAIN_ALT_MODE_COUNT) return;

  const FlightProfile& p = flightProfiles[mode];
  mainAltM = comet_recovery::Logic::target(p.mainAltM);
  mainArmMarginM = p.mainArmMarginM;
  apogeeNegVzThreshMps = p.apogeeNegVzThreshMps;

  flightLockoutMs = p.flightLockoutMs;
  drogueBackupMs = p.drogueBackupMs;
  mainBackupMs = p.mainBackupMs;

  baroMethodsEnabled = true; // Mandatory: main and apogee require barometer; legacy setting ignored.
  accelMethodsEnabled = p.accelEnabled != 0;
  timerMethodsEnabled = p.timerEnabled != 0; // Enabled only by explicit GUI/serial setting.
}

void captureRuntimeIntoActiveProfile() {
  if (rgbMode < 0 || rgbMode >= MAIN_ALT_MODE_COUNT) return;

  FlightProfile& p = flightProfiles[rgbMode];
  p.mainAltM = comet_recovery::Logic::target(mainAltM);
  p.mainArmMarginM = mainArmMarginM;
  p.apogeeNegVzThreshMps = apogeeNegVzThreshMps;

  p.flightLockoutMs = flightLockoutMs;
  p.drogueBackupMs = drogueBackupMs;
  p.mainBackupMs = mainBackupMs;

  p.baroEnabled = baroMethodsEnabled ? 1 : 0;
  p.accelEnabled = accelMethodsEnabled ? 1 : 0;
  p.timerEnabled = timerMethodsEnabled ? 1 : 0;
}

void setMainAltitudeMode(uint8_t mode, bool announce = true, bool saveToFs = false) {
  if (mode >= MAIN_ALT_MODE_COUNT) {
    mode = 0;
  }

  rgbMode = mode;

  const MainAltitudeMode& m = MAIN_ALT_MODES[rgbMode];
  applyFlightProfile((uint8_t)rgbMode);
  setRGBLevel(m.r, m.g, m.b);

  if (announce) {
    Serial.print("FLIGHT PROFILE: ");
    Serial.print(rgbMode);
    Serial.print(" ");
    Serial.print(m.name);
    Serial.print(" MAIN_ALT=");
    Serial.print(mainAltM, 1);
    Serial.print(" m legacy_VZ_NEG=");
    Serial.print(apogeeNegVzThreshMps, 2);
    Serial.print(" m/s BARO=");
    Serial.print(baroMethodsEnabled ? 1 : 0);
    Serial.print(" ACCEL=");
    Serial.print(accelMethodsEnabled ? 1 : 0);
    Serial.print(" TIMER=");
    Serial.println(timerMethodsEnabled ? 1 : 0);
  }

  if (saveToFs) {
    // Never perform a synchronous LittleFS settings write from a UI handler.
    // Queue it for safe preflight IDLE time instead.
    markSettingsDirty();
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

void startRapidBeepHz(uint16_t seconds, uint16_t freqHz) {
  uint32_t now = millis();

  buzMode = BuzzerMode::RAPID;
  buzState = true;
  buzFreqHz = freqHz;

  tone(BUZZER_PIN, buzFreqHz);

  buzNextToggle = now + BEEP_ON_MS;
  buzStopAt = now + (uint32_t)seconds * 1000UL;
}

void startRapidBeep(uint16_t seconds) {
  // Newer COMET default alert sound.
  startRapidBeepHz(seconds, BUZZER_ALERT_HZ);
}

void startPyroPreFireBeep() {
  // IMPORTANT: preserve the proven deployment buzzer behavior from the
  // known-good branch: 1 second rapid beep at 4000 Hz.
  startRapidBeepHz(1, 4000);
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
  // Also lock the mode UI once flight has begun and during a pyro pulse.
  if (loggerFull || pyroActive() || t_launch > 0) return;

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

bool profileValid(const FlightProfile& p) {
  if (!finitef_safe(p.mainAltM) || p.mainAltM < 1.0f || p.mainAltM > 10000.0f) return false;
  if (!finitef_safe(p.mainArmMarginM) || p.mainArmMarginM < 0.0f || p.mainArmMarginM > 1000.0f) return false;
  if (!finitef_safe(p.apogeeNegVzThreshMps) || p.apogeeNegVzThreshMps > 0.0f || p.apogeeNegVzThreshMps < -200.0f) return false;

  if (p.drogueBackupMs < 1000UL || p.drogueBackupMs > 600000UL) return false;
  if (p.mainBackupMs < 1000UL || p.mainBackupMs > 600000UL) return false;
  if (p.flightLockoutMs > 120000UL) return false;

  if (p.baroEnabled > 1 || p.accelEnabled > 1 || p.timerEnabled > 1) return false;
  return true;
}

bool settingsValid(const CometSettings& s) {
  if (s.magic != SETTINGS_MAGIC) return false;
  // v3 had inert timer fields. Accept it for migration but never reactivate
  // old timerEnabled flags; v4 represents an explicit new opt-in.
  if (s.version != SETTINGS_VERSION && s.version != 3) return false;
  if (s.mainAltMode >= MAIN_ALT_MODE_COUNT) return false;

  for (uint8_t i = 0; i < MAIN_ALT_MODE_COUNT; i++) {
    if (!profileValid(s.profiles[i])) return false;
  }

  return s.crc == settingsCrcCalc(s);
}

void printCometSettingsSummary(const char* prefix) {
  Serial.print(prefix ? prefix : "SETTINGS");
  Serial.print(": profile=");
  Serial.print(rgbMode);

  if (rgbMode >= 0 && rgbMode < MAIN_ALT_MODE_COUNT) {
    Serial.print(" ");
    Serial.print(MAIN_ALT_MODES[rgbMode].name);
  }

  Serial.print(" mainAltM=");
  Serial.print(mainAltM, 2);
  Serial.print(" mainArmMarginM=");
  Serial.print(mainArmMarginM, 2);
  Serial.print(" legacyApogeeVzNeg=");
  Serial.print(apogeeNegVzThreshMps, 2);
  Serial.print(" drogueBackupMs=");
  Serial.print((unsigned long)drogueBackupMs);
  Serial.print(" mainBackupMs=");
  Serial.print((unsigned long)mainBackupMs);
  Serial.print(" legacyLockoutMs=");
  Serial.print((unsigned long)flightLockoutMs);
  Serial.print(" baro=");
  Serial.print(baroMethodsEnabled ? 1 : 0);
  Serial.print(" accel=");
  Serial.print(accelMethodsEnabled ? 1 : 0);
  Serial.print(" timer=");
  Serial.println(timerMethodsEnabled ? 1 : 0);
}

bool saveCometSettings() {
  if (!loggerMounted) {
    Serial.println("SETTINGS SAVE FAILED: LittleFS not mounted.");
    return false;
  }

  // Make sure the currently active profile contains any direct runtime edits.
  captureRuntimeIntoActiveProfile();

  CometSettings s;
  memset(&s, 0, sizeof(s));

  s.magic = SETTINGS_MAGIC;
  s.version = SETTINGS_VERSION;
  s.mainAltMode = (uint8_t)rgbMode;

  for (uint8_t i = 0; i < MAIN_ALT_MODE_COUNT; i++) {
    s.profiles[i] = flightProfiles[i];
  }

  s.crc = settingsCrcCalc(s);

  File f = LittleFS.open(COMET_SETTINGS_FILE, "w");
  if (!f) {
    Serial.println("SETTINGS SAVE FAILED: could not open settings file.");
    return false;
  }

  size_t n = f.write((const uint8_t*)&s, sizeof(s));
  f.flush();
  f.close();

  bool ok = (n == sizeof(s));
  if (ok) {
    printCometSettingsSummary("SETTINGS SAVED");
  } else {
    Serial.println("SETTINGS SAVE FAILED: short write.");
  }

  return ok;
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
    Serial.println("SETTINGS INVALID/OLD: using default profiles and rewriting settings file.");
    return false;
  }

  const bool migratedFromV3 = s.version == 3;
  for (uint8_t i = 0; i < MAIN_ALT_MODE_COUNT; i++) {
    flightProfiles[i] = s.profiles[i];
    if (migratedFromV3) flightProfiles[i].timerEnabled = 0;
  }

  setMainAltitudeMode(s.mainAltMode, true, false);
  if (migratedFromV3) {
    Serial.println("SETTINGS: v3 profile migration; ALL timer backups forced OFF.");
    markSettingsDirty(); // Save v4 in safe IDLE, not during initial setup.
  }

  printCometSettingsSummary("SETTINGS LOADED");
  return true;
}

bool settingsWriteAllowed() {
  // Settings persistence is a preflight-only maintenance action.
  return loggerMounted &&
         !pyroActive() &&
         t_launch == 0 &&
         state == FlightState::IDLE;
}

void markSettingsDirty() {
  settingsDirty = true;
  settingsDirtySinceMs = millis();
}

void serviceDeferredSettingsSave(uint32_t now) {
  if (!settingsDirty) return;
  if (!settingsWriteAllowed()) return;

  // Coalesce rapid MODE-button/GUI changes into one flash write.
  if (now - settingsDirtySinceMs < SETTINGS_SAVE_SETTLE_MS) return;

  if (saveCometSettings()) {
    settingsDirty = false;
    Serial.println("SETTINGS: deferred save complete.");
  } else {
    // Leave dirty=true so a later safe pass can retry.
    settingsDirtySinceMs = now;
    Serial.println("SETTINGS: deferred save failed; will retry while safe.");
  }
}

uint32_t headerCrcCalc(FlightLogHeader h) {
  h.headerCrc = 0;
  return checksum32((const uint8_t*)&h, sizeof(h));
}

const char* logPath(uint8_t kind, uint8_t slot) {
  if (kind == LOG_KIND_FLIGHT && slot < LOG_SLOT_COUNT) return LOG_SLOT_FILES[slot];
  if (kind == LOG_KIND_STATIC && slot < STATIC_SLOT_COUNT) return STATIC_SLOT_FILES[slot];
  return nullptr;
}

bool headerValid(const FlightLogHeader& h, uint8_t kind, uint8_t slot) {
  return h.magic == LOG_MAGIC && h.version == LOG_VERSION &&
         h.headerSize == sizeof(FlightLogHeader) &&
         h.recordSize == sizeof(FlightLogRecord) &&
         h.reserved[0] == kind && h.slotIndex == slot &&
         h.headerCrc == headerCrcCalc(h);
}

bool readStoredHeader(uint8_t kind, uint8_t slot, FlightLogHeader& h) {
  const char* path = logPath(kind, slot);
  if (!path || !LittleFS.exists(path)) return false;
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  bool ok = f.size() >= sizeof(h) && f.read((uint8_t*)&h, sizeof(h)) == sizeof(h);
  f.close();
  return ok && headerValid(h, kind, slot);
}

bool writeStoredHeader(uint8_t kind, uint8_t slot, FlightLogHeader& h) {
  const char* path = logPath(kind, slot);
  if (!path) return false;
  h.magic = LOG_MAGIC;
  h.version = LOG_VERSION;
  h.slotIndex = slot;
  h.recordSize = sizeof(FlightLogRecord);
  h.headerSize = sizeof(FlightLogHeader);
  h.maxRecords = (kind == LOG_KIND_STATIC) ? STATIC_MAX_RECORDS : 0xFFFFFFFFUL;
  h.reserved[0] = kind;
  h.headerCrc = headerCrcCalc(h);
  File f = LittleFS.open(path, LittleFS.exists(path) ? "r+" : "w+");
  if (!f) return false;
  f.seek(0);
  bool ok = f.write((const uint8_t*)&h, sizeof(h)) == sizeof(h);
  f.flush();
  f.close();
  return ok;
}

uint32_t logFileSize(uint8_t kind, uint8_t slot) {
  const char* path = logPath(kind, slot);
  if (!path) return 0;
  File f = LittleFS.open(path, "r");
  if (!f) return 0;
  uint32_t size = f.size();
  f.close();
  return size;
}

void recoverLogCategory(uint8_t kind) {
  uint8_t slots = (kind == LOG_KIND_FLIGHT) ? LOG_SLOT_COUNT : STATIC_SLOT_COUNT;
  for (uint8_t i = 0; i < slots; ++i) {
    FlightLogHeader h = {};
    if (!readStoredHeader(kind, i, h)) continue;
    if (h.status == SLOT_RECORDING) {
      uint32_t size = logFileSize(kind, i);
      uint32_t possible = size >= sizeof(FlightLogHeader) ?
        (size - sizeof(FlightLogHeader)) / sizeof(FlightLogRecord) : 0;
      if (kind == LOG_KIND_STATIC && possible > STATIC_MAX_RECORDS) possible = STATIC_MAX_RECORDS;
      h.recordCount = possible;
      h.status = SLOT_COMPLETE;
      h.completedMillis = 0;  // Previous power-cycle clock unavailable.
      writeStoredHeader(kind, i, h);
    }
    uint32_t& next = (kind == LOG_KIND_FLIGHT) ? nextFlightNumber : nextStaticNumber;
    if (h.flightNumber >= next) next = h.flightNumber + 1;
  }
}

int8_t oldestStoredSlot(uint8_t kind) {
  int8_t selected = -1;
  uint32_t oldest = 0xFFFFFFFFUL;
  uint8_t slots = (kind == LOG_KIND_FLIGHT) ? LOG_SLOT_COUNT : STATIC_SLOT_COUNT;
  for (uint8_t i = 0; i < slots; ++i) {
    if ((kind == LOG_KIND_FLIGHT && loggerActive && activeSlot == (int8_t)i) ||
        (kind == LOG_KIND_STATIC && staticLoggerActive && activeStaticSlot == (int8_t)i)) continue;
    FlightLogHeader h = {};
    if (readStoredHeader(kind, i, h) &&
        (h.status == SLOT_COMPLETE || h.status == SLOT_DOWNLOADED) &&
        h.flightNumber < oldest) {
      selected = i;
      oldest = h.flightNumber;
    }
  }
  return selected;
}

int8_t latestStoredSlot(uint8_t kind) {
  int8_t selected = -1;
  uint32_t newest = 0;
  uint8_t slots = (kind == LOG_KIND_FLIGHT) ? LOG_SLOT_COUNT : STATIC_SLOT_COUNT;
  for (uint8_t i = 0; i < slots; ++i) {
    FlightLogHeader h = {};
    if (readStoredHeader(kind, i, h) && h.status != SLOT_EMPTY &&
        (selected < 0 || h.flightNumber > newest)) {
      selected = i;
      newest = h.flightNumber;
    }
  }
  return selected;
}

int8_t selectWriteSlot(uint8_t kind) {
  uint8_t slots = (kind == LOG_KIND_FLIGHT) ? LOG_SLOT_COUNT : STATIC_SLOT_COUNT;
  for (uint8_t i = 0; i < slots; ++i) {
    FlightLogHeader h = {};
    if (!LittleFS.exists(logPath(kind, i))) return i;
    if (readStoredHeader(kind, i, h) && h.status == SLOT_EMPTY) return i;
    // Preserve files with corrupt/unknown headers for manual recovery, not overwrite.
  }
  // Once all named slots are used, automatically recycle the OLDEST.
  return oldestStoredSlot(kind);
}

bool eraseOneLog(uint8_t kind, uint8_t slot) {
  const char* path = logPath(kind, slot);
  if (!path) return false;
  if (kind == LOG_KIND_FLIGHT && loggerActive && activeSlot == (int8_t)slot) return false;
  if (kind == LOG_KIND_STATIC && staticLoggerActive && activeStaticSlot == (int8_t)slot) return false;
  return !LittleFS.exists(path) || LittleFS.remove(path);
}

void listLogSlots(Stream& out = Serial) {
  int8_t lastFlight = latestStoredSlot(LOG_KIND_FLIGHT);
  int8_t lastStatic = latestStoredSlot(LOG_KIND_STATIC);
  out.println("\n===== COMET LOG ARCHIVE =====");
  out.println("FLIGHTS (max 20; newest marked with *)");
  for (uint8_t i = 0; i < LOG_SLOT_COUNT; ++i) {
    FlightLogHeader h = {};
    if (!readStoredHeader(LOG_KIND_FLIGHT, i, h)) continue;
    if (loggerActive && activeSlot == (int8_t)i) h = activeHeader;
    out.print(i == (uint8_t)lastFlight ? "* " : "  ");
    out.print("FLIGHT SLOT "); out.print(i);
    out.print(" ID="); out.print(h.flightNumber);
    out.print(" RECORDS="); out.print(h.recordCount);
    out.print(" SECONDS="); out.print(h.recordCount / LOG_RATE_HZ);
    out.print(" STATUS="); out.print(h.status);
    out.print(" FILE="); out.println(logPath(LOG_KIND_FLIGHT, i));
  }
  out.println("STATIC (last 3 non-flight power sessions, 1 Hz, last 30 min each)");
  for (uint8_t i = 0; i < STATIC_SLOT_COUNT; ++i) {
    FlightLogHeader h = {};
    if (!readStoredHeader(LOG_KIND_STATIC, i, h)) continue;
    if (staticLoggerActive && activeStaticSlot == (int8_t)i) h = activeStaticHeader;
    out.print(i == (uint8_t)lastStatic ? "* " : "  ");
    out.print("STATIC SLOT "); out.print(i);
    out.print(" ID="); out.print(h.flightNumber);
    out.print(" RECORDS="); out.print(h.recordCount);
    out.print(" SECONDS="); out.print(h.recordCount);
    out.print(" STATUS="); out.print(h.status);
    out.print(" FILE="); out.println(logPath(LOG_KIND_STATIC, i));
  }
  out.print("LASTFLIGHT="); out.println(lastFlight);
  out.print("LASTSTATIC="); out.println(lastStatic);
  out.println("* denotes most recent; slot numbers are physical and reused.");
  if (LittleFS.exists("/flight0.bin") || LittleFS.exists("/flight1.bin") ||
      LittleFS.exists("/flight2.bin"))
    out.println("WARNING: Legacy v1 /flight0..2.bin exist. Download them before FORMATLOG.");
  out.println("=============================\n");
}

bool beginFlightLogger() {
  // Arduino-Pico defaults to autoformat-on-mount-failure. Explicitly disable it;
  // an interrupted/corrupted flight archive must not be silently erased.
  LittleFSConfig fsConfig;
  fsConfig.setAutoFormat(false);
  LittleFS.setConfig(fsConfig);
  loggerMounted = LittleFS.begin();
  // Never autoformat at boot: a failed mount must not silently destroy flights.
  if (!loggerMounted) {
    Serial.println("LOGGER ERROR: LittleFS mount failed; data NOT formatted.");
    return false;
  }
  recoverLogCategory(LOG_KIND_FLIGHT);
  recoverLogCategory(LOG_KIND_STATIC);
  listLogSlots();
  return true;
}

bool flashBudgetHasRoom(uint32_t reserveBytes) {
  FSInfo info = {};
  if (!LittleFS.info(info) || info.totalBytes == 0) return false;
  uint32_t cap = (uint32_t)((uint64_t)info.totalBytes * 80ULL / 100ULL);
  return info.usedBytes < cap && reserveBytes <= cap - info.usedBytes;
}

bool reclaimSpaceBeforeFlight() {
  // Make a working reserve before launch rather than erasing old files during
  // pyro sequencing. Full retention is bounded by the actual LittleFS size.
  FSInfo info = {};
  if (!LittleFS.info(info) || info.totalBytes == 0) return false;
  // Dynamic reserve: 10% of LittleFS, bounded from 64 to 256 KiB.
  uint32_t reserveBytes = info.totalBytes / 10UL;
  if (reserveBytes < 64UL * 1024UL) reserveBytes = 64UL * 1024UL;
  if (reserveBytes > 256UL * 1024UL) reserveBytes = 256UL * 1024UL;
  for (uint8_t attempt = 0; attempt < LOG_SLOT_COUNT; ++attempt) {
    if (flashBudgetHasRoom(reserveBytes)) return true;
    int8_t oldest = oldestStoredSlot(LOG_KIND_FLIGHT);
    if (oldest < 0 || !eraseOneLog(LOG_KIND_FLIGHT, (uint8_t)oldest)) return false;
    Serial.print("LOG RETENTION: automatically deleted oldest flight slot ");
    Serial.println(oldest);
  }
  return flashBudgetHasRoom(reserveBytes);
}

bool startStaticLogger() {
  if (!loggerMounted || loggerActive || staticLoggerActive || t_launch > 0) return false;
  int8_t slot = selectWriteSlot(LOG_KIND_STATIC);
  if (slot < 0) return false;
  if (!eraseOneLog(LOG_KIND_STATIC, (uint8_t)slot)) return false;
  activeStaticSlot = slot;
  activeStaticHeader = {};
  activeStaticHeader.status = SLOT_RECORDING;
  activeStaticHeader.flightNumber = nextStaticNumber++;
  activeStaticHeader.startMillis = millis();
  if (!writeStoredHeader(LOG_KIND_STATIC, (uint8_t)slot, activeStaticHeader)) return false;
  staticFile = LittleFS.open(STATIC_SLOT_FILES[slot], "r+");
  if (!staticFile) return false;
  staticLoggerActive = true;
  staticWriteIndex = 0;
  lastStaticLogMs = millis();
  return true;
}

void closeStaticLogger(bool discard) {
  if (!staticLoggerActive) return;
  staticFile.flush();
  staticFile.close();
  activeStaticHeader.completedMillis = millis();
  activeStaticHeader.status = SLOT_COMPLETE;
  writeStoredHeader(LOG_KIND_STATIC, (uint8_t)activeStaticSlot, activeStaticHeader);
  staticLoggerActive = false;
  if (discard) eraseOneLog(LOG_KIND_STATIC, (uint8_t)activeStaticSlot);
  activeStaticSlot = -1;
}

bool startNewLogIfSafe() {
  if (!LOGGING_ENABLED || !loggerMounted || loggerActive || downloadMode) return false;
  // Reserve should have been established during IDLE, before launch.
  // A minimum 4KiB of free budget is needed to start a valid flight file.
  if (!flashBudgetHasRoom(4096UL)) {
    loggerFull = true;
    Serial.println("LOGGER: insufficient filesystem headroom for flight log.");
    return false;
  }
  int8_t slot = selectWriteSlot(LOG_KIND_FLIGHT);
  if (slot < 0 || !eraseOneLog(LOG_KIND_FLIGHT, (uint8_t)slot)) {
    loggerFull = true;
    return false;
  }
  activeSlot = slot;
  activeHeader = {};
  activeHeader.status = SLOT_RECORDING;
  activeHeader.flightNumber = nextFlightNumber++;
  activeHeader.startMillis = prelaunchCount ?
      prelaunchRing[(prelaunchHead + PRELAUNCH_RECORDS - prelaunchCount) % PRELAUNCH_RECORDS].t_ms : millis();
  if (!writeStoredHeader(LOG_KIND_FLIGHT, (uint8_t)slot, activeHeader)) return false;
  logFile = LittleFS.open(LOG_SLOT_FILES[slot], "a");
  if (!logFile) return false;
  loggerActive = true;
  loggerFull = false;
  lastLogMs = millis();
  // Persist prelaunch history oldest-first; 100 records maximum (3.2 kB RAM).
  for (uint16_t i = 0; i < prelaunchCount; ++i) {
    uint16_t idx = (prelaunchHead + PRELAUNCH_RECORDS - prelaunchCount + i) % PRELAUNCH_RECORDS;
    if (logFile.write((const uint8_t*)&prelaunchRing[idx], sizeof(FlightLogRecord))
        != sizeof(FlightLogRecord)) {
      Serial.println("LOGGER ERROR: prelaunch write failed.");
      break;
    }
    activeHeader.recordCount++;
  }
  prelaunchCount = 0;
  prelaunchHead = 0;
  pyroDeferredCount = 0;
  // A power session which became a flight is classified only as FLIGHT.
  closeStaticLogger(true);
  Serial.print("FLIGHT LOG START ID="); Serial.print(activeHeader.flightNumber);
  Serial.print(" SLOT="); Serial.println(activeSlot);
  return true;
}

void finishActiveLog(const char* reason = "COMPLETE") {
  if (!loggerActive) return;
  logFile.flush();
  logFile.close();
  activeHeader.status = SLOT_COMPLETE;
  activeHeader.completedMillis = millis();
  writeStoredHeader(LOG_KIND_FLIGHT, (uint8_t)activeSlot, activeHeader);
  loggerActive = false;
  Serial.print("FLIGHT LOG CLOSED: "); Serial.print(reason);
  Serial.print(" ID="); Serial.print(activeHeader.flightNumber);
  Serial.print(" RECORDS="); Serial.println(activeHeader.recordCount);
}

uint8_t makeLogFlags() {
  uint8_t flags = 0;
  if (drogueFired) flags |= 1 << 0;
  if (mainFired) flags |= 1 << 1;
  if (mainArmed) flags |= 1 << 2;
  if (pyroActive()) flags |= 1 << 3;
  if (mplOK) flags |= 1 << 4;
  if (imuOK) flags |= 1 << 5;
  if (t_launch > 0) flags |= 1 << 6;
  if (loggerFull) flags |= 1 << 7;
  return flags;
}

uint16_t makeEventFlags() {
  uint16_t flags = 0;
  if (state == FlightState::ASCENT_LOCKOUT) flags |= 1 << 0;
  if (state == FlightState::APOGEE_DETECT) flags |= 1 << 1;
  if (state == FlightState::DROGUE_DEPLOYED) flags |= 1 << 2;
  if (state == FlightState::MAIN_DEPLOYED) flags |= 1 << 3;
  if (loggerFull) flags |= 1 << 4;
  if (pyroActive()) flags |= 1 << 5;
  return flags | ((uint16_t)pendingLogEventCode << 8);
}

FlightLogRecord makeCurrentLogRecord() {
  FlightLogRecord r = {};
  float outAlt_m = (t_launch > 0) ? altAGL_m : alt_m;
  r.t_ms = millis();
  r.alt_cm = (int32_t)lroundf(outAlt_m * 100.0f);
  r.vz_cms = clampI16(lroundf(vz_mps * 100.0f)); // telemetry only, not deployment
  r.ax_cg = clampI16(lroundf(gFromMps2(ax_mps2) * 100.0f));
  r.ay_cg = clampI16(lroundf(gFromMps2(ay_mps2) * 100.0f));
  r.az_cg = clampI16(lroundf(gFromMps2(az_mps2) * 100.0f));
  r.gx_cdps = clampI16(lroundf(gx_dps * 100.0f));
  r.gy_cdps = clampI16(lroundf(gy_dps * 100.0f));
  r.gz_cdps = clampI16(lroundf(gz_dps * 100.0f));
  r.batt_mv = clampU16(lroundf(batt_V * 1000.0f));
  r.temp_cC = clampI16(lroundf((mplOK ? mplTemp_C : imuTemp_C) * 100.0f));
  r.state = (uint8_t)state;
  r.flags = makeLogFlags();
  r.eventFlags = makeEventFlags();
  r.crc = 0;
  r.crc = checksum16((const uint8_t*)&r, sizeof(r));
  return r;
}

void serviceFlightLogger(uint32_t now) {
  if (!LOGGING_ENABLED || now - lastLogMs < LOG_PERIOD_MS) return;
  lastLogMs = now;
  FlightLogRecord record = makeCurrentLogRecord();
  if (!loggerActive) {
    if (state == FlightState::IDLE && !downloadMode) {
      prelaunchRing[prelaunchHead] = record;
      prelaunchHead = (prelaunchHead + 1) % PRELAUNCH_RECORDS;
      if (prelaunchCount < PRELAUNCH_RECORDS) prelaunchCount++;
      pendingLogEventCode = LOG_EVENT_NONE;
    }
    return;
  }
  if (pyroActive()) {
    if (pyroDeferredCount < 16) pyroDeferredRecords[pyroDeferredCount++] = record;
    // Do not clear a pending event until it has been buffered.
    pendingLogEventCode = LOG_EVENT_NONE;
    return;
  }
  for (uint8_t i = 0; i < pyroDeferredCount; ++i) {
    if (logFile.write((const uint8_t*)&pyroDeferredRecords[i], sizeof(FlightLogRecord))
        != sizeof(FlightLogRecord)) { finishActiveLog("DEFERRED_WRITE_FAIL"); return; }
    activeHeader.recordCount++;
  }
  pyroDeferredCount = 0;
  if (logFile.write((const uint8_t*)&record, sizeof(record)) != sizeof(record)) {
    finishActiveLog("WRITE_FAIL");
    return;
  }
  activeHeader.recordCount++;
  pendingLogEventCode = LOG_EVENT_NONE;
  if (activeHeader.recordCount % LOG_DATA_FLUSH_RECORDS == 0) logFile.flush();

  // Avoid deletion/sector reclamation during ascent and deployment. Preflight
  // reserves space; if an extraordinarily long flight exceeds it, protect the
  // time-critical flight loop rather than blocking on a mass erase.
  if (activeHeader.recordCount % 100 == 0 && !flashBudgetHasRoom(0)) {
    // Never erase old files during an active flight or deployment pulse.
    if (!flashBudgetHasRoom(0)) {
      loggerFull = true;
      finishActiveLog("80_PERCENT_FILESYSTEM_CAP");
    }
  }
}

void serviceStaticLogger(uint32_t now) {
  if (!staticLoggerActive || state != FlightState::IDLE || t_launch > 0 || pyroActive()) return;
  if (now - lastStaticLogMs < STATIC_LOG_PERIOD_MS) return;
  lastStaticLogMs = now;
  FlightLogRecord record = makeCurrentLogRecord();
  staticFile.seek(sizeof(FlightLogHeader) + (uint32_t)staticWriteIndex * sizeof(record));
  if (staticFile.write((const uint8_t*)&record, sizeof(record)) != sizeof(record)) {
    closeStaticLogger(false);
    return;
  }
  if (activeStaticHeader.recordCount < STATIC_MAX_RECORDS) activeStaticHeader.recordCount++;
  staticWriteIndex = (staticWriteIndex + 1) % STATIC_MAX_RECORDS;
  if (staticWriteIndex % 30 == 0) staticFile.flush();
  if (staticWriteIndex % 60 == 0 && !flashBudgetHasRoom(256UL * 1024UL)) reclaimSpaceBeforeFlight();
}

// For a full circular static file, determine oldest sample by monotonic
// millis() inside the single power session. This is only called for downloads.
uint32_t firstStaticRecordIndex(File& f, uint32_t count) {
  if (count < STATIC_MAX_RECORDS) return 0;
  uint32_t bestIndex = 0;
  uint32_t smallest = 0xFFFFFFFFUL;
  for (uint32_t i = 0; i < count; ++i) {
    FlightLogRecord r = {};
    f.seek(sizeof(FlightLogHeader) + i * sizeof(r));
    if (f.read((uint8_t*)&r, sizeof(r)) != sizeof(r)) break;
    if (r.t_ms < smallest) { smallest = r.t_ms; bestIndex = i; }
  }
  return bestIndex;
}

void dumpStoredCsv(uint8_t kind, uint8_t slot) {
  FlightLogHeader h = {};
  if (!readStoredHeader(kind, slot, h)) { Serial.println("ERR: invalid log slot"); return; }
  if (kind == LOG_KIND_FLIGHT && loggerActive && activeSlot == (int8_t)slot) {
    Serial.println("ERR: cannot dump an active flight log"); return;
  }
  if (kind == LOG_KIND_STATIC && staticLoggerActive && activeStaticSlot == (int8_t)slot) {
    staticFile.flush();
    h = activeStaticHeader;
  }
  File f = LittleFS.open(logPath(kind, slot), "r");
  if (!f) { Serial.println("ERR: open failed"); return; }
  uint32_t available = f.size() > sizeof(FlightLogHeader) ?
      (f.size() - sizeof(FlightLogHeader)) / sizeof(FlightLogRecord) : 0;
  uint32_t count = h.recordCount ? (h.recordCount < available ? h.recordCount : available) : available;
  uint32_t begin = (kind == LOG_KIND_STATIC) ? firstStaticRecordIndex(f, count) : 0;
  Serial.println();
  Serial.print("BEGIN_CSV "); Serial.print(kind == LOG_KIND_FLIGHT ? "SLOT " : "STATIC ");
  Serial.println(slot);
  Serial.println("t_ms,state,state_name,flags,eventFlags,eventCode,alt_m,vz_mps,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,batt_V,temp_C,crc_ok,event_note");
  for (uint32_t i = 0; i < count; ++i) {
    uint32_t index = (kind == LOG_KIND_STATIC) ? (begin + i) % count : i;
    f.seek(sizeof(FlightLogHeader) + index * sizeof(FlightLogRecord));
    FlightLogRecord r = {};
    if (f.read((uint8_t*)&r, sizeof(r)) != sizeof(r)) break;
    uint16_t crc = r.crc;
    r.crc = 0;
    uint16_t calc = checksum16((const uint8_t*)&r, sizeof(r));
    uint8_t ev = (uint8_t)(r.eventFlags >> 8);
    Serial.print(r.t_ms); Serial.print(',');
    Serial.print(r.state); Serial.print(',');
    Serial.print(stateStr((FlightState)r.state)); Serial.print(',');
    Serial.print(r.flags); Serial.print(',');
    Serial.print(r.eventFlags); Serial.print(',');
    Serial.print(ev); Serial.print(',');
    Serial.print(r.alt_cm / 100.0f, 2); Serial.print(',');
    Serial.print(r.vz_cms / 100.0f, 2); Serial.print(',');
    Serial.print(r.ax_cg / 100.0f, 2); Serial.print(',');
    Serial.print(r.ay_cg / 100.0f, 2); Serial.print(',');
    Serial.print(r.az_cg / 100.0f, 2); Serial.print(',');
    Serial.print(r.gx_cdps / 100.0f, 2); Serial.print(',');
    Serial.print(r.gy_cdps / 100.0f, 2); Serial.print(',');
    Serial.print(r.gz_cdps / 100.0f, 2); Serial.print(',');
    Serial.print(r.batt_mv / 1000.0f, 3); Serial.print(',');
    Serial.print(r.temp_cC / 100.0f, 2); Serial.print(',');
    Serial.print(calc == crc ? 1 : 0); Serial.print(',');
    Serial.println(logEventNote(ev));
    if (i % 64 == 0) delay(1);
  }
  Serial.print("END_CSV "); Serial.print(kind == LOG_KIND_FLIGHT ? "SLOT " : "STATIC ");
  Serial.println(slot); Serial.println();
  f.close();
}

void dumpStoredBinary(uint8_t kind, uint8_t slot) {
  FlightLogHeader h = {};
  if (!readStoredHeader(kind, slot, h)) { Serial.println("ERR: invalid log slot"); return; }
  if (kind == LOG_KIND_FLIGHT && loggerActive && activeSlot == (int8_t)slot) {
    Serial.println("ERR: cannot dump an active flight log"); return;
  }
  if (kind == LOG_KIND_STATIC && staticLoggerActive && activeStaticSlot == (int8_t)slot) {
    staticFile.flush();
  }
  File f = LittleFS.open(logPath(kind, slot), "r");
  if (!f) return;
  Serial.print("BEGIN_BIN "); Serial.print(kind == LOG_KIND_FLIGHT ? "SLOT " : "STATIC ");
  Serial.print(slot); Serial.print(" BYTES "); Serial.println(f.size());
  while (f.available()) {
    uint8_t buf[64];
    int n = f.read(buf, sizeof(buf));
    if (n <= 0) break;
    Serial.write(buf, n);
    delay(1);
  }
  Serial.println();
  Serial.print("END_BIN "); Serial.print(kind == LOG_KIND_FLIGHT ? "SLOT " : "STATIC ");
  Serial.println(slot);
  f.close();
}

void dumpSlotCsv(uint8_t slot) { dumpStoredCsv(LOG_KIND_FLIGHT, slot); }
void dumpSlotBinary(uint8_t slot) { dumpStoredBinary(LOG_KIND_FLIGHT, slot); }

void markSlotDownloaded(uint8_t slot) {
  FlightLogHeader h = {};
  if (loggerActive && activeSlot == (int8_t)slot) { Serial.println("ERR: active flight"); return; }
  if (!readStoredHeader(LOG_KIND_FLIGHT, slot, h)) { Serial.println("ERR: invalid slot"); return; }
  h.status = SLOT_DOWNLOADED;
  h.downloaded = 1;
  if (writeStoredHeader(LOG_KIND_FLIGHT, slot, h)) Serial.println("OK: marked downloaded");
}

void eraseSlot(uint8_t slot) {
  Serial.println(eraseOneLog(LOG_KIND_FLIGHT, slot) ? "OK: flight erased" : "ERR: cannot erase flight");
}

bool clearOldestFlightSlot() {
  int8_t slot = oldestStoredSlot(LOG_KIND_FLIGHT);
  if (slot < 0) return false;
  bool ok = eraseOneLog(LOG_KIND_FLIGHT, (uint8_t)slot);
  if (ok) loggerFull = false;
  return ok;
}

void serviceLoggerFullUI(uint32_t now) {
  if (!loggerFull || loggerActive || t_launch > 0) return;
  if (now - fullBlinkLastMs >= FULL_YELLOW_BLINK_MS) {
    fullBlinkLastMs = now;
    fullBlinkOn = !fullBlinkOn;
    setRGB(fullBlinkOn, fullBlinkOn, false);
  }
  bool pressed = digitalRead(MODE_SWITCH_PIN) == LOW;
  if (!pressed) { fullButtonHoldStartMs = 0; return; }
  if (!fullButtonHoldStartMs) fullButtonHoldStartMs = now;
  if (now - fullButtonHoldStartMs >= FULL_SLOT_HOLD_CLEAR_MS) {
    if (clearOldestFlightSlot()) Serial.println("OLD FLIGHT ERASED");
    fullButtonHoldStartMs = 0;
    updateRGBMode();
  }
}

void formatAllLogs() {
  if (!loggerMounted) { Serial.println("ERR: no mounted filesystem; use FORMATFS CONFIRM only if data loss is acceptable"); return; }
  if (loggerActive || t_launch > 0 || pyroActive()) { Serial.println("ERR: flight active"); return; }
  closeStaticLogger(false);
  for (uint8_t i = 0; i < LOG_SLOT_COUNT; ++i) LittleFS.remove(LOG_SLOT_FILES[i]);
  for (uint8_t i = 0; i < STATIC_SLOT_COUNT; ++i) LittleFS.remove(STATIC_SLOT_FILES[i]);
  // Also clear the three legacy v1 files; settings remain intact.
  LittleFS.remove("/flight0.bin");
  LittleFS.remove("/flight1.bin");
  LittleFS.remove("/flight2.bin");
  loggerFull = false;
  Serial.println("OK: logs formatted (saved profiles preserved)");
  startStaticLogger();
}

void exportAllCsv() {
  if (t_launch > 0 || loggerActive || pyroActive()) {
    Serial.println("ERR: EXPORTALLCSV denied during flight or pyro activity");
    return;
  }
  Serial.println("BEGIN_EXPORT_ALL");
  listLogSlots();
  for (uint8_t i = 0; i < LOG_SLOT_COUNT; ++i) {
    FlightLogHeader h = {};
    if (readStoredHeader(LOG_KIND_FLIGHT, i, h) && h.status != SLOT_EMPTY)
      dumpStoredCsv(LOG_KIND_FLIGHT, i);
  }
  for (uint8_t i = 0; i < STATIC_SLOT_COUNT; ++i) {
    FlightLogHeader h = {};
    if (readStoredHeader(LOG_KIND_STATIC, i, h) && h.status != SLOT_EMPTY)
      dumpStoredCsv(LOG_KIND_STATIC, i);
  }
  Serial.println("END_EXPORT_ALL");
}

bool parseLogSlot(String value, int maxSlot, int& parsed) {
  value.trim();
  if (value.length() == 0) return false;
  for (unsigned i=0; i<value.length(); ++i)
    if (!isDigit(value[i])) return false;
  int n = value.toInt();
  if (n < 0 || n >= maxSlot) return false;
  parsed = n;
  return true;
}

void printLogStatus() {
  Serial.println("\n===== LOGGER STATUS =====");
  Serial.print("loggerMounted: "); Serial.println(loggerMounted ? "YES" : "NO");
  Serial.print("loggerActive: "); Serial.println(loggerActive ? "YES" : "NO");
  Serial.print("loggerFull: "); Serial.println(loggerFull ? "YES" : "NO");
  Serial.print("downloadMode: "); Serial.println(downloadMode ? "YES" : "NO");
  Serial.print("static active: "); Serial.println(staticLoggerActive);
  Serial.print("activeSlot: "); Serial.println(activeSlot);
  Serial.print("static slot: "); Serial.println(activeStaticSlot);
  Serial.print("prelaunch buffered: "); Serial.print(prelaunchCount);
  Serial.print("/"); Serial.println(PRELAUNCH_RECORDS);
  Serial.print("flight records: "); Serial.println(activeHeader.recordCount);
  Serial.print("flight ID next: "); Serial.println(nextFlightNumber);
  Serial.print("static ID next: "); Serial.println(nextStaticNumber);
  if (loggerMounted) {
    FSInfo info = {};
    if (LittleFS.info(info)) {
      Serial.print("LittleFS total bytes: "); Serial.println(info.totalBytes);
      Serial.print("LittleFS used bytes: "); Serial.println(info.usedBytes);
      Serial.print("Budget (80% partition): "); Serial.println((uint32_t)((uint64_t)info.totalBytes * 80ULL / 100ULL));
    }
  }
  Serial.println("=========================\n");
}

void printParams() {
  Serial.println();
  Serial.println("===== COMET SIMPLE RECOVERY =====");
  Serial.print("PARAM MAIN_ALT_MODE "); Serial.println(rgbMode);
  Serial.print("PARAM MAIN_ALT "); Serial.println(comet_recovery::Logic::target(mainAltM), 3);
  Serial.print("PARAM MAIN_HARD_FLOOR_M "); Serial.println(comet_recovery::MAIN_ABSOLUTE_FLOOR_M, 1);
  Serial.print("PARAM BARO_LAUNCH_RISE_M "); Serial.println(comet_recovery::BARO_LAUNCH_RISE_M, 1);
  Serial.print("PARAM ACCEL_LAUNCH_DELTA_G "); Serial.println(comet_recovery::ACCEL_LAUNCH_DELTA_G, 2);
  Serial.print("PARAM APOGEE_DROP_M "); Serial.println(comet_recovery::APOGEE_DROP_M, 1);
  Serial.print("PARAM APOGEE_DROP_DWELL_MS "); Serial.println(comet_recovery::APOGEE_DROP_DWELL_MS);
  Serial.print("PARAM ACCEL_ENABLE "); Serial.println(accelMethodsEnabled ? 1 : 0);
  Serial.println("PARAM BARO_ENABLE 1 (hard-required)");
  Serial.print("PARAM TIMER_ENABLE "); Serial.println(timerMethodsEnabled ? 1 : 0);
  Serial.print("PARAM DROGUE_BACKUP_MS "); Serial.println((unsigned long)drogueBackupMs);
  Serial.print("PARAM MAIN_BACKUP_MS "); Serial.println((unsigned long)mainBackupMs);
  Serial.println("Backup delays are measured from LAUNCH, NOT from apogee/drogue.");
  Serial.println("ENABLE only after sim/replay checks; timer backups can fire during ascent.");
  Serial.println("===============================");
  Serial.println();
}

bool parseEnableValue(const String& value, bool& out) {
  String v = value;
  v.trim();
  v.toUpperCase();

  if (v == "1" || v == "ON" || v == "TRUE" || v == "ENABLE" || v == "ENABLED") {
    out = true;
    return true;
  }

  if (v == "0" || v == "OFF" || v == "FALSE" || v == "DISABLE" || v == "DISABLED") {
    out = false;
    return true;
  }

  return false;
}

// Retained in milliseconds for compatibility with the pre-existing COMET GUI.
// Disallow signs/fractions/overflow and enforce 1..600 seconds.
bool parseBackupMilliseconds(const String& input, uint32_t& milliseconds) {
  String v = input;
  v.trim();
  if (v.length() == 0 || v.length() > 6) return false;
  for (unsigned i = 0; i < v.length(); ++i) if (!isDigit(v[i])) return false;
  long n = v.toInt();
  if (n < 1000L || n > 600000L) return false;
  milliseconds = (uint32_t)n;
  return true;
}

// Main timer is launch-relative, so it must follow drogue's timer by at least
// 2 seconds when the backup pair is enabled.
bool backupDelayPairValid(uint32_t drogueMs, uint32_t mainMs) {
  return mainMs >= drogueMs + 2000UL;
}

bool setParam(const String& name, const String& value) {
  if (t_launch > 0 || pyroActive()) {
    Serial.println("ERR: settings locked during flight/pyro activity.");
    return false;
  }
  bool changed = false;
  if (name == "MAIN_ALT_MODE") {
    int mode = findMainAltitudeMode(value);
    if (mode < 0) return false;
    setMainAltitudeMode((uint8_t)mode, true, false);
    changed = true;
  } else if (name == "MAIN_ALT") {
    float fv = value.toFloat();
    if (!finitef_safe(fv) || fv < comet_recovery::MAIN_ABSOLUTE_FLOOR_M || fv > 10000.0f) {
      Serial.println("ERR: MAIN_ALT must be 200..10000 m. Hard 200m floor is not editable.");
      return false;
    }
    mainAltM = fv;
    changed = true;
  } else if (name == "ACCEL_ENABLE") {
    bool enabled;
    if (!parseEnableValue(value, enabled)) return false;
    accelMethodsEnabled = enabled;
    changed = true;
  } else if (name == "TIMER_ENABLE") {
    bool enabled;
    if (!parseEnableValue(value, enabled)) return false;
    if (enabled && !backupDelayPairValid(drogueBackupMs, mainBackupMs)) {
      Serial.println("ERR: main backup must be at least 2000 ms later than drogue backup.");
      return false;
    }
    timerMethodsEnabled = enabled;
    changed = true;
  } else if (name == "DROGUE_BACKUP_MS" || name == "MAIN_BACKUP_MS") {
    uint32_t ms;
    if (!parseBackupMilliseconds(value, ms)) return false;
    const uint32_t newD = name == "DROGUE_BACKUP_MS" ? ms : drogueBackupMs;
    const uint32_t newM = name == "MAIN_BACKUP_MS" ? ms : mainBackupMs;
    if (timerMethodsEnabled && !backupDelayPairValid(newD, newM)) return false;
    drogueBackupMs = newD;
    mainBackupMs = newM;
    changed = true;
  } else {
    Serial.println("ERR: obsolete apogee/lockout/baro-disable settings cannot be changed.");
    return false;
  }
  if (changed) {
    captureRuntimeIntoActiveProfile();
    markSettingsDirty();
    Serial.print("OK SET "); Serial.print(name); Serial.print(' '); Serial.println(value);
  }
  return changed;
}

void printProfile(uint8_t mode) {
  if (mode >= MAIN_ALT_MODE_COUNT) return;
  const FlightProfile& p = flightProfiles[mode];
  Serial.print("PROFILE "); Serial.print(MAIN_ALT_MODES[mode].name);
  Serial.print(" MAIN_ALT "); Serial.print(comet_recovery::Logic::target(p.mainAltM), 3);
  Serial.print(" ACCEL_ENABLE "); Serial.print(p.accelEnabled ? 1 : 0);
  Serial.print(" TIMER_ENABLE "); Serial.print(p.timerEnabled ? 1 : 0);
  Serial.print(" DROGUE_BACKUP_MS "); Serial.print((unsigned long)p.drogueBackupMs);
  Serial.print(" MAIN_BACKUP_MS "); Serial.println((unsigned long)p.mainBackupMs);
}

bool setProfileParam(uint8_t mode, const String& name, const String& value) {
  if (mode >= MAIN_ALT_MODE_COUNT || t_launch > 0 || pyroActive()) return false;
  FlightProfile candidate = flightProfiles[mode];
  if (name == "MAIN_ALT") {
    float fv = value.toFloat();
    if (!finitef_safe(fv) || fv < comet_recovery::MAIN_ABSOLUTE_FLOOR_M || fv > 10000.0f) return false;
    candidate.mainAltM = fv;
  } else if (name == "ACCEL_ENABLE") {
    bool v;
    if (!parseEnableValue(value, v)) return false;
    candidate.accelEnabled = v ? 1 : 0;
  } else if (name == "TIMER_ENABLE") {
    bool v;
    if (!parseEnableValue(value, v)) return false;
    candidate.timerEnabled = v ? 1 : 0;
  } else if (name == "DROGUE_BACKUP_MS" || name == "MAIN_BACKUP_MS") {
    uint32_t ms;
    if (!parseBackupMilliseconds(value, ms)) return false;
    if (name == "DROGUE_BACKUP_MS") candidate.drogueBackupMs = ms;
    else candidate.mainBackupMs = ms;
  } else return false;
  if (candidate.timerEnabled &&
      !backupDelayPairValid(candidate.drogueBackupMs, candidate.mainBackupMs)) return false;
  if (!profileValid(candidate)) return false;
  flightProfiles[mode] = candidate;
  if (mode == (uint8_t)rgbMode) applyFlightProfile(mode);
  markSettingsDirty();
  return true;
}

void printLoggerHelp() {
  Serial.println("\nCOMET rolling logger commands:");
  Serial.println("  LIST                  - show flight/static slots; newest marked *");
  Serial.println("  LASTFLIGHT / LASTSTATIC - show newest physical slot");
  Serial.println("  EXPORTALLCSV          - dump all flight+static logs, one command");
  Serial.println("  DUMPCSV 0..19         - export one flight CSV");
  Serial.println("  DUMPCSV LASTFLIGHT    - export newest flight CSV");
  Serial.println("  DUMPBIN 0..19         - raw binary flight dump");
  Serial.println("  STATICCSV 0..2        - export one static session CSV");
  Serial.println("  STATICCSV LASTSTATIC - export newest static CSV (can be active)");
  Serial.println("  STATICBIN 0..2        - raw binary static dump");
  Serial.println("  ERASE 0..19           - erase one flight slot (old command)");
  Serial.println("  ERASESTATIC 0..2      - erase one static session");
  Serial.println("  CLEAROLDEST           - erase oldest flight");
  Serial.println("  MARKDOWNLOADED n      - mark flight downloaded (legacy)");
  Serial.println("  FORMATLOG             - erase all logs, preserve settings (old command)");
  Serial.println("  FORMATFS CONFIRM      - erase entire LittleFS partition (DESTRUCTIVE)");
  Serial.println("  STOPLOG / STARTLOG    - manual bench flight logging");
  Serial.println("  LOGSTATUS             - show LittleFS size, usage, 80% budget");
  Serial.println("  GETPARAMS, SET MAIN_ALT, SET ACCEL_ENABLE, PROFILE ...");
  Serial.println("  SET TIMER_ENABLE 0|1       - default OFF; enables both launch-relative backups");
  Serial.println("  SET DROGUE_BACKUP_MS 15000 - drogue timer after launch, 1..600 s");
  Serial.println("  SET MAIN_BACKUP_MS 25000   - main timer after launch, only after drogue");
  Serial.println("  PROFILE SET <COLOR> TIMER_ENABLE 0|1 (also supports both _BACKUP_MS fields)");
  Serial.println("  Prelaunch history: 10s at 10Hz; static: 1Hz, max 30min;");
  Serial.println("  landing close: 15s stable altitude+accel+gyro after launch.\n");
}

bool handleLoggerCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  // These commands can touch LittleFS or alter flight parameters. Keep them
  // completely out of the active-flight path. GETPARAMS remains safe because
  // it only reports RAM values.
  if (t_launch > 0) {
    bool unsafeDuringFlight =
      cmd == "LIST" ||
      cmd == "EXPORTALLCSV" ||
      cmd == "LASTFLIGHT" || cmd == "LASTSTATIC" ||
      cmd.startsWith("STATICCSV") || cmd.startsWith("STATICBIN") ||
      cmd.startsWith("ERASESTATIC") || cmd == "CLEAROLDEST" ||
      cmd == "FORMATFS CONFIRM" ||
      cmd.startsWith("DUMPCSV") ||
      cmd.startsWith("DUMPBIN") ||
      cmd.startsWith("MARKDOWNLOADED") ||
      cmd.startsWith("ERASE") ||
      cmd == "FORMATLOG" ||
      cmd == "STOPLOG" ||
      cmd == "STARTLOG" ||
      cmd == "LOGSTATUS" ||
      cmd == "SAVEPARAMS" ||
      cmd.startsWith("SET ") ||
      cmd.startsWith("PROFILE ");

    if (unsafeDuringFlight) {
      Serial.println("ERR: logger/settings management is locked during flight. RESET first.");
      return true;
    }
  }

  if (cmd == "FORMATFS CONFIRM") {
    if (loggerActive || t_launch > 0 || pyroActive()) {
      Serial.println("ERR: format denied during active flight"); return true;
    }
    if (staticLoggerActive) {
      staticFile.flush(); staticFile.close();
      staticLoggerActive = false; activeStaticSlot = -1;
    }
    if (loggerMounted) LittleFS.end();
    bool formatted = LittleFS.format();
    loggerMounted = formatted && LittleFS.begin();
    if (loggerMounted) {
      nextFlightNumber = nextStaticNumber = 1;
      activeSlot = activeStaticSlot = -1;
      loggerFull = false;
      saveCometSettings();
      startStaticLogger();
      Serial.println("OK: entire filesystem formatted, settings rewritten");
    } else Serial.println("ERR: filesystem format failed");
    return true;
  }
  if (cmd == "EXPORTALLCSV") { exportAllCsv(); return true; }
  if (cmd == "LASTFLIGHT") {
    Serial.print("LASTFLIGHT="); Serial.println(latestStoredSlot(LOG_KIND_FLIGHT)); return true;
  }
  if (cmd == "LASTSTATIC") {
    Serial.print("LASTSTATIC="); Serial.println(latestStoredSlot(LOG_KIND_STATIC)); return true;
  }
  if (cmd == "CLEAROLDEST") {
    Serial.println(clearOldestFlightSlot() ? "OK: oldest flight erased" : "ERR: no flight available");
    return true;
  }
  if (cmd.startsWith("STATICCSV") || cmd.startsWith("STATICBIN")) {
    bool csv = cmd.startsWith("STATICCSV");
    String arg = cmd.substring(9); arg.trim();
    int slot = (arg == "LASTSTATIC") ? latestStoredSlot(LOG_KIND_STATIC) :
      (arg.length() > 0 ? arg.toInt() : -1);
    if (slot < 0 || slot >= STATIC_SLOT_COUNT) Serial.println("ERR: invalid static slot");
    else if (csv) dumpStoredCsv(LOG_KIND_STATIC, (uint8_t)slot);
    else dumpStoredBinary(LOG_KIND_STATIC, (uint8_t)slot);
    return true;
  }
  if (cmd.startsWith("ERASESTATIC")) {
    String arg = cmd.substring(11); arg.trim();
    int slot = arg.length() > 0 ? arg.toInt() : -1;
    if (slot < 0 || slot >= STATIC_SLOT_COUNT) Serial.println("ERR: invalid static slot");
    else Serial.println(eraseOneLog(LOG_KIND_STATIC, (uint8_t)slot) ?
      "OK: static erased" : "ERR: static active / cannot erase");
    return true;
  }
  if (cmd == "LIST") {
    listLogSlots();
    return true;
  }

  if (cmd.startsWith("DUMPCSV") || cmd.startsWith("DUMPBIN")) {
    bool csv = cmd.startsWith("DUMPCSV");
    String arg = cmd.substring(7); arg.trim();
    int slot = (arg == "LASTFLIGHT") ? latestStoredSlot(LOG_KIND_FLIGHT) :
      (arg.length() > 0 ? arg.toInt() : -1);
    if (slot < 0 || slot >= LOG_SLOT_COUNT) Serial.println("ERR: invalid flight slot");
    else if (csv) dumpSlotCsv((uint8_t)slot);
    else dumpSlotBinary((uint8_t)slot);
    return true;
  }

  if (cmd.startsWith("MARKDOWNLOADED")) {
    int slot = cmd.substring(14).toInt();
    markSlotDownloaded((uint8_t)slot);
    return true;
  }

  if (cmd.startsWith("ERASE")) {
    int slot = -1;
    if (!parseLogSlot(cmd.substring(5), LOG_SLOT_COUNT, slot))
      Serial.println("ERR: use ERASE <0..19>");
    else eraseSlot((uint8_t)slot);
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

  if (cmd == "SAVEPARAMS") {
    if (settingsWriteAllowed()) {
      if (saveCometSettings()) {
        settingsDirty = false;
      }
    } else {
      markSettingsDirty();
      Serial.println("SAVEPARAMS deferred until safe preflight IDLE conditions.");
    }
    return true;
  }

  if (cmd == "PROFILE LIST") {
    for (uint8_t i = 0; i < MAIN_ALT_MODE_COUNT; i++) {
      printProfile(i);
    }
    return true;
  }

  if (cmd.startsWith("PROFILE GET ")) {
    String modeText = cmd.substring(12);
    modeText.trim();
    int mode = findMainAltitudeMode(modeText);
    if (mode < 0) {
      Serial.println("ERR: invalid profile color/index.");
    } else {
      printProfile((uint8_t)mode);
    }
    return true;
  }

  if (cmd.startsWith("PROFILE APPLY ")) {
    String modeText = cmd.substring(14);
    modeText.trim();
    int mode = findMainAltitudeMode(modeText);
    if (mode < 0) {
      Serial.println("ERR: invalid profile color/index.");
    } else {
      setMainAltitudeMode((uint8_t)mode, true, true);
      printParams();
    }
    return true;
  }

  if (cmd.startsWith("PROFILE SET ")) {
    String rest = cmd.substring(12);
    rest.trim();

    int s1 = rest.indexOf(' ');
    int s2 = (s1 >= 0) ? rest.indexOf(' ', s1 + 1) : -1;

    if (s1 < 0 || s2 < 0) {
      Serial.println("ERR: use PROFILE SET <COLOR> <PARAM> <VALUE>");
      return true;
    }

    String modeText = rest.substring(0, s1);
    String name = rest.substring(s1 + 1, s2);
    String value = rest.substring(s2 + 1);
    modeText.trim();
    name.trim();
    value.trim();

    int mode = findMainAltitudeMode(modeText);
    if (mode < 0 || !setProfileParam((uint8_t)mode, name, value)) {
      Serial.println("ERR: invalid PROFILE SET command/value.");
    } else {
      Serial.print("OK PROFILE SET ");
      Serial.print(MAIN_ALT_MODES[mode].name);
      Serial.print(' ');
      Serial.print(name);
      Serial.print(' ');
      Serial.println(value);
    }
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
    latestImuReadMs = now;
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
      latestBaroReadMs = now;
    }
  }

  // ---------------- AGL reference and diagnostic maximum ----------------
  // Ground reference remains the boot-calibrated pad altitude even for a
  // barometer-triggered launch (which necessarily detects ascent late).
  if (t_launch > 0) {
    altAGL_m = alt_m - launchAlt_m;
    if (altAGL_m > maxAltAGL_m) maxAltAGL_m = altAGL_m;
    // MAINARM is diagnostic only; it never gates an altitude deployment.
    mainArmed = maxAltAGL_m >= comet_recovery::Logic::target(mainAltM);
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

// Launch and apogee decisions now live exclusively in SimpleRecoveryLogic.h.
// The legacy launchDetectAccel()/positiveTrend()/negativeTrend() gates have
// been removed; Vz remains available for telemetry and flight-log diagnostics.

// ============================================================================
// Telemetry
// ============================================================================

void serviceLandingDetection(uint32_t now) {
  if (!loggerActive || t_launch == 0 || pyroActive()) return;
  // Archive completes on 15 seconds of quiet sensors after launch. This is
  // independent of recovery success; even failed deployments remain logged.
  const bool baroFresh = mplOK && latestBaroReadMs > 0 &&
                         (uint32_t)(now - latestBaroReadMs) <= 350UL;
  const bool imuFresh = imuOK && latestImuReadMs > 0 &&
                        (uint32_t)(now - latestImuReadMs) <= 150UL;
  const float rotMag = sqrtf(gx_dps * gx_dps + gy_dps * gy_dps + gz_dps * gz_dps);
  const bool imuStill = imuFresh && accelBaselineValid &&
                        fabsf(accelMag_g - accelMagBaseline_g) <= LANDING_ACCEL_TOL_G &&
                        rotMag <= LANDING_GYRO_MAX_DPS;
  if (!baroFresh || !imuStill) {
    landingStableStartMs = 0;
    return;
  }
  if (landingStableStartMs == 0) {
    landingStableStartMs = now;
    landingStableReferenceAlt = alt_m;
    return;
  }
  if (fabsf(alt_m - landingStableReferenceAlt) > LANDING_MAX_ALT_DRIFT_M) {
    landingStableStartMs = now;
    landingStableReferenceAlt = alt_m;
    return;
  }
  if ((uint32_t)(now - landingStableStartMs) >= LANDING_STABLE_MS) {
    // Do not modify recovery state: log closure cannot stop recovery logic.
    sendTelemetry("LANDED_STABLE");
    FlightLogRecord endRecord = makeCurrentLogRecord();
    if (logFile.write((const uint8_t*)&endRecord, sizeof(endRecord)) == sizeof(endRecord)) {
      activeHeader.recordCount++;
      pendingLogEventCode = LOG_EVENT_NONE;
    }
    finishActiveLog("LANDED_STABLE_15S");
  }
}

void sendTelemetry(const char* eventName = nullptr) {
  const uint32_t now = millis();

  uint8_t code = logEventCodeFromName(eventName);
  if (code != LOG_EVENT_NONE) {
    pendingLogEventCode = code;
  }

  float outAlt_m = (t_launch > 0) ? altAGL_m : alt_m;

  char pkt[520];

  // VZNEG in the legacy packet is informational; no old negative-Vz gate exists.
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
    "VZNEG:%.2f:"
    "BAROEN:%u:"
    "ACCELEN:%u:"
    "TIMEREN:%u:"
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
    comet_recovery::FALLING_VZ_MPS,
    baroMethodsEnabled ? 1 : 0,
    accelMethodsEnabled ? 1 : 0,
    timerMethodsEnabled ? 1 : 0,
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

bool deployDrogue(const char* reason, bool force = false) {
  (void)force; // Recovery logic owns apogee confirmation, not the GPIO handler.
  if (drogueFired || pyroActive()) return false;
  drogueFired = true;
  t_drogue_fire = millis();
  Serial.print(">>> DROGUE DEPLOY CMD: "); Serial.println(reason ? reason : "-");
  startPyroPreFireBeep();
  sendTelemetry(reason ? reason : "DROGUE_DEPLOY");
  startPyro(PyroKind::DROGUE, DROGUE_PIN);
  return true;
}

bool deployMain(const char* reason, bool force = false) {
  (void)force;
  if (mainFired || pyroActive()) return false;
  mainFired = true;
  t_main_fire = millis();
  Serial.print(">>> MAIN DEPLOY CMD: "); Serial.println(reason ? reason : "-");
  startPyroPreFireBeep();
  sendTelemetry(reason ? reason : "MAIN_DEPLOY");
  startPyro(PyroKind::MAIN, MAIN_PIN);
  return true;
}

// If an output is occupied, its pending request remains queued instead of
// advancing the state as though the deployment had occurred.
void serviceRecoveryRequests() {
  if (pyroActive()) return;
  if (pendingDrogueRequest) {
    if (drogueFired) pendingDrogueRequest = false;
    else if (deployDrogue(pendingDrogueReason, true)) {
      pendingDrogueRequest = false;
      state = FlightState::DROGUE_DEPLOYED;
      return; // Main must wait for the drogue pyro pulse to finish.
    }
  }
  if (pendingMainRequest) {
    if (mainFired) pendingMainRequest = false;
    else if (deployMain(pendingMainReason, true)) {
      pendingMainRequest = false;
      recoveryLogic.markMainAccepted();
      state = FlightState::MAIN_DEPLOYED;
    }
  }
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
  if (pyroActive()) { Serial.println("RESET denied during pyro pulse"); return; }
  if (loggerActive) finishActiveLog("BENCH_RESET");
  landingStableStartMs = 0;
  drogueFired = false;
  mainFired = false;
  mainArmed = false;
  pendingDrogueRequest = pendingMainRequest = false;
  recoveryLogic.reset();
  processedBaroReadMs = latestBaroReadMs;
  processedImuReadMs = latestImuReadMs;

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
  if (loggerMounted && !staticLoggerActive) startStaticLogger();

  Serial.println("Flight state reset to IDLE.");
  sendTelemetry("RESET");
}

void forceLaunch() {
  if (t_launch > 0 || pyroActive() || state != FlightState::IDLE) return;
  uint32_t now = millis();
  // Manual bench launch uses the current altitude as its own ground baseline.
  launchAlt_m = alt_m;
  altAGL_m = 0.0f;
  maxAltAGL_m = 0.0f;
  mainArmed = false;
  recoveryLogic.forceLaunch(0.0f);
  pendingDrogueRequest = pendingMainRequest = false;
  t_launch = now;
  state = FlightState::ASCENT;
  landingStableStartMs = 0;
  startLaunchDetectedSound();
  startNewLogIfSafe();
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
          if (pyroActive() || t_launch > 0) {
            Serial.println("BEEP ignored during flight/pyro activity.");
          } else {
            beepFreqBlocking(BUZZER_ALERT_HZ, 45, 20);
            Serial.println("OK BEEP");
          }
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
  Serial.println("RP2040 SIMPLE RECOVERY + OPTIONAL TIMER BACKUPS (BENCH BUILD)");
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
    // Default to the VIOLET flight profile until saved settings load.
    // VIOLET defaults to 200 m main deployment.
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
        Serial.println("No valid saved settings found. Saving current defaults.");
        saveCometSettings();
      }

      if (!reclaimSpaceBeforeFlight()) {
        loggerFull = true;
        Serial.println("WARNING: LittleFS low on space for a new flight log.");
      }
      startStaticLogger();
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

  // Flight-critical output shutdown gets first CPU priority every pass.
  servicePyro(now);
  serviceBuzzer(now);

  // Keep newly-added UI/filesystem management activity out of the actual
  // pyro pulse. The proven sensor/logger/state-machine path below is left
  // unchanged from the known-good architecture.
  if (!pyroActive()) {
    serviceLoggerFullUI(now);
    handleModeSwitch();

    if (TEST_MODE) {
      handleSerialInput();
    }

    serviceDeferredSettingsSave(now);
  }

  if (state == FlightState::FAULT) {
    digitalWrite(MAIN_PIN, LOW);
    digitalWrite(DROGUE_PIN, LOW);
    delay(50);
    return;
  }

  readSensors();
  serviceFlightLogger(now);  // prelaunch RAM ring while IDLE; flight file after launch
  serviceStaticLogger(now);  // one hertz, last 3 non-flight power sessions

  if (state == FlightState::IDLE) {
    updateAccelBaseline(now);
  }

  // Feed NEW sensor samples only; the decision engine never counts the same
  // barometric reading multiple times or advances a dwell on stale samples.
  bool newBaro = latestBaroReadMs != 0 && latestBaroReadMs != processedBaroReadMs;
  bool newImu  = latestImuReadMs != 0 && latestImuReadMs != processedImuReadMs;
  if (newBaro) processedBaroReadMs = latestBaroReadMs;
  if (newImu) processedImuReadMs = latestImuReadMs;
  float recoveryAltitudeAGL = t_launch > 0 ? altAGL_m : alt_m;
  float accelDeltaG = accelMag_g - accelMagBaseline_g;
  comet_recovery::Result decision = recoveryLogic.update(
    now, recoveryAltitudeAGL, mplOK && newBaro,
    accelDeltaG, imuOK && accelBaselineValid && accelMethodsEnabled && newImu,
    mainAltM);

  if (decision.launchAccel || decision.launchBaro) {
    // Do not rebase at detection: baro can confirm launch 30m above the pad.
    launchAlt_m = 0.0f;
    altAGL_m = alt_m;
    maxAltAGL_m = altAGL_m;
    t_launch = now;
    state = FlightState::ASCENT;
    landingStableStartMs = 0;
    startLaunchDetectedSound();
    startNewLogIfSafe();
    sendTelemetry(decision.launchAccel ? "LAUNCH_ACCEL" : "LAUNCH_BARO");
  }

  if (decision.apogee) {
    // State advances to DROGUE_DEPLOYED only after GPIO request accepted.
    sendTelemetry(decision.apogeeMethod == comet_recovery::ApogeeMethod::PEAK_DROP
      ? "APOGEE_DROP" : "APOGEE_REVERSAL");
    if (!drogueFired && !pendingDrogueRequest) {
      pendingDrogueRequest = true;
      pendingDrogueReason = "APOGEE";
    }
  }

  if (decision.mainDue && !mainFired && !pendingMainRequest) {
    pendingMainRequest = true;
    pendingMainReason = decision.hardFloorDue ? "MAIN_HARD_FLOOR" : "MAIN_AGL_CROSS";
  }

  // OPTIONAL backup timers (default OFF). They use time since launch, exactly
  // like the older COMET profiles. Leave the primary baro recovery logic intact.
  // A 15 s drogue backup can command deployment during a longer ascent;
  // a 25 s main backup can override a higher selected main altitude.
  comet_backup::Due backup = comet_backup::evaluate(
    timerMethodsEnabled, t_launch > 0, now, t_launch,
    drogueBackupMs, mainBackupMs, drogueFired, mainFired);
  if (backup.drogue && !pendingDrogueRequest) {
    pendingDrogueRequest = true;
    pendingDrogueReason = "TIMER_BACKUP_DROGUE";
  }
  if (backup.main && !pendingMainRequest) {
    pendingMainRequest = true;
    pendingMainReason = "TIMER_BACKUP_MAIN_AFTER_DROGUE";
  }
  serviceRecoveryRequests();
  serviceLandingDetection(now); // 15s stable; independent of drogue/main success

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
