#pragma once
// COMET simple recovery decision logic (no Arduino dependencies).
// BENCH DEVELOPMENT ONLY. This module never controls hardware pins.
#include <stdint.h>
#include <math.h>

namespace comet_recovery {

constexpr float MAIN_ABSOLUTE_FLOOR_M = 200.0f; // fixed, NOT editable by profiles/serial
constexpr float BARO_LAUNCH_RISE_M = 30.0f;
constexpr uint32_t BARO_LAUNCH_DWELL_MS = 200;
constexpr float ACCEL_LAUNCH_DELTA_G = 1.2f;
constexpr uint32_t ACCEL_LAUNCH_DWELL_MS = 100;
constexpr float APOGEE_DROP_M = 3.0f;
constexpr uint32_t APOGEE_DROP_DWELL_MS = 600;
// Barometric altitude velocity: positive ascent -> negative descent.
// Equivalent to pressure trend changing negative -> positive.
constexpr float RISING_VZ_MPS = 1.0f;
constexpr float FALLING_VZ_MPS = -1.0f;
constexpr uint32_t REVERSAL_DWELL_MS = 250;
constexpr uint32_t FLOOR_CROSS_DWELL_MS = 200;
constexpr float FLOOR_CROSS_MIN_PEAK_M = 3.0f;
constexpr uint32_t BARO_FRESH_MAX_MS = 350;
constexpr uint32_t ACCEL_FRESH_MAX_MS = 150;

// These are the ONLY four operating states (BOOT/FAULT handled by main sketch).
enum class Phase : uint8_t { PAD, ASCENT, DESCENT, DONE };
enum class ApogeeMethod : uint8_t { NONE, PEAK_DROP, VZ_REVERSAL };
struct Result {
  bool launchAccel = false;
  bool launchBaro = false;
  bool apogee = false;
  ApogeeMethod apogeeMethod = ApogeeMethod::NONE;
  bool drogueDue = false;
  bool mainDue = false;
  bool hardFloorDue = false;
};

class Logic {
public:
  Phase phase = Phase::PAD;
  float filteredAGL = 0.0f;
  float maxAGL = 0.0f;
  float velocityMps = 0.0f;
  bool seenAscent = false;
  bool mainAccepted = false;

  static float target(float programmed) {
    return (isfinite(programmed) && programmed > MAIN_ABSOLUTE_FLOOR_M)
      ? programmed : MAIN_ABSOLUTE_FLOOR_M;
  }

  void reset() { *this = Logic(); }

  Result update(uint32_t now, float baroAGL, bool newBaro,
                float accelDeltaG, bool newAccel, float programmedMain) {
    Result r;
    bool baroValid = newBaro && isfinite(baroAGL);
    if (baroValid) {
      lastBaroMs = now;
      if (!altReady) { filteredAGL = baroAGL; altReady = true; }
      else { filteredAGL += 0.4f * (baroAGL - filteredAGL); }
      // 8 samples cover roughly 0.7 seconds at 10 Hz.
      hist[head] = {now, filteredAGL};
      head = (uint8_t)((head + 1u) % 8u);
      if (count < 8) ++count;
      if (count >= 2) {
        uint8_t oldest = (uint8_t)((head + 8u - count) % 8u);
        const Sample& a = hist[oldest];
        const Sample& b = hist[(head + 7u) % 8u];
        float seconds = (b.ms - a.ms) / 1000.0f;
        if (seconds > 0.001f) velocityMps = (b.alt - a.alt) / seconds;
      }
    }
    const bool fresh = altReady && (uint32_t)(now - lastBaroMs) <= BARO_FRESH_MAX_MS;

    if (phase == Phase::PAD) {
      if (newAccel && isfinite(accelDeltaG)) {
        lastAccelMs = now;
        if (accelDeltaG >= ACCEL_LAUNCH_DELTA_G) {
          if (!accelStartValid) { accelStart = now; accelStartValid = true; }
        } else accelStartValid = false;
      }
      if (baroValid) {
        if (filteredAGL >= BARO_LAUNCH_RISE_M) {
          if (!baroStartValid) { baroStart = now; baroStartValid = true; }
        } else baroStartValid = false;
      }
      r.launchAccel = accelStartValid && newAccel &&
                      (uint32_t)(now - lastAccelMs) <= ACCEL_FRESH_MAX_MS &&
                      (uint32_t)(now - accelStart) >= ACCEL_LAUNCH_DWELL_MS;
      r.launchBaro = baroStartValid && baroValid && fresh &&
                     (uint32_t)(now - baroStart) >= BARO_LAUNCH_DWELL_MS;
      if (r.launchAccel || r.launchBaro) {
        phase = Phase::ASCENT;
        maxAGL = filteredAGL;
        seenAscent = (velocityMps >= RISING_VZ_MPS);
        // Keep prelaunch filter history, necessary for late baro launch detection.
        accelStartValid = baroStartValid = false;
      }
      return r;
    }
    if (phase == Phase::DONE || !fresh || !baroValid) return r;

    if (filteredAGL > maxAGL) maxAGL = filteredAGL;
    if (velocityMps >= RISING_VZ_MPS) seenAscent = true;

    bool negative = seenAscent && velocityMps <= FALLING_VZ_MPS;
    if (negative) {
      if (!negStartValid) { negStart = now; negStartValid = true; }
    } else negStartValid = false;
    bool reversal = negative && (uint32_t)(now - negStart) >= REVERSAL_DWELL_MS;

    bool belowPeak = maxAGL - filteredAGL >= APOGEE_DROP_M;
    if (belowPeak) {
      if (!dropStartValid) { dropStart = now; dropStartValid = true; }
    } else dropStartValid = false;
    bool drop = belowPeak && (uint32_t)(now - dropStart) >= APOGEE_DROP_DWELL_MS;

    if (phase == Phase::ASCENT && (drop || reversal)) {
      phase = Phase::DESCENT;
      r.apogee = true;
      r.apogeeMethod = drop ? ApogeeMethod::PEAK_DROP : ApogeeMethod::VZ_REVERSAL;
      r.drogueDue = true;
    }

    if (!mainAccepted) {
      bool descending = phase == Phase::DESCENT || drop || reversal;
      // An independent 200 m crossing condition can still request main
      // even if a drogue request was rejected or normal apogee logic stalls.
      bool floorCross = maxAGL >= MAIN_ABSOLUTE_FLOOR_M + FLOOR_CROSS_MIN_PEAK_M &&
                        filteredAGL <= MAIN_ABSOLUTE_FLOOR_M && velocityMps < 0.0f;
      if (floorCross) {
        if (!floorStartValid) { floorStart = now; floorStartValid = true; }
      } else floorStartValid = false;
      bool floorConfirmed = floorCross && (uint32_t)(now - floorStart) >= FLOOR_CROSS_DWELL_MS;
      r.mainDue = (descending && filteredAGL <= target(programmedMain)) || floorConfirmed;
      r.hardFloorDue = floorConfirmed;
    }
    return r;
  }

  void markMainAccepted() { mainAccepted = true; phase = Phase::DONE; }
  void forceLaunch(float startingAltitude) {
    phase = Phase::ASCENT;
    if (!altReady) { altReady = true; filteredAGL = startingAltitude; }
    maxAGL = filteredAGL;
    seenAscent = false;
    accelStartValid = baroStartValid = false;
  }

private:
  struct Sample { uint32_t ms; float alt; };
  Sample hist[8] = {};
  uint8_t head = 0;
  uint8_t count = 0;
  bool altReady = false;
  uint32_t lastBaroMs = 0;
  uint32_t lastAccelMs = 0;
  uint32_t accelStart = 0, baroStart = 0, negStart = 0, dropStart = 0, floorStart = 0;
  bool accelStartValid = false, baroStartValid = false;
  bool negStartValid = false, dropStartValid = false, floorStartValid = false;
};
} // namespace comet_recovery
