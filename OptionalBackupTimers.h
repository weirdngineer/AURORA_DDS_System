#pragma once
// Optional launch-relative timer backups. Never writes GPIO or changes flight state.
// Default OFF; upstream firmware setting must explicitly opt in.
#include <stdint.h>

namespace comet_backup {
struct Due { bool drogue = false; bool main = false; };
static inline Due evaluate(bool enabled, bool hasLaunched, uint32_t now,
                           uint32_t launchTimeMs, uint32_t drogueDelayMs,
                           uint32_t mainDelayMs, bool drogueCommanded,
                           bool mainCommanded) {
  Due out;
  if (!enabled || !hasLaunched || mainCommanded) return out;
  uint32_t elapsed = now - launchTimeMs; // safe across millis() wraparound
  if (!drogueCommanded && elapsed >= drogueDelayMs) out.drogue = true;
  // Measured from launch, not from drogue. Main timer is dependent on a
  // previously accepted drogue command (not proof of successful deployment).
  if (drogueCommanded && elapsed >= mainDelayMs) out.main = true;
  return out;
}
} // namespace comet_backup
