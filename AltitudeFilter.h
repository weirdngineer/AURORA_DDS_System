#pragma once
// COMET diagnostic altitude filter. No pins, sensor calls or heap allocations.
// Median of up to 3 genuinely new baro measurements -> single-pole IIR.
// The filter is meant to suppress isolated barometer spikes. Development only:
// verify altitude lag against OpenRocket and sensor replay before flight.
#include <math.h>
#include <stdint.h>
namespace comet_alt {
struct Filter {
  float history[3] = {};
  float output = 0;
  uint8_t used = 0;
  uint8_t index = 0;
  bool ready = false;
  void reset(float initial) {
    output = initial;
    for (uint8_t i=0;i<3;i++) history[i] = initial;
    used=0; index=0; ready=true;
  }
  float update(float raw) {
    if (!isfinite(raw)) return output;
    if (!ready) { reset(raw); return output; }
    history[index]=raw;
    index=(uint8_t)((index+1)%3);
    if (used<3) used++;
    float median=raw;
    if (used==3) {
      const float a=history[0],b=history[1],c=history[2];
      median = fmaxf(fminf(a,b),fminf(fmaxf(a,b),c));
    }
    // Alpha 0.65 keeps one-pole group delay modest at actual ~10 Hz.
    output += 0.65f * (median-output);
    return output;
  }
};
}
