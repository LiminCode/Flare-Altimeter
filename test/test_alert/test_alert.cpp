// Tests for the alert engine.
//
// These are the project's own scenarios, not API coverage. Each one corresponds to
// a decision made during design, and several correspond to a way the device could
// hurt someone: screaming on the ground, going quiet on a fast canopy, or shouting
// FLARE NOW because the laser blinked at 8 metres.
//
// Time is an argument, so a whole landing runs in microseconds and the awkward
// cases -- a dropout mid-descent, a thermal, a 6.7 m/s canopy -- are staged by
// typing numbers instead of jumping out of an aircraft.
#include <unity.h>

#include <math.h>

#include "alert_engine.h"

using flare::AlertConfig;
using flare::AlertEngine;
using flare::AlertOutput;
using flare::Stage;

namespace {

constexpr uint32_t kTickMs = 10;   // 100 Hz, the real laser rate

struct Run {
  int beeps = 0;
  int firstHz = 0;
  int lastHz = 0;
  float firstBeepCm = 0.0f;
  float lastBeepCm = 0.0f;
  bool pitchMonotonic = true;
  uint32_t flareAtMs = 0;
  uint32_t silentAtMs = 0;
  bool sawFlare = false;
  bool sawSilentAfterFlare = false;
};

// Fly a constant-rate descent and collect what the engine did.
Run descend(AlertEngine &e, float fromCm, float toCm, float cmPerSec,
            uint32_t startMs = 1000) {
  Run r;
  const uint32_t durMs = (uint32_t)((fromCm - toCm) / cmPerSec * 1000.0f);
  int prevHz = 0;
  for (uint32_t t = 0; t <= durMs; t += kTickMs) {
    const float h = fromCm - cmPerSec * (t / 1000.0f);
    const AlertOutput o = e.update(startMs + t, h, true);
    if (o.beepFired) {
      if (r.beeps == 0) {
        r.firstHz = o.beepHz;
        r.firstBeepCm = o.beepAtCm;
      }
      if (prevHz && o.beepHz <= prevHz) r.pitchMonotonic = false;
      prevHz = o.beepHz;
      r.lastHz = o.beepHz;
      r.lastBeepCm = o.beepAtCm;
      r.beeps++;
    }
    if (o.stage == Stage::FlareNow && !r.sawFlare) {
      r.sawFlare = true;
      r.flareAtMs = startMs + t;
    }
  }
  return r;
}

AlertConfig defaultCfg() { return AlertConfig{}; }

}  // namespace

void setUp() {}
void tearDown() {}

// --- arming ----------------------------------------------------------------

void test_silent_sitting_on_the_ground() {
  // The boarding-to-exit scenario: a foot-mounted sensor reads the floor at
  // ~10 cm continuously. Height alone cannot distinguish that from "below the
  // changeover height", so without arming the device howls all the way to the
  // aircraft door.
  AlertEngine e(defaultCfg());
  for (uint32_t t = 0; t < 30000; t += kTickMs) {
    const AlertOutput o = e.update(1000 + t, 10.0f, true);
    TEST_ASSERT_EQUAL_UINT16(0, o.toneHz);
    TEST_ASSERT_FALSE(o.armed);
  }
}

void test_does_not_arm_when_high_but_stationary() {
  // Hanging under canopy in strong lift, or the unit sitting on a shelf pointing
  // at a distant floor. High, valid, continuous -- but not descending.
  AlertEngine e(defaultCfg());
  for (uint32_t t = 0; t < 5000; t += kTickMs) {
    const AlertOutput o = e.update(1000 + t, 1000.0f, true);
    TEST_ASSERT_FALSE(o.armed);
    TEST_ASSERT_EQUAL_UINT16(0, o.toneHz);
  }
}

void test_does_not_arm_when_descending_but_too_low() {
  // Walking down a slope, or stepping off a kerb: descending, continuous data,
  // but never high enough to be a landing.
  AlertEngine e(defaultCfg());
  float h = 700.0f;
  for (uint32_t t = 0; t < 5000; t += kTickMs) {
    h -= 0.5f;
    if (h < 50.0f) h = 50.0f;
    const AlertOutput o = e.update(1000 + t, h, true);
    TEST_ASSERT_FALSE(o.armed);
  }
}

void test_arms_after_the_confirmation_window() {
  AlertEngine e(defaultCfg());
  bool armedYet = false;
  uint32_t armedAt = 0;
  for (uint32_t t = 0; t < 1000; t += kTickMs) {
    const float h = 1300.0f - 4.5f * t;    // 4.5 m/s in cm/ms
    const AlertOutput o = e.update(1000 + t, h, true);
    if (o.armed && !armedYet) {
      armedYet = true;
      armedAt = t;
    }
  }
  TEST_ASSERT_TRUE(armedYet);
  TEST_ASSERT_UINT32_WITHIN(30, 300, armedAt);
}

void test_arms_on_a_fast_canopy_that_is_already_low_by_window_end() {
  // The reason the height test looks at the START of the window. At 6.7 m/s a
  // unit acquiring at 13 m is below 8 m one second later; an end-of-window test
  // would leave the fastest canopies with no warning at all.
  AlertConfig cfg = defaultCfg();
  cfg.armConfirmMs = 1000;               // exaggerate to make the point sharp
  AlertEngine e(cfg);
  bool armed = false;
  for (uint32_t t = 0; t <= 1200; t += kTickMs) {
    const float h = 1300.0f - 6.7f * t;  // 6.7 m/s => 6.3 m at t=1000
    const AlertOutput o = e.update(1000 + t, h, true);
    if (o.armed) armed = true;
  }
  TEST_ASSERT_TRUE(armed);
}

void test_data_gap_breaks_the_arming_window() {
  // Continuity is part of the contract: a gap means we can no longer claim to
  // have been tracking this descent.
  AlertEngine e(defaultCfg());
  for (uint32_t t = 0; t < 200; t += kTickMs) {
    e.update(1000 + t, 1300.0f - 4.5f * t, true);
  }
  // Long dropout, past the grace period.
  for (uint32_t t = 200; t < 800; t += kTickMs) {
    e.update(1000 + t, 0.0f, false);
  }
  TEST_ASSERT_FALSE(e.armed());
}

void test_a_stale_low_arming_window_does_not_delay_a_real_descent() {
  // Found by diffing firmware behaviour before and after the refactor. Sitting on
  // valid low data (a bench target, or a low pass over trees) leaves a window open
  // with a low starting height. When a genuine descent then begins from altitude,
  // that stale window must not have to expire first -- it costs up to a full
  // confirmation window, which is a whole beep step of a descent that is already
  // only two seconds long.
  AlertEngine e(defaultCfg());
  for (uint32_t t = 0; t < 2000; t += kTickMs) {
    e.update(1000 + t, 440.0f, true);      // low, valid, continuous
  }
  TEST_ASSERT_FALSE(e.armed());

  uint32_t armedAt = 0;
  for (uint32_t t = 0; t < 1000; t += kTickMs) {
    const float h = 1300.0f - 4.5f * t;
    if (e.update(3000 + t, h, true).armed) { armedAt = t; break; }
  }
  TEST_ASSERT_UINT32_WITHIN(40, 300, armedAt);
}

// --- the beep ladder -------------------------------------------------------

void test_beep_count_matches_the_height_travelled() {
  AlertConfig cfg = defaultCfg();
  AlertEngine e(cfg);
  const Run r = descend(e, 1300.0f, 200.0f, 450.0f);
  // Arming eats the first ~300 ms (~135 cm at 4.5 m/s), so beeps start near
  // 1150 and run to 250: about 19 steps of 50 cm.
  TEST_ASSERT_INT_WITHIN(2, 19, r.beeps);
  TEST_ASSERT_TRUE(r.firstBeepCm > 1050.0f);
  TEST_ASSERT_TRUE(r.lastBeepCm >= 200.0f);
}

void test_beep_rate_equals_descent_rate_over_step() {
  // The core property of the whole model. Doubling the descent rate must double
  // the beep rate, with no v ever being computed.
  AlertConfig cfg = defaultCfg();
  cfg.armConfirmMs = 0;
  cfg.armMinDropCm = 0.0f;

  AlertEngine slow(cfg);
  const Run a = descend(slow, 1300.0f, 300.0f, 450.0f);
  AlertEngine fast(cfg);
  const Run b = descend(fast, 1300.0f, 300.0f, 900.0f);

  // Same height travelled => same number of beeps, regardless of speed...
  TEST_ASSERT_INT_WITHIN(1, a.beeps, b.beeps);
  // ...which is exactly what makes the RATE proportional to speed.
  const float slowSecs = (1300.0f - 300.0f) / 450.0f;
  const float fastSecs = (1300.0f - 300.0f) / 900.0f;
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 9.0f, a.beeps / slowSecs);
  TEST_ASSERT_FLOAT_WITHIN(2.0f, 18.0f, b.beeps / fastSecs);
}

void test_step_size_changes_the_beep_count_proportionally() {
  AlertConfig cfg = defaultCfg();
  cfg.armConfirmMs = 0;
  cfg.armMinDropCm = 0.0f;

  AlertEngine fine(cfg);
  const int n50 = descend(fine, 1300.0f, 300.0f, 450.0f).beeps;

  cfg.beepStepCm = 100.0f;
  AlertEngine coarse(cfg);
  const int n100 = descend(coarse, 1300.0f, 300.0f, 450.0f).beeps;

  TEST_ASSERT_INT_WITHIN(1, n50 / 2, n100);
}

void test_pitch_rises_monotonically_and_clamps_at_both_ends() {
  AlertEngine e(defaultCfg());
  const Run r = descend(e, 1300.0f, 210.0f, 450.0f);
  TEST_ASSERT_TRUE(r.pitchMonotonic);
  TEST_ASSERT_TRUE(r.firstHz >= 700);
  TEST_ASSERT_TRUE(r.lastHz <= 3500);

  const AlertConfig cfg = defaultCfg();
  // Above the reference top and below the changeover, pitch must not run away.
  TEST_ASSERT_EQUAL_UINT16(700, flare::pitchForHeightHz(cfg, 2000.0f));
  TEST_ASSERT_EQUAL_UINT16(3500, flare::pitchForHeightHz(cfg, 200.0f));
  TEST_ASSERT_EQUAL_UINT16(3500, flare::pitchForHeightHz(cfg, 50.0f));
}

void test_no_double_beep_on_the_same_step() {
  // Two properties at once. Hovering exactly on a step boundary must not
  // machine-gun; and the 300 cm instantaneous drop at the start must cost one
  // beep, not one per tick until the ladder catches up. The second is what a
  // reacquisition after a dropout looks like, and a burst there would claim
  // several 50 cm descents that were never measured.
  AlertConfig cfg = defaultCfg();
  cfg.armConfirmMs = 0;
  cfg.armMinDropCm = 0.0f;
  AlertEngine e(cfg);
  e.update(1000, 1300.0f, true);
  e.update(1010, 1000.0f, true);          // arms, seats the ladder, beeps
  int extra = 0;
  for (uint32_t t = 20; t < 2000; t += kTickMs) {
    if (e.update(1000 + t, 1000.0f, true).beepFired) extra++;
  }
  TEST_ASSERT_EQUAL_INT(0, extra);
}

void test_climbing_back_up_re_seats_the_ladder() {
  // A thermal, a bad frame, or the foot swinging over different ground. Without
  // re-seating, the ladder stays latched low and silently swallows every beep on
  // the way back down -- no error, just silence where the warning should be.
  AlertConfig cfg = defaultCfg();
  cfg.armConfirmMs = 0;
  cfg.armMinDropCm = 0.0f;
  AlertEngine e(cfg);

  e.update(1000, 1300.0f, true);
  descend(e, 1300.0f, 600.0f, 450.0f, 1000);

  // Rise well past two steps.
  for (uint32_t t = 0; t < 500; t += kTickMs) {
    e.update(5000 + t, 600.0f + t * 0.5f, true);
  }
  // Coming back down must beep again.
  int beeps = 0;
  for (uint32_t t = 0; t < 2000; t += kTickMs) {
    const float h = 850.0f - 0.45f * t;
    if (e.update(6000 + t, h, true).beepFired) beeps++;
  }
  TEST_ASSERT_TRUE(beeps >= 3);
}

// --- changeover and stop ---------------------------------------------------

void test_continuous_tone_at_and_below_the_changeover() {
  AlertConfig cfg = defaultCfg();
  cfg.armConfirmMs = 0;
  cfg.armMinDropCm = 0.0f;
  AlertEngine e(cfg);
  e.update(1000, 1300.0f, true);
  const AlertOutput o = e.update(1010, 199.0f, true);
  TEST_ASSERT_EQUAL_UINT16(3500, o.toneHz);
  TEST_ASSERT_TRUE(o.stage == Stage::FlareNow);
}

void test_changeover_height_is_configurable() {
  AlertConfig cfg = defaultCfg();
  cfg.armConfirmMs = 0;
  cfg.armMinDropCm = 0.0f;
  cfg.continuousAtCm = 450.0f;
  AlertEngine e(cfg);
  e.update(1000, 1300.0f, true);
  TEST_ASSERT_EQUAL_UINT16(3500, e.update(1010, 440.0f, true).toneHz);

  AlertConfig low = cfg;
  low.continuousAtCm = 150.0f;
  AlertEngine e2(low);
  e2.update(1000, 1300.0f, true);
  const AlertOutput o = e2.update(1010, 440.0f, true);
  TEST_ASSERT_TRUE(o.stage != Stage::FlareNow);
}

void test_auto_stop_after_two_seconds_on_the_ground() {
  AlertConfig cfg = defaultCfg();
  cfg.armConfirmMs = 0;
  cfg.armMinDropCm = 0.0f;
  AlertEngine e(cfg);
  e.update(1000, 1300.0f, true);
  e.update(1010, 500.0f, true);

  const uint32_t landed = 2000;
  // Just under the hold: still sounding.
  for (uint32_t t = 0; t <= 1990; t += kTickMs) {
    const AlertOutput o = e.update(landed + t, 10.0f, true);
    TEST_ASSERT_EQUAL_UINT16(3500, o.toneHz);
  }
  // Past it: silent and disarmed.
  const AlertOutput o = e.update(landed + 2100, 10.0f, true);
  TEST_ASSERT_EQUAL_UINT16(0, o.toneHz);
  TEST_ASSERT_FALSE(o.armed);
}

// --- dropouts: the dangerous one -------------------------------------------

void test_short_dropout_holds_the_last_valid_height_not_minus_one() {
  // REGRESSION. The monolithic version fed the caller's height variable straight
  // through, and the caller sets it to -1 when the laser has no echo. A 200 ms
  // dropout at 8 m therefore pushed h = -1 into the ladder, which is below the
  // changeover, and the "smooth over a lost frame" grace period produced a burst
  // of full-volume FLARE NOW at 8 metres.
  AlertConfig cfg = defaultCfg();
  cfg.armConfirmMs = 0;
  cfg.armMinDropCm = 0.0f;
  AlertEngine e(cfg);
  e.update(1000, 1300.0f, true);
  e.update(1010, 800.0f, true);
  TEST_ASSERT_TRUE(e.armed());

  for (uint32_t t = 20; t < 220; t += kTickMs) {
    // Caller passes -1 alongside valid=false, exactly as the firmware does.
    const AlertOutput o = e.update(1000 + t, -1.0f, false);
    TEST_ASSERT_NOT_EQUAL(Stage::FlareNow, o.stage);
    TEST_ASSERT_TRUE(o.toneHz != 3500);
  }
}

void test_long_dropout_goes_silent_rather_than_holding_a_stale_alarm() {
  AlertConfig cfg = defaultCfg();
  cfg.armConfirmMs = 0;
  cfg.armMinDropCm = 0.0f;
  AlertEngine e(cfg);
  e.update(1000, 1300.0f, true);
  e.update(1010, 300.0f, true);
  TEST_ASSERT_TRUE(e.armed());

  const AlertOutput o = e.update(1010 + 500, -1.0f, false);
  TEST_ASSERT_EQUAL_UINT16(0, o.toneHz);
  TEST_ASSERT_FALSE(o.armed);
}

void test_stage_names_are_stable() {
  // The host viewer indexes its labels off these values.
  TEST_ASSERT_EQUAL_INT(0, (int)Stage::Silent);
  TEST_ASSERT_EQUAL_INT(4, (int)Stage::FlareNow);
  TEST_ASSERT_EQUAL_STRING("FLARE NOW", flare::stageName(Stage::FlareNow));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_silent_sitting_on_the_ground);
  RUN_TEST(test_does_not_arm_when_high_but_stationary);
  RUN_TEST(test_does_not_arm_when_descending_but_too_low);
  RUN_TEST(test_arms_after_the_confirmation_window);
  RUN_TEST(test_arms_on_a_fast_canopy_that_is_already_low_by_window_end);
  RUN_TEST(test_data_gap_breaks_the_arming_window);
  RUN_TEST(test_a_stale_low_arming_window_does_not_delay_a_real_descent);
  RUN_TEST(test_beep_count_matches_the_height_travelled);
  RUN_TEST(test_beep_rate_equals_descent_rate_over_step);
  RUN_TEST(test_step_size_changes_the_beep_count_proportionally);
  RUN_TEST(test_pitch_rises_monotonically_and_clamps_at_both_ends);
  RUN_TEST(test_no_double_beep_on_the_same_step);
  RUN_TEST(test_climbing_back_up_re_seats_the_ladder);
  RUN_TEST(test_continuous_tone_at_and_below_the_changeover);
  RUN_TEST(test_changeover_height_is_configurable);
  RUN_TEST(test_auto_stop_after_two_seconds_on_the_ground);
  RUN_TEST(test_short_dropout_holds_the_last_valid_height_not_minus_one);
  RUN_TEST(test_long_dropout_goes_silent_rather_than_holding_a_stale_alarm);
  RUN_TEST(test_stage_names_are_stable);
  return UNITY_END();
}
