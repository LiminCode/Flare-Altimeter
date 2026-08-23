// Tests for settings validation.
//
// These guard the moment a value crosses back in from NVS, which is the only
// point at which a setting written by an older build, or written half-way through
// a power loss, can be caught.
#include <unity.h>

#include <math.h>

#include "flare_config.h"

void setUp() {}
void tearDown() {}

void test_changeover_clamps_to_the_adjustable_range() {
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, flare::clampContinuousAtCm(-500.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, flare::clampContinuousAtCm(0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 600.0f, flare::clampContinuousAtCm(99999.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 200.0f, flare::clampContinuousAtCm(200.0f));
}

void test_changeover_rejects_nan() {
  // An uninitialised or corrupt NVS float can be NaN, and every comparison
  // against NaN is false -- so a naive clamp lets it straight through and the
  // changeover height silently stops working.
  const float v = flare::clampContinuousAtCm(NAN);
  TEST_ASSERT_TRUE(v == v);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, v);
}

void test_changeover_adjust_steps_and_saturates() {
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 225.0f, flare::adjustContinuousAtCm(200.0f, true));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 175.0f, flare::adjustContinuousAtCm(200.0f, false));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, flare::adjustContinuousAtCm(100.0f, false));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 600.0f, flare::adjustContinuousAtCm(600.0f, true));
}

void test_beep_step_snaps_to_a_known_choice() {
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 25.0f, flare::snapBeepStepCm(25.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, flare::snapBeepStepCm(50.2f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, flare::snapBeepStepCm(99.9f));
}

void test_unrecognised_beep_step_falls_back_to_the_default() {
  // A stale key or a corrupt record must not become the ladder spacing: a step of
  // 0 divides the whole descent by nothing, a step of 3000 means one beep ever,
  // and neither reports an error.
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, flare::snapBeepStepCm(0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, flare::snapBeepStepCm(-7.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, flare::snapBeepStepCm(3000.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, flare::snapBeepStepCm(NAN));
}

void test_beep_step_cycles_and_wraps() {
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, flare::cycleBeepStepCm(25.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, flare::cycleBeepStepCm(50.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 25.0f, flare::cycleBeepStepCm(100.0f));
}

void test_beeps_per_second_matches_the_published_numbers() {
  // The figures quoted in the design docs, so a change to the step choices that
  // invalidates them fails here rather than in a field test.
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 9.0f, flare::beepsPerSecond(50.0f, 450.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 13.4f, flare::beepsPerSecond(50.0f, 670.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 4.5f, flare::beepsPerSecond(100.0f, 450.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 18.0f, flare::beepsPerSecond(25.0f, 450.0f));
}

void test_beeps_per_second_survives_a_zero_step() {
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, flare::beepsPerSecond(0.0f, 450.0f));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_changeover_clamps_to_the_adjustable_range);
  RUN_TEST(test_changeover_rejects_nan);
  RUN_TEST(test_changeover_adjust_steps_and_saturates);
  RUN_TEST(test_beep_step_snaps_to_a_known_choice);
  RUN_TEST(test_unrecognised_beep_step_falls_back_to_the_default);
  RUN_TEST(test_beep_step_cycles_and_wraps);
  RUN_TEST(test_beeps_per_second_matches_the_published_numbers);
  RUN_TEST(test_beeps_per_second_survives_a_zero_step);
  return UNITY_END();
}
