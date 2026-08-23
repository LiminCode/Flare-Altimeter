// Tests for the attitude/geometry maths.
//
// The scenarios here are the ones the project actually depends on being right,
// not a mechanical sweep of the API: the mounting reference zeroing correctly,
// the multiplication order that fails by 2% if reversed, the NaN guard, and the
// "no reading must never look like zero height" rule.
#include <unity.h>

#include <math.h>

#include "flare_math.h"

using flare::Quat;

namespace {

constexpr float kDeg = 3.14159265358979f / 180.0f;

Quat axisAngle(float ax, float ay, float az, float deg) {
  const float a = deg * kDeg / 2.0f;
  const float n = sqrtf(ax * ax + ay * ay + az * az);
  const float s = sinf(a) / n;
  return {cosf(a), ax * s, ay * s, az * s};
}

}  // namespace

void setUp() {}
void tearDown() {}

// --- tilt ------------------------------------------------------------------

void test_tilt_identity_is_zero() {
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f,
                           flare::tiltFromVerticalDeg({1, 0, 0, 0}));
}

void test_tilt_matches_known_rotations() {
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f,
      flare::tiltFromVerticalDeg(axisAngle(1, 0, 0, 20)));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 90.0f,
      flare::tiltFromVerticalDeg(axisAngle(0, 1, 0, 90)));
}

void test_tilt_is_zero_about_the_vertical_axis() {
  // Spinning about world Z changes yaw but not tilt. If this ever fails, the
  // 40 deg validity gate would start rejecting perfectly level frames purely
  // because the wearer turned.
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f,
      flare::tiltFromVerticalDeg(axisAngle(0, 0, 1, 137)));
}

void test_tilt_never_returns_nan_out_of_domain() {
  // Float error can push R33 slightly past 1.0; without the clamp acosf() gives
  // NaN, which then silently poisons every downstream number.
  const Quat almost = {1.0000001f, 0.0f, 0.0f, 0.0f};
  const float t = flare::tiltFromVerticalDeg(almost);
  TEST_ASSERT_TRUE(t == t);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, t);
}

// --- mounting reference ----------------------------------------------------

void test_reference_zeroes_the_pose_it_was_taken_at() {
  const Quat mounted = flare::qMul(axisAngle(0, 0, 1, 35), axisAngle(1, 0, 0, 20));
  // Uncorrected, this mounting is 20 deg off vertical.
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, flare::tiltFromVerticalDeg(mounted));
  // Referenced against itself, it is level by definition.
  const Quat rel = flare::applyReference(mounted, mounted);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, flare::tiltFromVerticalDeg(rel));
}

void test_reference_multiplication_order_is_not_reversed() {
  // The whole point of this test. Swinging 15 deg from the reference must read
  // 15.00, not 14.71 -- the value the reversed order produces. That error is far
  // too small to notice on a log and permanent enough to bias every height.
  const Quat ref = flare::qMul(axisAngle(0, 0, 1, 35), axisAngle(1, 0, 0, 20));
  const Quat now = flare::qMul(axisAngle(1, 0, 0, 15), ref);

  const float correct = flare::tiltFromVerticalDeg(flare::applyReference(now, ref));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 15.0f, correct);

  const float reversed =
      flare::tiltFromVerticalDeg(flare::qMul(flare::qConj(ref), now));
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 14.71f, reversed);
  TEST_ASSERT_TRUE(fabsf(correct - reversed) > 0.2f);
}

void test_identity_reference_changes_nothing() {
  const Quat q = axisAngle(1, 2, 3, 47);
  const Quat rel = flare::applyReference(q, {1, 0, 0, 0});
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, q.w, rel.w);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, q.x, rel.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, q.y, rel.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, q.z, rel.z);
}

void test_quat_times_its_conjugate_is_identity() {
  const Quat q = axisAngle(0.3f, -0.7f, 0.2f, 63);
  const Quat i = flare::qMul(q, flare::qConj(q));
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, i.w);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, i.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, i.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, i.z);
}

// --- slant range -> height -------------------------------------------------

void test_height_equals_distance_when_level() {
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 198.0f, flare::heightFromSlantCm(198, 0.0f));
}

void test_height_applies_cosine() {
  // The measured desk case: 198 cm at 9.9 deg read 195 cm on the console.
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 195.0f, flare::heightFromSlantCm(198, 9.9f));
  // At the 40 deg gate the correction is 23%, which is why the gate exists.
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 766.0f, flare::heightFromSlantCm(1000, 40.0f));
}

void test_no_reading_stays_negative_never_zero() {
  // Zero would read downstream as "on the ground" -- the most dangerous default
  // this device could have.
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, flare::heightFromSlantCm(-1, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, flare::heightFromSlantCm(-1, 35.0f));
}

// --- euler -----------------------------------------------------------------

void test_rpy_of_identity_is_all_zero() {
  float r, p, y;
  flare::quatToRPY({1, 0, 0, 0}, r, p, y);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, r);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, p);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, y);
}

void test_rpy_recovers_a_pure_roll() {
  float r, p, y;
  flare::quatToRPY(axisAngle(1, 0, 0, 30), r, p, y);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 30.0f, r / kDeg);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, p / kDeg);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_tilt_identity_is_zero);
  RUN_TEST(test_tilt_matches_known_rotations);
  RUN_TEST(test_tilt_is_zero_about_the_vertical_axis);
  RUN_TEST(test_tilt_never_returns_nan_out_of_domain);
  RUN_TEST(test_reference_zeroes_the_pose_it_was_taken_at);
  RUN_TEST(test_reference_multiplication_order_is_not_reversed);
  RUN_TEST(test_identity_reference_changes_nothing);
  RUN_TEST(test_quat_times_its_conjugate_is_identity);
  RUN_TEST(test_height_equals_distance_when_level);
  RUN_TEST(test_height_applies_cosine);
  RUN_TEST(test_no_reading_stays_negative_never_zero);
  RUN_TEST(test_rpy_of_identity_is_all_zero);
  RUN_TEST(test_rpy_recovers_a_pure_roll);
  return UNITY_END();
}
