// Tests for the TF02-Pro frame parser.
//
// The scenarios are the ones that actually happen on a serial line in a vibrating
// foot mount: a dropped byte, a stray header byte in payload data, a frame split
// across reads, and the sensor honestly reporting a return it does not trust.
#include <unity.h>

#include "tf02_parser.h"

using flare::LidarReading;
using flare::Tf02Parser;

namespace {

// Build a well-formed frame with a correct checksum.
void makeFrame(uint8_t *f, uint16_t dist, uint16_t strength, uint16_t rawTemp) {
  f[0] = 0x59;
  f[1] = 0x59;
  f[2] = dist & 0xFF;
  f[3] = dist >> 8;
  f[4] = strength & 0xFF;
  f[5] = strength >> 8;
  f[6] = rawTemp & 0xFF;
  f[7] = rawTemp >> 8;
  uint16_t sum = 0;
  for (int i = 0; i < 8; i++) sum += f[i];
  f[8] = sum & 0xFF;
}

// Feed bytes, return how many frames completed and the last reading.
int feedAll(Tf02Parser &p, const uint8_t *bytes, int n, LidarReading &last) {
  int got = 0;
  LidarReading r{};
  for (int i = 0; i < n; i++) {
    if (p.feed(bytes[i], r)) {
      got++;
      last = r;
    }
  }
  return got;
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_valid_frame_decodes() {
  uint8_t f[9];
  makeFrame(f, 198, 1668, (uint16_t)((27.0f + 256.0f) * 8.0f));
  Tf02Parser p;
  LidarReading r{};
  TEST_ASSERT_EQUAL_INT(1, feedAll(p, f, 9, r));
  TEST_ASSERT_EQUAL_INT32(198, r.distCm);
  TEST_ASSERT_EQUAL_UINT16(1668, r.strength);
  TEST_ASSERT_FLOAT_WITHIN(0.2f, 27.0f, r.tempC);
  TEST_ASSERT_EQUAL_UINT32(1, p.frames());
  TEST_ASSERT_EQUAL_UINT32(0, p.badChecksums());
}

void test_bad_checksum_is_rejected_and_counted() {
  uint8_t f[9];
  makeFrame(f, 198, 1668, 2264);
  f[8] ^= 0xFF;
  Tf02Parser p;
  LidarReading r{};
  TEST_ASSERT_EQUAL_INT(0, feedAll(p, f, 9, r));
  TEST_ASSERT_EQUAL_UINT32(0, p.frames());
  TEST_ASSERT_EQUAL_UINT32(1, p.badChecksums());
}

void test_parser_resyncs_after_a_dropped_byte() {
  // The property that matters most on a noisy line: losing one byte must cost one
  // frame, not every frame from then on. A parser that stays one byte out of
  // phase fails silently and forever.
  uint8_t f[9];
  makeFrame(f, 500, 900, 2264);
  Tf02Parser p;
  LidarReading r{};

  uint8_t truncated[8];
  for (int i = 0; i < 8; i++) truncated[i] = f[i + 1];   // drop the first byte
  feedAll(p, truncated, 8, r);

  TEST_ASSERT_EQUAL_INT(1, feedAll(p, f, 9, r));
  TEST_ASSERT_EQUAL_INT32(500, r.distCm);
}

void test_stray_header_byte_does_not_desync() {
  uint8_t f[9];
  makeFrame(f, 321, 800, 2264);
  Tf02Parser p;
  LidarReading r{};
  const uint8_t junk[] = {0x59, 0x01, 0x59, 0xAB, 0x00};
  feedAll(p, junk, 5, r);
  TEST_ASSERT_EQUAL_INT(1, feedAll(p, f, 9, r));
  TEST_ASSERT_EQUAL_INT32(321, r.distCm);
}

void test_frame_split_across_reads() {
  uint8_t f[9];
  makeFrame(f, 777, 1200, 2264);
  Tf02Parser p;
  LidarReading r{};
  TEST_ASSERT_EQUAL_INT(0, feedAll(p, f, 4, r));
  TEST_ASSERT_EQUAL_INT(1, feedAll(p, f + 4, 5, r));
  TEST_ASSERT_EQUAL_INT32(777, r.distCm);
}

void test_weak_return_is_reported_as_no_range() {
  // Strength below 100: too far, too dark or too oblique. The sensor still sends
  // a distance and it still looks like a number -- that is the trap.
  uint8_t f[9];
  makeFrame(f, 1200, 42, 2264);
  Tf02Parser p;
  LidarReading r{};
  TEST_ASSERT_EQUAL_INT(1, feedAll(p, f, 9, r));
  TEST_ASSERT_EQUAL_INT32(-1, r.distCm);
  TEST_ASSERT_EQUAL_UINT16(42, r.strength);
}

void test_saturated_return_is_reported_as_no_range() {
  uint8_t f[9];
  makeFrame(f, 30, 65535, 2264);
  Tf02Parser p;
  LidarReading r{};
  TEST_ASSERT_EQUAL_INT(1, feedAll(p, f, 9, r));
  TEST_ASSERT_EQUAL_INT32(-1, r.distCm);
}

void test_ffff_distance_is_reported_as_no_range() {
  uint8_t f[9];
  makeFrame(f, 0xFFFF, 1000, 2264);
  Tf02Parser p;
  LidarReading r{};
  TEST_ASSERT_EQUAL_INT(1, feedAll(p, f, 9, r));
  TEST_ASSERT_EQUAL_INT32(-1, r.distCm);
}

void test_rejected_range_still_counts_as_a_frame() {
  // "Is the sensor talking to us" and "can we see the ground" are different
  // questions. Conflating them makes a wiring fault look like a dark target.
  uint8_t f[9];
  makeFrame(f, 1200, 42, 2264);
  Tf02Parser p;
  LidarReading r{};
  feedAll(p, f, 9, r);
  TEST_ASSERT_EQUAL_UINT32(1, p.frames());
}

void test_back_to_back_frames() {
  uint8_t f[18];
  makeFrame(f, 100, 500, 2264);
  makeFrame(f + 9, 150, 600, 2264);
  Tf02Parser p;
  LidarReading r{};
  TEST_ASSERT_EQUAL_INT(2, feedAll(p, f, 18, r));
  TEST_ASSERT_EQUAL_INT32(150, r.distCm);
  TEST_ASSERT_EQUAL_UINT32(2, p.frames());
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_valid_frame_decodes);
  RUN_TEST(test_bad_checksum_is_rejected_and_counted);
  RUN_TEST(test_parser_resyncs_after_a_dropped_byte);
  RUN_TEST(test_stray_header_byte_does_not_desync);
  RUN_TEST(test_frame_split_across_reads);
  RUN_TEST(test_weak_return_is_reported_as_no_range);
  RUN_TEST(test_saturated_return_is_reported_as_no_range);
  RUN_TEST(test_ffff_distance_is_reported_as_no_range);
  RUN_TEST(test_rejected_range_still_counts_as_a_frame);
  RUN_TEST(test_back_to_back_frames);
  return UNITY_END();
}
