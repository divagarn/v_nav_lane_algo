#include <gtest/gtest.h>

#include "radar_fusion/byte_tracker.hpp"

using namespace radar_fusion;

/**
 * @brief Test detection: 40 x 100 px "person" box.
 * @param[in] x, y  top-left corner [px]
 * @param[in] s     score
 * @param[in] cls   class id
 * @return detection
 */
static Detection2D box(float x, float y, float s = 0.9f, int cls = 0)
{
  Detection2D d;
  d.x1 = x; d.y1 = y; d.x2 = x + 40; d.y2 = y + 100; d.score = s; d.class_id = cls; d.class_name = "person";
  return d;
}

// Two boxes crossing each other keep their ids and get the right velocity sign.
TEST(ByteTracker, KeepsIdsOfTwoCrossingObjects)
{
  ByteTracker t;
  std::vector<TrackedBox> out;
  for (int k = 0; k < 30; ++k) {
    out = t.update({box(100 + 5 * k, 100), box(400 - 5 * k, 100)}, k * 0.033);
  }
  ASSERT_EQ(out.size(), 2u);
  // object starting left moved right
  const auto & a = out[0].det.x1 < out[1].det.x1 ? out[0] : out[1];
  EXPECT_NEAR(a.det.x1, 245, 6);
  EXPECT_GT(a.vu, 100.f);
}

// A low-score detection (stage 2) keeps an existing track matched.
TEST(ByteTracker, LowScoreDetectionKeepsTrackAlive)
{
  ByteTracker t;
  for (int k = 0; k < 5; ++k) {t.update({box(100, 100)}, k * 0.033);}
  auto out = t.update({box(101, 100, 0.2f)}, 5 * 0.033);   // occluded -> low score
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].track_id, 1u);
}
