#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

#include "rolling_trajectory_controller/rolling_buffer.hpp"

namespace rolling_trajectory_controller
{
namespace
{

struct OwnedPoint
{
  std::uint64_t time_ns{0U};
  std::array<double, kAxisCount> positions{};
  std::array<double, kAxisCount> velocities{};

  PointView view() const noexcept
  {
    return PointView{
      time_ns, positions.data(), positions.size(), velocities.data(), velocities.size()};
  }
};

OwnedPoint makePoint(std::uint64_t time_ns, double offset = 0.0)
{
  OwnedPoint point;
  point.time_ns = time_ns;
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    point.positions[axis] = offset + static_cast<double>(axis) * 0.01;
    point.velocities[axis] = 0.1 + static_cast<double>(axis) * 0.001;
  }
  return point;
}

template<std::size_t Size>
std::array<PointView, Size> makeViews(const std::array<OwnedPoint, Size> & points)
{
  std::array<PointView, Size> views{};
  for (std::size_t index = 0; index < Size; ++index) {
    views[index] = points[index].view();
  }
  return views;
}

TEST(RollingBuffer, ConfigurationEnforcesTheTransportCeiling)
{
  RollingBuffer buffer;

  EXPECT_FALSE(buffer.configure(0U));
  EXPECT_FALSE(buffer.configured());
  EXPECT_FALSE(buffer.configure(kTransportMaxPoints + 1U));
  EXPECT_FALSE(buffer.configured());

  EXPECT_TRUE(buffer.configure(1U));
  EXPECT_TRUE(buffer.configured());
  EXPECT_EQ(buffer.capacity(), 1U);

  EXPECT_TRUE(buffer.configure(kTransportMaxPoints));
  EXPECT_EQ(buffer.capacity(), kTransportMaxPoints);
  EXPECT_EQ(buffer.image().point_count, 0U);
  EXPECT_EQ(buffer.image().generation, 0U);
}

TEST(RollingBuffer, RejectsEmptyNullAndRuntimeCapacityOverflow)
{
  RollingBuffer buffer;
  ASSERT_TRUE(buffer.configure(2U));
  EXPECT_EQ(buffer.replace(nullptr, 0U), RejectCode::kInvalidShape);
  EXPECT_EQ(buffer.replace(nullptr, 1U), RejectCode::kInvalidShape);

  const std::array<OwnedPoint, 3> points = {
    makePoint(0U), makePoint(4'000'000U), makePoint(8'000'000U)};
  const auto views = makeViews(points);
  EXPECT_EQ(buffer.replace(views.data(), views.size()), RejectCode::kCapacityExceeded);
  EXPECT_EQ(buffer.image().point_count, 0U);
  EXPECT_EQ(buffer.image().generation, 0U);
}

TEST(RollingBuffer, RejectsAnyPointWithoutExactlyFourteenPositionsAndVelocities)
{
  RollingBuffer buffer;
  ASSERT_TRUE(buffer.configure(2U));
  const std::array<OwnedPoint, 2> points = {makePoint(0U), makePoint(4'000'000U)};
  auto views = makeViews(points);

  views[1].position_count = kAxisCount - 1U;
  EXPECT_EQ(buffer.replace(views.data(), views.size()), RejectCode::kInvalidShape);

  views = makeViews(points);
  views[0].velocity_count = kAxisCount - 1U;
  EXPECT_EQ(buffer.replace(views.data(), views.size()), RejectCode::kInvalidShape);
  EXPECT_EQ(buffer.image().generation, 0U);
}

TEST(RollingBuffer, RejectsNonFinitePositionOrVelocity)
{
  RollingBuffer buffer;
  ASSERT_TRUE(buffer.configure(2U));
  std::array<OwnedPoint, 2> points = {makePoint(0U), makePoint(4'000'000U)};

  points[1].positions[4] = std::numeric_limits<double>::quiet_NaN();
  auto views = makeViews(points);
  EXPECT_EQ(buffer.replace(views.data(), views.size()), RejectCode::kNonFinite);

  points[1] = makePoint(4'000'000U);
  points[0].velocities[13] = std::numeric_limits<double>::infinity();
  views = makeViews(points);
  EXPECT_EQ(buffer.replace(views.data(), views.size()), RejectCode::kNonFinite);
  EXPECT_EQ(buffer.image().generation, 0U);
}

TEST(RollingBuffer, ValidationPrecedenceIsShapeThenFiniteThenTime)
{
  RollingBuffer buffer;
  ASSERT_TRUE(buffer.configure(3U));
  std::array<OwnedPoint, 3> points = {
    makePoint(4'000'000U), makePoint(0U), makePoint(8'000'000U)};
  points[0].positions[0] = std::numeric_limits<double>::quiet_NaN();
  auto views = makeViews(points);

  views[2].velocity_count = kAxisCount - 1U;
  EXPECT_EQ(buffer.replace(views.data(), views.size()), RejectCode::kInvalidShape);

  views = makeViews(points);
  EXPECT_EQ(buffer.replace(views.data(), views.size()), RejectCode::kNonFinite);

  points[0] = makePoint(4'000'000U);
  views = makeViews(points);
  EXPECT_EQ(buffer.replace(views.data(), views.size()), RejectCode::kNonMonotonicTime);
  EXPECT_EQ(buffer.image().generation, 0U);
}

TEST(RollingBuffer, RejectsDuplicateOrDecreasingTime)
{
  RollingBuffer buffer;
  ASSERT_TRUE(buffer.configure(3U));

  std::array<OwnedPoint, 3> points = {
    makePoint(0U), makePoint(4'000'000U), makePoint(4'000'000U)};
  auto views = makeViews(points);
  EXPECT_EQ(buffer.replace(views.data(), views.size()), RejectCode::kNonMonotonicTime);

  points[2].time_ns = 1U;
  views = makeViews(points);
  EXPECT_EQ(buffer.replace(views.data(), views.size()), RejectCode::kNonMonotonicTime);
  EXPECT_EQ(buffer.image().generation, 0U);
}

TEST(RollingBuffer, AcceptsUpToTheConfiguredCapacityAndAdvancesGeneration)
{
  RollingBuffer buffer;
  ASSERT_TRUE(buffer.configure(kTransportMaxPoints));

  std::array<OwnedPoint, kTransportMaxPoints> points{};
  for (std::size_t index = 0; index < points.size(); ++index) {
    points[index] = makePoint(static_cast<std::uint64_t>(index) * 4'000'000U, 1.0);
  }
  const auto views = makeViews(points);

  ASSERT_EQ(buffer.replace(views.data(), views.size()), RejectCode::kNone);
  EXPECT_EQ(buffer.image().point_count, kTransportMaxPoints);
  EXPECT_EQ(buffer.image().generation, 1U);
  EXPECT_EQ(buffer.image().points.front().positions[0], 1.0);
  EXPECT_EQ(buffer.image().points.back().time_ns, 1'020'000'000U);

  ASSERT_EQ(buffer.replace(views.data(), 1U), RejectCode::kNone);
  EXPECT_EQ(buffer.image().point_count, 1U);
  EXPECT_EQ(buffer.image().generation, 2U);
}

TEST(RollingBuffer, RejectionLeavesTheActiveImageByteForByteUnchanged)
{
  RollingBuffer buffer;
  ASSERT_TRUE(buffer.configure(2U));
  std::array<OwnedPoint, 2> points = {makePoint(0U), makePoint(4'000'000U)};
  auto views = makeViews(points);
  ASSERT_EQ(buffer.replace(views.data(), views.size()), RejectCode::kNone);

  std::array<std::byte, sizeof(TrajectoryImage)> before{};
  std::memcpy(before.data(), &buffer.image(), before.size());

  points[1].positions[0] = std::numeric_limits<double>::quiet_NaN();
  views = makeViews(points);
  ASSERT_EQ(buffer.replace(views.data(), views.size()), RejectCode::kNonFinite);

  std::array<std::byte, sizeof(TrajectoryImage)> after{};
  std::memcpy(after.data(), &buffer.image(), after.size());
  EXPECT_EQ(before, after);
}

}  // namespace
}  // namespace rolling_trajectory_controller
