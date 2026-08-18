#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>

#include "rolling_trajectory_controller/cubic_hermite.hpp"
#include "rolling_trajectory_controller/limit_checker.hpp"
#include "rolling_trajectory_controller/rolling_buffer.hpp"
#include "rolling_trajectory_controller/session_core.hpp"

namespace rolling_trajectory_controller
{
namespace
{

constexpr std::uint64_t kSecondNs = 1'000'000'000U;

DynamicEnvelope makeTestOnlyEnvelope()
{
  DynamicEnvelope envelope;
  envelope.source = LimitsSource::kTestOnly;
  envelope.limits_version[0] = 0xE1U;
  envelope.limits_version[1] = 0x02U;
  envelope.limits_version[2] = 0x04U;
  for (AxisEnvelope & axis : envelope.axes) {
    axis.position_lower = -100.0;
    axis.position_upper = 100.0;
    axis.velocity_positive = 20.0;
    axis.velocity_negative = 20.0;
    axis.acceleration_positive = 40.0;
    axis.acceleration_negative = 40.0;
    axis.stop_acceleration_positive = 2.0;
    axis.stop_acceleration_negative = 3.0;
    axis.position_margin_lower = 0.0;
    axis.position_margin_upper = 0.0;
  }
  return envelope;
}

JointPoint makeJointPoint(std::uint64_t time_ns, double position, double velocity)
{
  JointPoint point;
  point.time_ns = time_ns;
  point.positions.fill(position);
  point.velocities.fill(velocity);
  return point;
}

TEST(StopTrajectory, ZeroVelocityTransitionsDirectlyToAnExactHold)
{
  StopTrajectory trajectory;
  const DynamicEnvelope envelope = makeTestOnlyEnvelope();
  ASSERT_TRUE(trajectory.configure(envelope, true));
  JointPoint desired = makeJointPoint(17U, 1.25, 0.0);

  ASSERT_TRUE(trajectory.begin(desired));
  EXPECT_EQ(trajectory.state(), StopTrajectoryState::kHolding);
  EXPECT_DOUBLE_EQ(trajectory.durationSeconds(), 0.0);
  EXPECT_EQ(trajectory.durationNs(), 0U);

  JointPoint sample;
  ASSERT_TRUE(trajectory.sample(0U, sample));
  EXPECT_EQ(sample.positions, desired.positions);
  for (const double velocity : sample.velocities) {
    EXPECT_DOUBLE_EQ(velocity, 0.0);
  }
}

TEST(StopTrajectory, PositiveLimitingAxisSetsOneSynchronousDuration)
{
  StopTrajectory trajectory;
  DynamicEnvelope envelope = makeTestOnlyEnvelope();
  envelope.axes[0].stop_acceleration_positive = 2.0;
  envelope.axes[1].stop_acceleration_positive = 4.0;
  ASSERT_TRUE(trajectory.configure(envelope, true));
  JointPoint desired = makeJointPoint(0U, 0.0, 0.0);
  desired.velocities[0] = 4.0;
  desired.velocities[1] = 2.0;

  ASSERT_TRUE(trajectory.begin(desired));
  EXPECT_EQ(trajectory.state(), StopTrajectoryState::kStopping);
  EXPECT_DOUBLE_EQ(trajectory.durationSeconds(), 2.0);
  EXPECT_EQ(trajectory.durationNs(), 2U * kSecondNs);
  EXPECT_DOUBLE_EQ(trajectory.accelerations()[0], -2.0);
  EXPECT_DOUBLE_EQ(trajectory.accelerations()[1], -1.0);

  JointPoint halfway;
  ASSERT_TRUE(trajectory.sample(kSecondNs, halfway));
  EXPECT_DOUBLE_EQ(halfway.velocities[0], 2.0);
  EXPECT_DOUBLE_EQ(halfway.velocities[1], 1.0);
  EXPECT_DOUBLE_EQ(halfway.positions[0], 3.0);
  EXPECT_DOUBLE_EQ(halfway.positions[1], 1.5);

  JointPoint terminal;
  ASSERT_TRUE(trajectory.sample(2U * kSecondNs, terminal));
  EXPECT_EQ(trajectory.state(), StopTrajectoryState::kHolding);
  EXPECT_DOUBLE_EQ(terminal.positions[0], 4.0);
  EXPECT_DOUBLE_EQ(terminal.positions[1], 2.0);
  EXPECT_DOUBLE_EQ(terminal.velocities[0], 0.0);
  EXPECT_DOUBLE_EQ(terminal.velocities[1], 0.0);
}

TEST(StopTrajectory, NegativeVelocityUsesTheNegativeDirectionalLimit)
{
  StopTrajectory trajectory;
  DynamicEnvelope envelope = makeTestOnlyEnvelope();
  envelope.axes[2].stop_acceleration_positive = 100.0;
  envelope.axes[2].stop_acceleration_negative = 1.0;
  ASSERT_TRUE(trajectory.configure(envelope, true));
  JointPoint desired = makeJointPoint(0U, 0.0, 0.0);
  desired.velocities[2] = -3.0;

  ASSERT_TRUE(trajectory.begin(desired));
  EXPECT_DOUBLE_EQ(trajectory.durationSeconds(), 3.0);
  EXPECT_DOUBLE_EQ(trajectory.accelerations()[2], 1.0);
  JointPoint terminal;
  ASSERT_TRUE(trajectory.sample(3U * kSecondNs, terminal));
  EXPECT_DOUBLE_EQ(terminal.positions[2], -4.5);
  EXPECT_DOUBLE_EQ(terminal.velocities[2], 0.0);
}

TEST(StopTrajectory, MixedAxesStopTogetherAndCannotBeReplaced)
{
  StopTrajectory trajectory;
  DynamicEnvelope envelope = makeTestOnlyEnvelope();
  JointPoint desired = makeJointPoint(100U, 0.0, 0.0);
  double expected_duration = 0.0;
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    const double magnitude = 0.25 * static_cast<double>(axis + 1U);
    desired.velocities[axis] = axis % 2U == 0U ? magnitude : -magnitude;
    const double limit = axis % 2U == 0U ?
      envelope.axes[axis].stop_acceleration_positive :
      envelope.axes[axis].stop_acceleration_negative;
    expected_duration = std::max(expected_duration, magnitude / limit);
  }
  ASSERT_TRUE(trajectory.configure(envelope, true));
  ASSERT_TRUE(trajectory.begin(desired));
  EXPECT_DOUBLE_EQ(trajectory.durationSeconds(), expected_duration);

  JointPoint conflicting = desired;
  conflicting.positions.fill(50.0);
  EXPECT_FALSE(trajectory.begin(conflicting));

  JointPoint terminal;
  ASSERT_TRUE(trajectory.sample(trajectory.durationNs(), terminal));
  EXPECT_EQ(trajectory.state(), StopTrajectoryState::kHolding);
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    EXPECT_NEAR(
      terminal.positions[axis],
      desired.positions[axis] + 0.5 * desired.velocities[axis] * expected_duration,
      1.0e-13);
    EXPECT_DOUBLE_EQ(terminal.velocities[axis], 0.0);
    const double acceleration_magnitude = std::abs(trajectory.accelerations()[axis]);
    const double directional_limit = desired.velocities[axis] >= 0.0 ?
      envelope.axes[axis].stop_acceleration_positive :
      envelope.axes[axis].stop_acceleration_negative;
    EXPECT_LE(acceleration_magnitude, directional_limit);
  }
}

TEST(StopTrajectory, ExactSafePositionBoundPassesAndNextValueFails)
{
  DynamicEnvelope envelope = makeTestOnlyEnvelope();
  envelope.axes[0].position_upper = 10.0;
  envelope.axes[0].position_margin_upper = 0.5;
  envelope.axes[0].stop_acceleration_positive = 2.0;
  JointPoint desired = makeJointPoint(0U, 0.0, 0.0);
  desired.positions[0] = 8.5;
  desired.velocities[0] = 2.0;

  StopTrajectory exact;
  ASSERT_TRUE(exact.configure(envelope, true));
  EXPECT_TRUE(exact.begin(desired));
  EXPECT_DOUBLE_EQ(exact.terminalPositions()[0], 9.5);

  desired.positions[0] = std::nextafter(8.5, std::numeric_limits<double>::infinity());
  StopTrajectory outside;
  ASSERT_TRUE(outside.configure(envelope, true));
  EXPECT_FALSE(outside.begin(desired));
  EXPECT_EQ(outside.state(), StopTrajectoryState::kIdle);
}

TEST(StoppingViability, ConservativeEnvelopeRejectsAStopUnsafeDirectlyValidSegment)
{
  DynamicEnvelope envelope = makeTestOnlyEnvelope();
  envelope.axes[0].position_upper = 1.0;
  envelope.axes[0].stop_acceleration_positive = 1.0;
  LimitChecker checker;
  ASSERT_TRUE(checker.configure(envelope, true));
  JointPoint start = makeJointPoint(0U, 0.0, 0.0);
  JointPoint end = makeJointPoint(kSecondNs, 0.0, 0.0);
  start.velocities[0] = 0.8;
  end.positions[0] = 0.8;
  end.velocities[0] = 0.8;

  SegmentExtrema extrema;
  ASSERT_EQ(checker.checkSegment(start, end, extrema).code, RejectCode::kNone);
  StoppingEnvelope stopping_envelope;
  const SegmentCheckResult result =
    checker.checkStoppingViability(extrema, stopping_envelope);
  EXPECT_EQ(result.code, RejectCode::kNotStoppingViable);
  EXPECT_EQ(result.axis, 0U);
  EXPECT_DOUBLE_EQ(stopping_envelope.duration_seconds, 0.8);
  EXPECT_NEAR(stopping_envelope.upper_positions[0], 1.12, 1.0e-14);
}

TEST(StoppingViability, ConservativeBoundsNeverUnderestimateDenseExactStops)
{
  constexpr std::uint32_t kSeed = 0xE10213U;
  std::mt19937 generator(kSeed);
  std::uniform_real_distribution<double> position_distribution(-2.0, 2.0);
  std::uniform_real_distribution<double> velocity_distribution(-3.0, 3.0);
  DynamicEnvelope envelope = makeTestOnlyEnvelope();
  LimitChecker checker;
  ASSERT_TRUE(checker.configure(envelope, true));

  for (std::size_t curve = 0U; curve < 50U; ++curve) {
    JointPoint start = makeJointPoint(0U, 0.0, 0.0);
    JointPoint end = makeJointPoint(kSecondNs, 0.0, 0.0);
    for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
      start.positions[axis] = position_distribution(generator);
      end.positions[axis] = position_distribution(generator);
      start.velocities[axis] = velocity_distribution(generator);
      end.velocities[axis] = velocity_distribution(generator);
    }

    SegmentExtrema extrema;
    ASSERT_EQ(checker.checkSegment(start, end, extrema).code, RejectCode::kNone)
      << "seed=" << kSeed << " curve=" << curve;
    StoppingEnvelope conservative;
    ASSERT_EQ(
      checker.checkStoppingViability(extrema, conservative).code,
      RejectCode::kNone);

    for (std::size_t sample_index = 0U; sample_index <= 1024U; ++sample_index) {
      const double s = static_cast<double>(sample_index) / 1024.0;
      std::array<ScalarKinematicState, kAxisCount> states{};
      double exact_duration = 0.0;
      for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
        CubicHermite segment;
        ASSERT_TRUE(
          buildCubicHermite(
            start.positions[axis], start.velocities[axis],
            end.positions[axis], end.velocities[axis], 1.0, segment));
        ASSERT_TRUE(sampleCubicHermite(segment, s, states[axis]));
        const double stop_limit = states[axis].velocity >= 0.0 ?
          envelope.axes[axis].stop_acceleration_positive :
          envelope.axes[axis].stop_acceleration_negative;
        exact_duration = std::max(
          exact_duration, std::abs(states[axis].velocity) / stop_limit);
      }
      for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
        const double exact_terminal =
          states[axis].position + 0.5 * states[axis].velocity * exact_duration;
        EXPECT_LE(conservative.lower_positions[axis], exact_terminal + 1.0e-12)
          << "seed=" << kSeed << " curve=" << curve << " sample=" << sample_index;
        EXPECT_GE(conservative.upper_positions[axis], exact_terminal - 1.0e-12)
          << "seed=" << kSeed << " curve=" << curve << " sample=" << sample_index;
      }
    }
  }
}

TEST(HorizonGuard, EqualityLatchesLowWaterAndOneNanosecondMorePasses)
{
  const SchedulingGuard guard{1U, 2U, 3U, 4U};
  EXPECT_FALSE(hasSufficientStoppingHorizon(1'000U, 1'110U, 100U, guard));
  EXPECT_FALSE(hasSufficientStoppingHorizon(1'000U, 1'109U, 100U, guard));
  EXPECT_TRUE(hasSufficientStoppingHorizon(1'000U, 1'111U, 100U, guard));
}

TEST(HorizonGuard, StopTimeGrowthComponentCannotBeSilentlyDropped)
{
  const SchedulingGuard without_growth{4U, 0U, 3U, 3U};
  const SchedulingGuard with_growth{4U, 5U, 3U, 3U};
  EXPECT_TRUE(hasSufficientStoppingHorizon(0U, 111U, 100U, without_growth));
  EXPECT_FALSE(hasSufficientStoppingHorizon(0U, 111U, 100U, with_growth));

  const SchedulingGuard overflowing{
    std::numeric_limits<std::uint64_t>::max(), 1U, 0U, 0U};
  EXPECT_FALSE(
    hasSufficientStoppingHorizon(
      0U, std::numeric_limits<std::uint64_t>::max(), 0U,
      overflowing));
}

}  // namespace
}  // namespace rolling_trajectory_controller
