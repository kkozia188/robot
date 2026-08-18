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
  for (AxisEnvelope & axis : envelope.axes) {
    axis.position_lower = -10.0;
    axis.position_upper = 10.0;
    axis.velocity_positive = 20.0;
    axis.velocity_negative = 20.0;
    axis.acceleration_positive = 40.0;
    axis.acceleration_negative = 40.0;
    axis.stop_acceleration_positive = 10.0;
    axis.stop_acceleration_negative = 10.0;
    axis.position_margin_lower = 0.1;
    axis.position_margin_upper = 0.1;
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

TEST(DynamicEnvelope, TestOnlyLimitsRequireExplicitOptInAndVersion)
{
  LimitChecker checker;
  DynamicEnvelope envelope = makeTestOnlyEnvelope();

  EXPECT_FALSE(checker.configure(envelope, false));
  EXPECT_FALSE(checker.configured());
  EXPECT_TRUE(checker.configure(envelope, true));
  EXPECT_TRUE(checker.configured());

  envelope.limits_version.fill(0U);
  LimitChecker missing_version_checker;
  EXPECT_FALSE(missing_version_checker.configure(envelope, true));

  envelope = makeTestOnlyEnvelope();
  envelope.axes[3].position_margin_upper = 20.0;
  LimitChecker invalid_bounds_checker;
  EXPECT_FALSE(invalid_bounds_checker.configure(envelope, true));
}

TEST(CubicHermite, ConstantPositionHasOnlyEndpointExtrema)
{
  CubicHermite segment;
  ASSERT_TRUE(buildCubicHermite(1.25, 0.0, 1.25, 0.0, 2.0, segment));

  for (const double normalized_time : {0.0, 0.25, 0.5, 0.75, 1.0}) {
    ScalarKinematicState state;
    ASSERT_TRUE(sampleCubicHermite(segment, normalized_time, state));
    EXPECT_DOUBLE_EQ(state.position, 1.25);
    EXPECT_DOUBLE_EQ(state.velocity, 0.0);
    EXPECT_DOUBLE_EQ(state.acceleration, 0.0);
  }

  ScalarKinematicExtrema extrema;
  ASSERT_TRUE(computeCubicHermiteExtrema(segment, extrema));
  EXPECT_DOUBLE_EQ(extrema.position_min, 1.25);
  EXPECT_DOUBLE_EQ(extrema.position_max, 1.25);
  EXPECT_DOUBLE_EQ(extrema.velocity_positive, 0.0);
  EXPECT_DOUBLE_EQ(extrema.velocity_negative, 0.0);
  EXPECT_DOUBLE_EQ(extrema.acceleration_positive, 0.0);
  EXPECT_DOUBLE_EQ(extrema.acceleration_negative, 0.0);
}

TEST(CubicHermite, ConstantVelocityMatchesTheAnalyticLine)
{
  CubicHermite segment;
  ASSERT_TRUE(buildCubicHermite(0.0, 2.0, 4.0, 2.0, 2.0, segment));

  ScalarKinematicState state;
  ASSERT_TRUE(sampleCubicHermite(segment, 0.25, state));
  EXPECT_DOUBLE_EQ(state.position, 1.0);
  EXPECT_DOUBLE_EQ(state.velocity, 2.0);
  EXPECT_DOUBLE_EQ(state.acceleration, 0.0);

  ScalarKinematicExtrema extrema;
  ASSERT_TRUE(computeCubicHermiteExtrema(segment, extrema));
  EXPECT_DOUBLE_EQ(extrema.position_min, 0.0);
  EXPECT_DOUBLE_EQ(extrema.position_max, 4.0);
  EXPECT_DOUBLE_EQ(extrema.velocity_positive, 2.0);
  EXPECT_DOUBLE_EQ(extrema.velocity_negative, 0.0);
  EXPECT_DOUBLE_EQ(extrema.acceleration_positive, 0.0);
  EXPECT_DOUBLE_EQ(extrema.acceleration_negative, 0.0);
}

TEST(LimitChecker, RejectsAnInteriorPositionExtremumWhenEndpointsPass)
{
  DynamicEnvelope envelope = makeTestOnlyEnvelope();
  envelope.axes[0].position_lower = -2.0;
  envelope.axes[0].position_upper = 1.0;
  envelope.axes[0].position_margin_lower = 0.0;
  envelope.axes[0].position_margin_upper = 0.1;
  LimitChecker checker;
  ASSERT_TRUE(checker.configure(envelope, true));

  JointPoint start = makeJointPoint(0U, 0.0, 0.0);
  JointPoint end = makeJointPoint(kSecondNs, 0.0, 0.0);
  start.velocities[0] = 4.0;
  end.velocities[0] = -4.0;

  SegmentExtrema extrema;
  const SegmentCheckResult result = checker.checkSegment(start, end, extrema);
  EXPECT_EQ(result.code, RejectCode::kPositionLimit);
  EXPECT_EQ(result.axis, 0U);
  EXPECT_NEAR(extrema.axes[0].position_max, 1.0, 1.0e-14);
  EXPECT_LE(start.positions[0], 0.9);
  EXPECT_LE(end.positions[0], 0.9);
}

TEST(LimitChecker, RejectsAnInteriorVelocityExtremumWhenEndpointVelocitiesPass)
{
  DynamicEnvelope envelope = makeTestOnlyEnvelope();
  envelope.axes[0].velocity_positive = 1.4;
  LimitChecker checker;
  ASSERT_TRUE(checker.configure(envelope, true));

  JointPoint start = makeJointPoint(0U, 0.0, 0.0);
  JointPoint end = makeJointPoint(kSecondNs, 0.0, 0.0);
  end.positions[0] = 1.0;

  SegmentExtrema extrema;
  const SegmentCheckResult result = checker.checkSegment(start, end, extrema);
  EXPECT_EQ(result.code, RejectCode::kVelocityLimit);
  EXPECT_EQ(result.axis, 0U);
  EXPECT_NEAR(extrema.axes[0].velocity_positive, 1.5, 1.0e-14);
  EXPECT_DOUBLE_EQ(start.velocities[0], 0.0);
  EXPECT_DOUBLE_EQ(end.velocities[0], 0.0);
}

TEST(LimitChecker, AppliesPositiveAndNegativeAccelerationLimitsIndependently)
{
  DynamicEnvelope envelope = makeTestOnlyEnvelope();
  envelope.axes[0].acceleration_positive = 1.0;
  envelope.axes[0].acceleration_negative = 0.5;
  LimitChecker checker;
  ASSERT_TRUE(checker.configure(envelope, true));

  JointPoint positive_start = makeJointPoint(0U, 0.0, 0.0);
  JointPoint positive_end = makeJointPoint(kSecondNs, 0.0, 0.0);
  positive_end.positions[0] = 0.5;
  positive_end.velocities[0] = 1.0;
  SegmentExtrema extrema;
  EXPECT_EQ(
    checker.checkSegment(positive_start, positive_end, extrema).code,
    RejectCode::kNone);

  JointPoint negative_start = makeJointPoint(0U, 0.0, 0.0);
  JointPoint negative_end = makeJointPoint(kSecondNs, 0.0, 0.0);
  negative_end.positions[0] = -0.5;
  negative_end.velocities[0] = -1.0;
  const SegmentCheckResult negative_result =
    checker.checkSegment(negative_start, negative_end, extrema);
  EXPECT_EQ(negative_result.code, RejectCode::kAccelerationLimit);
  EXPECT_EQ(negative_result.axis, 0U);
  EXPECT_NEAR(extrema.axes[0].acceleration_negative, 1.0, 1.0e-14);
}

TEST(CubicHermite, DegenerateCurvesAgreeWithLongDoubleReferenceWithoutNan)
{
  constexpr std::uint32_t kSeed = 0xE10203U;
  std::mt19937 generator(kSeed);
  std::uniform_real_distribution<double> position_distribution(-2.0, 2.0);
  std::uniform_real_distribution<double> velocity_distribution(-3.0, 3.0);
  std::uniform_real_distribution<double> duration_distribution(0.001, 2.0);

  for (std::size_t curve = 0U; curve < 100U; ++curve) {
    const double q0 = position_distribution(generator);
    const double q1 = position_distribution(generator);
    const double v0 = velocity_distribution(generator);
    const double v1 = velocity_distribution(generator);
    const double duration = duration_distribution(generator);
    CubicHermite segment;
    ASSERT_TRUE(buildCubicHermite(q0, v0, q1, v1, duration, segment))
      << "seed=" << kSeed << " curve=" << curve;

    ScalarKinematicExtrema extrema;
    ASSERT_TRUE(computeCubicHermiteExtrema(segment, extrema));
    long double sampled_q_min = std::numeric_limits<long double>::infinity();
    long double sampled_q_max = -std::numeric_limits<long double>::infinity();
    long double sampled_v_min = std::numeric_limits<long double>::infinity();
    long double sampled_v_max = -std::numeric_limits<long double>::infinity();

    for (std::size_t sample = 0U; sample <= 4096U; ++sample) {
      const long double s = static_cast<long double>(sample) / 4096.0L;
      const long double h = static_cast<long double>(duration);
      const long double c0 = static_cast<long double>(q0);
      const long double c1 = h * static_cast<long double>(v0);
      const long double c2 =
        -3.0L * q0 - 2.0L * h * v0 + 3.0L * q1 - h * v1;
      const long double c3 =
        2.0L * q0 + h * v0 - 2.0L * q1 + h * v1;
      const long double expected_q = ((c3 * s + c2) * s + c1) * s + c0;
      const long double expected_v = (3.0L * c3 * s * s + 2.0L * c2 * s + c1) / h;
      sampled_q_min = std::min(sampled_q_min, expected_q);
      sampled_q_max = std::max(sampled_q_max, expected_q);
      sampled_v_min = std::min(sampled_v_min, expected_v);
      sampled_v_max = std::max(sampled_v_max, expected_v);

      if (sample % 257U == 0U) {
        ScalarKinematicState state;
        ASSERT_TRUE(sampleCubicHermite(segment, static_cast<double>(s), state));
        EXPECT_NEAR(state.position, static_cast<double>(expected_q), 2.0e-12)
          << "seed=" << kSeed << " curve=" << curve << " sample=" << sample;
        EXPECT_NEAR(state.velocity, static_cast<double>(expected_v), 2.0e-11)
          << "seed=" << kSeed << " curve=" << curve << " sample=" << sample;
      }
    }

    EXPECT_LE(extrema.position_min, static_cast<double>(sampled_q_min) + 2.0e-12);
    EXPECT_GE(extrema.position_max, static_cast<double>(sampled_q_max) - 2.0e-12);
    EXPECT_LE(-extrema.velocity_negative, static_cast<double>(sampled_v_min) + 2.0e-11);
    EXPECT_GE(extrema.velocity_positive, static_cast<double>(sampled_v_max) - 2.0e-11);
  }

  CubicHermite near_linear;
  ASSERT_TRUE(buildCubicHermite(0.0, 1.0, 1.0 + 1.0e-13, 1.0 + 2.0e-13, 1.0, near_linear));
  ScalarKinematicExtrema extrema;
  EXPECT_TRUE(computeCubicHermiteExtrema(near_linear, extrema));
  EXPECT_TRUE(std::isfinite(extrema.position_min));
  EXPECT_TRUE(std::isfinite(extrema.position_max));
  EXPECT_TRUE(std::isfinite(extrema.velocity_positive));
  EXPECT_TRUE(std::isfinite(extrema.acceleration_positive));
}

TEST(SpliceContinuity, ExactTolerancePassesAndNextRepresentableValueFails)
{
  JointPoint expected = makeJointPoint(0U, 0.0, 0.0);
  JointPoint replacement = expected;
  std::array<double, kAxisCount> position_tolerance{};
  std::array<double, kAxisCount> velocity_tolerance{};
  position_tolerance.fill(1.0e-6);
  velocity_tolerance.fill(2.0e-6);

  replacement.positions[7] = position_tolerance[7];
  EXPECT_EQ(
    checkSpliceContinuity(expected, replacement, position_tolerance, velocity_tolerance),
    RejectCode::kNone);
  replacement.positions[7] =
    std::nextafter(position_tolerance[7], std::numeric_limits<double>::infinity());
  EXPECT_EQ(
    checkSpliceContinuity(expected, replacement, position_tolerance, velocity_tolerance),
    RejectCode::kPositionDiscontinuity);

  replacement = expected;
  replacement.velocities[13] = velocity_tolerance[13];
  EXPECT_EQ(
    checkSpliceContinuity(expected, replacement, position_tolerance, velocity_tolerance),
    RejectCode::kNone);
  replacement.velocities[13] =
    std::nextafter(velocity_tolerance[13], std::numeric_limits<double>::infinity());
  EXPECT_EQ(
    checkSpliceContinuity(expected, replacement, position_tolerance, velocity_tolerance),
    RejectCode::kVelocityDiscontinuity);
}

TEST(LimitChecker, RotaryAndPrismaticLimitsRemainAxisLocal)
{
  DynamicEnvelope envelope = makeTestOnlyEnvelope();
  envelope.axes[0].velocity_positive = 5.0;
  envelope.axes[13].position_lower = 0.0;
  envelope.axes[13].position_upper = 0.8;
  envelope.axes[13].position_margin_lower = 0.01;
  envelope.axes[13].position_margin_upper = 0.02;
  envelope.axes[13].velocity_positive = 0.3;
  LimitChecker checker;
  ASSERT_TRUE(checker.configure(envelope, true));

  JointPoint start = makeJointPoint(0U, 0.0, 0.0);
  JointPoint end = makeJointPoint(kSecondNs, 0.0, 0.0);
  start.positions[13] = 0.4;
  end.positions[13] = 0.7;
  start.velocities[0] = 2.0;
  end.velocities[0] = 2.0;
  start.velocities[13] = 0.31;
  end.velocities[13] = 0.31;

  SegmentExtrema extrema;
  const SegmentCheckResult result = checker.checkSegment(start, end, extrema);
  EXPECT_EQ(result.code, RejectCode::kVelocityLimit);
  EXPECT_EQ(result.axis, 13U);
}

}  // namespace
}  // namespace rolling_trajectory_controller
