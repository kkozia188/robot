#include "rolling_trajectory_controller/limit_checker.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace rolling_trajectory_controller
{
namespace
{

bool hasVersion(const std::array<std::uint8_t, 32> & version) noexcept
{
  return std::any_of(
    version.begin(), version.end(), [](std::uint8_t value) {
      return value != 0U;
    });
}

bool validAxisEnvelope(const AxisEnvelope & axis) noexcept
{
  const bool values_are_finite =
    std::isfinite(axis.position_lower) && std::isfinite(axis.position_upper) &&
    std::isfinite(axis.velocity_positive) && std::isfinite(axis.velocity_negative) &&
    std::isfinite(axis.acceleration_positive) &&
    std::isfinite(axis.acceleration_negative) &&
    std::isfinite(axis.stop_acceleration_positive) &&
    std::isfinite(axis.stop_acceleration_negative) &&
    std::isfinite(axis.position_margin_lower) &&
    std::isfinite(axis.position_margin_upper);
  if (!values_are_finite) {
    return false;
  }
  if (
    axis.position_lower >= axis.position_upper ||
    axis.velocity_positive <= 0.0 || axis.velocity_negative <= 0.0 ||
    axis.acceleration_positive <= 0.0 || axis.acceleration_negative <= 0.0 ||
    axis.stop_acceleration_positive <= 0.0 ||
    axis.stop_acceleration_negative <= 0.0 ||
    axis.position_margin_lower < 0.0 || axis.position_margin_upper < 0.0)
  {
    return false;
  }

  const double safe_lower = axis.position_lower + axis.position_margin_lower;
  const double safe_upper = axis.position_upper - axis.position_margin_upper;
  return std::isfinite(safe_lower) && std::isfinite(safe_upper) && safe_lower < safe_upper;
}

bool validEnvelope(
  const DynamicEnvelope & envelope, bool allow_test_only_limits) noexcept
{
  if (
    envelope.source != LimitsSource::kProduction &&
    envelope.source != LimitsSource::kTestOnly)
  {
    return false;
  }
  if (envelope.source == LimitsSource::kTestOnly && !allow_test_only_limits) {
    return false;
  }
  if (!hasVersion(envelope.limits_version)) {
    return false;
  }
  return std::all_of(
    envelope.axes.begin(), envelope.axes.end(), validAxisEnvelope);
}

}  // namespace

bool LimitChecker::configure(
  const DynamicEnvelope & envelope, bool allow_test_only_limits) noexcept
{
  configured_ = false;
  if (!validEnvelope(envelope, allow_test_only_limits)) {
    return false;
  }
  envelope_ = envelope;
  configured_ = true;
  return true;
}

bool LimitChecker::configured() const noexcept
{
  return configured_;
}

const DynamicEnvelope & LimitChecker::envelope() const noexcept
{
  return envelope_;
}

SegmentCheckResult LimitChecker::checkSegment(
  const JointPoint & start, const JointPoint & end,
  SegmentExtrema & extrema) const noexcept
{
  if (!configured_) {
    return SegmentCheckResult{RejectCode::kSessionNotAccepting, kAxisCount};
  }
  if (end.time_ns <= start.time_ns) {
    return SegmentCheckResult{RejectCode::kNonMonotonicTime, kAxisCount};
  }

  const std::uint64_t duration_ns = end.time_ns - start.time_ns;
  const double duration_seconds = static_cast<double>(duration_ns) * 1.0e-9;
  SegmentExtrema candidate;
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    CubicHermite segment;
    if (
      !buildCubicHermite(
        start.positions[axis], start.velocities[axis],
        end.positions[axis], end.velocities[axis],
        duration_seconds, segment) ||
      !computeCubicHermiteExtrema(segment, candidate.axes[axis]))
    {
      return SegmentCheckResult{RejectCode::kNonFinite, axis};
    }
  }
  extrema = candidate;

  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    const AxisEnvelope & limits = envelope_.axes[axis];
    const double safe_lower = limits.position_lower + limits.position_margin_lower;
    const double safe_upper = limits.position_upper - limits.position_margin_upper;
    const bool below_lower_bound = candidate.axes[axis].position_min < safe_lower;
    const bool above_upper_bound = candidate.axes[axis].position_max > safe_upper;
    if (below_lower_bound || above_upper_bound) {
      return SegmentCheckResult{RejectCode::kPositionLimit, axis};
    }
  }
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    const AxisEnvelope & limits = envelope_.axes[axis];
    if (
      candidate.axes[axis].velocity_positive > limits.velocity_positive ||
      candidate.axes[axis].velocity_negative > limits.velocity_negative)
    {
      return SegmentCheckResult{RejectCode::kVelocityLimit, axis};
    }
  }
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    const AxisEnvelope & limits = envelope_.axes[axis];
    if (
      candidate.axes[axis].acceleration_positive > limits.acceleration_positive ||
      candidate.axes[axis].acceleration_negative > limits.acceleration_negative)
    {
      return SegmentCheckResult{RejectCode::kAccelerationLimit, axis};
    }
  }
  return SegmentCheckResult{};
}

SegmentCheckResult LimitChecker::checkStoppingViability(
  const SegmentExtrema & extrema,
  StoppingEnvelope & stopping_envelope) const noexcept
{
  if (!configured_) {
    return SegmentCheckResult{RejectCode::kSessionNotAccepting, kAxisCount};
  }

  StoppingEnvelope candidate;
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    const AxisEnvelope & limits = envelope_.axes[axis];
    const ScalarKinematicExtrema & axis_extrema = extrema.axes[axis];
    if (
      !std::isfinite(axis_extrema.position_min) ||
      !std::isfinite(axis_extrema.position_max) ||
      !std::isfinite(axis_extrema.velocity_positive) ||
      !std::isfinite(axis_extrema.velocity_negative) ||
      axis_extrema.velocity_positive < 0.0 || axis_extrema.velocity_negative < 0.0)
    {
      return SegmentCheckResult{RejectCode::kNonFinite, axis};
    }
    candidate.duration_seconds = std::max(
      candidate.duration_seconds,
      axis_extrema.velocity_positive / limits.stop_acceleration_positive);
    candidate.duration_seconds = std::max(
      candidate.duration_seconds,
      axis_extrema.velocity_negative / limits.stop_acceleration_negative);
  }
  if (!std::isfinite(candidate.duration_seconds)) {
    return SegmentCheckResult{RejectCode::kNonFinite, kAxisCount};
  }

  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    const ScalarKinematicExtrema & axis_extrema = extrema.axes[axis];
    candidate.lower_positions[axis] =
      axis_extrema.position_min -
      0.5 * axis_extrema.velocity_negative * candidate.duration_seconds;
    candidate.upper_positions[axis] =
      axis_extrema.position_max +
      0.5 * axis_extrema.velocity_positive * candidate.duration_seconds;
    if (
      !std::isfinite(candidate.lower_positions[axis]) ||
      !std::isfinite(candidate.upper_positions[axis]))
    {
      return SegmentCheckResult{RejectCode::kNonFinite, axis};
    }
  }
  stopping_envelope = candidate;

  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    const AxisEnvelope & limits = envelope_.axes[axis];
    const double safe_lower = limits.position_lower + limits.position_margin_lower;
    const double safe_upper = limits.position_upper - limits.position_margin_upper;
    if (
      candidate.lower_positions[axis] < safe_lower ||
      candidate.upper_positions[axis] > safe_upper)
    {
      return SegmentCheckResult{RejectCode::kNotStoppingViable, axis};
    }
  }
  return SegmentCheckResult{};
}

RejectCode checkSpliceContinuity(
  const JointPoint & expected, const JointPoint & replacement,
  const std::array<double, kAxisCount> & position_tolerance,
  const std::array<double, kAxisCount> & velocity_tolerance) noexcept
{
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    if (
      !std::isfinite(expected.positions[axis]) ||
      !std::isfinite(expected.velocities[axis]) ||
      !std::isfinite(replacement.positions[axis]) ||
      !std::isfinite(replacement.velocities[axis]) ||
      !std::isfinite(position_tolerance[axis]) ||
      !std::isfinite(velocity_tolerance[axis]) ||
      position_tolerance[axis] < 0.0 || velocity_tolerance[axis] < 0.0)
    {
      return RejectCode::kNonFinite;
    }
  }
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    if (
      std::abs(replacement.positions[axis] - expected.positions[axis]) >
      position_tolerance[axis])
    {
      return RejectCode::kPositionDiscontinuity;
    }
  }
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    if (
      std::abs(replacement.velocities[axis] - expected.velocities[axis]) >
      velocity_tolerance[axis])
    {
      return RejectCode::kVelocityDiscontinuity;
    }
  }
  return RejectCode::kNone;
}

}  // namespace rolling_trajectory_controller
