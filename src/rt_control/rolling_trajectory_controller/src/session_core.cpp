#include "rolling_trajectory_controller/session_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace rolling_trajectory_controller
{
namespace
{

bool addWithoutOverflow(std::uint64_t value, std::uint64_t increment, std::uint64_t & sum) noexcept
{
  if (increment > std::numeric_limits<std::uint64_t>::max() - value) {
    return false;
  }
  sum = value + increment;
  return true;
}

bool durationToNanoseconds(double duration_seconds, std::uint64_t & duration_ns) noexcept
{
  if (!std::isfinite(duration_seconds) || duration_seconds < 0.0) {
    return false;
  }
  const long double nanoseconds =
    std::ceil(static_cast<long double>(duration_seconds) * 1.0e9L);
  if (
    nanoseconds < 0.0L ||
    nanoseconds > static_cast<long double>(std::numeric_limits<std::uint64_t>::max()))
  {
    return false;
  }
  duration_ns = static_cast<std::uint64_t>(nanoseconds);
  return true;
}

}  // namespace

bool StopTrajectory::configure(
  const DynamicEnvelope & envelope, bool allow_test_only_limits) noexcept
{
  if (state_ != StopTrajectoryState::kIdle) {
    return false;
  }
  return limit_checker_.configure(envelope, allow_test_only_limits);
}

bool StopTrajectory::begin(const JointPoint & desired) noexcept
{
  if (!limit_checker_.configured() || state_ != StopTrajectoryState::kIdle) {
    return false;
  }

  double duration_seconds = 0.0;
  const DynamicEnvelope & envelope = limit_checker_.envelope();
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    if (!std::isfinite(desired.positions[axis]) || !std::isfinite(desired.velocities[axis])) {
      return false;
    }
    const double velocity = desired.velocities[axis];
    const double stop_acceleration = velocity >= 0.0 ?
      envelope.axes[axis].stop_acceleration_positive :
      envelope.axes[axis].stop_acceleration_negative;
    duration_seconds = std::max(duration_seconds, std::abs(velocity) / stop_acceleration);
  }
  if (!std::isfinite(duration_seconds)) {
    return false;
  }

  std::uint64_t duration_ns = 0U;
  if (!durationToNanoseconds(duration_seconds, duration_ns)) {
    return false;
  }

  std::array<double, kAxisCount> accelerations{};
  std::array<double, kAxisCount> terminal_positions{};
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    const double velocity = desired.velocities[axis];
    accelerations[axis] = duration_seconds == 0.0 ? 0.0 : -velocity / duration_seconds;
    terminal_positions[axis] =
      desired.positions[axis] + 0.5 * velocity * duration_seconds;
    const AxisEnvelope & limits = envelope.axes[axis];
    const double safe_lower = limits.position_lower + limits.position_margin_lower;
    const double safe_upper = limits.position_upper - limits.position_margin_upper;
    if (
      !std::isfinite(accelerations[axis]) || !std::isfinite(terminal_positions[axis]) ||
      terminal_positions[axis] < safe_lower || terminal_positions[axis] > safe_upper)
    {
      return false;
    }
    const double directional_limit = velocity >= 0.0 ?
      limits.stop_acceleration_positive : limits.stop_acceleration_negative;
    if (std::abs(accelerations[axis]) > directional_limit) {
      return false;
    }
  }

  start_ = desired;
  accelerations_ = accelerations;
  terminal_positions_ = terminal_positions;
  duration_seconds_ = duration_seconds;
  duration_ns_ = duration_ns;
  state_ = duration_seconds == 0.0 ?
    StopTrajectoryState::kHolding : StopTrajectoryState::kStopping;
  return true;
}

bool StopTrajectory::sample(std::uint64_t elapsed_ns, JointPoint & desired) noexcept
{
  if (state_ == StopTrajectoryState::kIdle) {
    return false;
  }
  if (elapsed_ns > std::numeric_limits<std::uint64_t>::max() - start_.time_ns) {
    return false;
  }

  JointPoint candidate;
  candidate.time_ns = start_.time_ns + elapsed_ns;
  const bool completed =
    state_ == StopTrajectoryState::kHolding || elapsed_ns >= duration_ns_;
  if (completed) {
    candidate.positions = terminal_positions_;
    candidate.velocities.fill(0.0);
  } else {
    const double elapsed_seconds = static_cast<double>(elapsed_ns) * 1.0e-9;
    for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
      candidate.positions[axis] =
        start_.positions[axis] + start_.velocities[axis] * elapsed_seconds +
        0.5 * accelerations_[axis] * elapsed_seconds * elapsed_seconds;
      candidate.velocities[axis] =
        start_.velocities[axis] + accelerations_[axis] * elapsed_seconds;
      if (
        !std::isfinite(candidate.positions[axis]) ||
        !std::isfinite(candidate.velocities[axis]))
      {
        return false;
      }
    }
  }

  desired = candidate;
  if (completed) {
    state_ = StopTrajectoryState::kHolding;
  }
  return true;
}

bool StopTrajectory::configured() const noexcept
{
  return limit_checker_.configured();
}

StopTrajectoryState StopTrajectory::state() const noexcept
{
  return state_;
}

double StopTrajectory::durationSeconds() const noexcept
{
  return duration_seconds_;
}

std::uint64_t StopTrajectory::durationNs() const noexcept
{
  return duration_ns_;
}

const std::array<double, kAxisCount> & StopTrajectory::accelerations() const noexcept
{
  return accelerations_;
}

const std::array<double, kAxisCount> & StopTrajectory::terminalPositions() const noexcept
{
  return terminal_positions_;
}

bool hasSufficientStoppingHorizon(
  std::uint64_t execution_time_ns, std::uint64_t buffered_until_ns,
  std::uint64_t stop_duration_ns, const SchedulingGuard & guard) noexcept
{
  if (buffered_until_ns < execution_time_ns) {
    return false;
  }
  std::uint64_t required = stop_duration_ns;
  if (!addWithoutOverflow(required, guard.one_cycle_detection_ns, required) ||
    !addWithoutOverflow(required, guard.stop_time_growth_ns, required) ||
    !addWithoutOverflow(required, guard.non_rt_to_rt_visibility_ns, required) ||
    !addWithoutOverflow(required, guard.period_quantization_ns, required))
  {
    return false;
  }
  return buffered_until_ns - execution_time_ns > required;
}

}  // namespace rolling_trajectory_controller
