#ifndef ROLLING_TRAJECTORY_CONTROLLER__LIMIT_CHECKER_HPP_
#define ROLLING_TRAJECTORY_CONTROLLER__LIMIT_CHECKER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

#include "rolling_trajectory_controller/cubic_hermite.hpp"
#include "rolling_trajectory_controller/rolling_types.hpp"

namespace rolling_trajectory_controller
{

enum class LimitsSource : std::uint8_t
{
  kUnspecified = 0U,
  kProduction = 1U,
  kTestOnly = 2U
};

struct AxisEnvelope
{
  double position_lower{0.0};
  double position_upper{0.0};
  double velocity_positive{0.0};
  double velocity_negative{0.0};
  double acceleration_positive{0.0};
  double acceleration_negative{0.0};
  double stop_acceleration_positive{0.0};
  double stop_acceleration_negative{0.0};
  double position_margin_lower{0.0};
  double position_margin_upper{0.0};
};

struct DynamicEnvelope
{
  std::array<AxisEnvelope, kAxisCount> axes{};
  std::array<std::uint8_t, 32> limits_version{};
  LimitsSource source{LimitsSource::kUnspecified};
};

struct SegmentExtrema
{
  std::array<ScalarKinematicExtrema, kAxisCount> axes{};
};

struct SegmentCheckResult
{
  RejectCode code{RejectCode::kNone};
  std::size_t axis{kAxisCount};
};

struct StoppingEnvelope
{
  double duration_seconds{0.0};
  std::array<double, kAxisCount> lower_positions{};
  std::array<double, kAxisCount> upper_positions{};
};

class LimitChecker
{
public:
  bool configure(
    const DynamicEnvelope & envelope, bool allow_test_only_limits) noexcept;
  [[nodiscard]] bool configured() const noexcept;
  [[nodiscard]] const DynamicEnvelope & envelope() const noexcept;

  SegmentCheckResult checkSegment(
    const JointPoint & start, const JointPoint & end,
    SegmentExtrema & extrema) const noexcept;
  SegmentCheckResult checkStoppingViability(
    const SegmentExtrema & extrema,
    StoppingEnvelope & stopping_envelope) const noexcept;

private:
  DynamicEnvelope envelope_{};
  bool configured_{false};
};

RejectCode checkSpliceContinuity(
  const JointPoint & expected, const JointPoint & replacement,
  const std::array<double, kAxisCount> & position_tolerance,
  const std::array<double, kAxisCount> & velocity_tolerance) noexcept;

}  // namespace rolling_trajectory_controller

#endif  // ROLLING_TRAJECTORY_CONTROLLER__LIMIT_CHECKER_HPP_
