#ifndef ROLLING_TRAJECTORY_CONTROLLER__SESSION_CORE_HPP_
#define ROLLING_TRAJECTORY_CONTROLLER__SESSION_CORE_HPP_

#include <array>
#include <cstdint>

#include "rolling_trajectory_controller/limit_checker.hpp"

namespace rolling_trajectory_controller
{

enum class StopTrajectoryState : std::uint8_t
{
  kIdle = 0U,
  kStopping = 1U,
  kHolding = 2U
};

struct SchedulingGuard
{
  std::uint64_t one_cycle_detection_ns{0U};
  std::uint64_t stop_time_growth_ns{0U};
  std::uint64_t non_rt_to_rt_visibility_ns{0U};
  std::uint64_t period_quantization_ns{0U};
};

class StopTrajectory
{
public:
  bool configure(
    const DynamicEnvelope & envelope, bool allow_test_only_limits) noexcept;
  bool begin(const JointPoint & desired) noexcept;
  bool sample(std::uint64_t elapsed_ns, JointPoint & desired) noexcept;

  [[nodiscard]] bool configured() const noexcept;
  [[nodiscard]] StopTrajectoryState state() const noexcept;
  [[nodiscard]] double durationSeconds() const noexcept;
  [[nodiscard]] std::uint64_t durationNs() const noexcept;
  [[nodiscard]] const std::array<double, kAxisCount> & accelerations() const noexcept;
  [[nodiscard]] const std::array<double, kAxisCount> & terminalPositions() const noexcept;

private:
  LimitChecker limit_checker_{};
  JointPoint start_{};
  std::array<double, kAxisCount> accelerations_{};
  std::array<double, kAxisCount> terminal_positions_{};
  double duration_seconds_{0.0};
  std::uint64_t duration_ns_{0U};
  StopTrajectoryState state_{StopTrajectoryState::kIdle};
};

bool hasSufficientStoppingHorizon(
  std::uint64_t execution_time_ns, std::uint64_t buffered_until_ns,
  std::uint64_t stop_duration_ns, const SchedulingGuard & guard) noexcept;

}  // namespace rolling_trajectory_controller

#endif  // ROLLING_TRAJECTORY_CONTROLLER__SESSION_CORE_HPP_
