#ifndef ROLLING_TRAJECTORY_CONTROLLER__ROLLING_TYPES_HPP_
#define ROLLING_TRAJECTORY_CONTROLLER__ROLLING_TYPES_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace rolling_trajectory_controller
{

inline constexpr std::size_t kAxisCount = 14U;
inline constexpr std::size_t kTransportMaxPoints = 256U;
inline constexpr std::uint16_t kProtocolMajor = 1U;
inline constexpr std::uint16_t kProtocolMinor = 0U;

inline constexpr std::array<const char *, kAxisCount> kJointNames = {
  "right_joint1", "right_joint2", "right_joint3", "right_joint4", "right_joint5",
  "right_joint6", "left_joint1", "left_joint2", "left_joint3", "left_joint4",
  "left_joint5", "left_joint6", "turn", "updown"};

using Identifier = std::array<std::uint8_t, 16>;

enum class RejectCode : std::uint8_t
{
  kNone = 0U,
  kWrongProtocol = 1U,
  kWrongBoot = 2U,
  kWrongSession = 3U,
  kWrongClient = 4U,
  kStaleSequence = 5U,
  kInvalidShape = 6U,
  kNonFinite = 7U,
  kNonMonotonicTime = 8U,
  kLateReplace = 9U,
  kTimeGap = 10U,
  kCapacityExceeded = 11U,
  kInsufficientHorizon = 12U,
  kPositionDiscontinuity = 13U,
  kVelocityDiscontinuity = 14U,
  kPositionLimit = 15U,
  kVelocityLimit = 16U,
  kAccelerationLimit = 17U,
  kNotStoppingViable = 18U,
  kSessionNotAccepting = 19U
};

struct PointView
{
  std::uint64_t time_ns{0U};
  const double * positions{nullptr};
  std::size_t position_count{0U};
  const double * velocities{nullptr};
  std::size_t velocity_count{0U};
};

struct BatchView
{
  std::uint16_t protocol_major{0U};
  std::uint16_t protocol_minor{0U};
  Identifier controller_boot_id{};
  Identifier session_id{};
  Identifier client_instance_id{};
  std::uint64_t sequence{0U};
  std::uint64_t replace_from_ns{0U};
  const PointView * points{nullptr};
  std::size_t point_count{0U};
};

struct SessionIdentity
{
  Identifier controller_boot_id{};
  Identifier session_id{};
  Identifier client_instance_id{};
};

enum class SessionState : std::uint8_t
{
  kNone = 0U,
  kPriming = 1U,
  kRunning = 2U,
  kStopping = 3U,
  kHolding = 4U,
  kTerminated = 5U
};

enum class StopReason : std::uint8_t
{
  kNone = 0U,
  kGracefulClose = 1U,
  kPrimeTimeout = 2U,
  kUpdateTimeout = 3U,
  kLowWater = 4U,
  kClockAnomaly = 5U,
  kInternalInvariant = 6U,
  kControllerDeactivated = 7U,
  kDisable = 8U,
  kGroupFault = 9U,
  kControllerRestart = 10U
};

struct JointPoint
{
  std::uint64_t time_ns{0U};
  std::array<double, kAxisCount> positions{};
  std::array<double, kAxisCount> velocities{};
};

struct TrajectoryImage
{
  std::array<JointPoint, kTransportMaxPoints> points{};
  std::size_t point_count{0U};
  std::uint64_t generation{0U};
  std::uint64_t earliest_changed_ns{0U};
};

static_assert(std::is_trivially_copyable_v<JointPoint>);
static_assert(std::is_trivially_copyable_v<TrajectoryImage>);

}  // namespace rolling_trajectory_controller

#endif  // ROLLING_TRAJECTORY_CONTROLLER__ROLLING_TYPES_HPP_
