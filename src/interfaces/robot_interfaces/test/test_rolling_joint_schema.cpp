#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "robot_interfaces/msg/rolling_joint_control_state.hpp"
#include "robot_interfaces/msg/rolling_joint_point.hpp"
#include "robot_interfaces/msg/rolling_joint_target_batch.hpp"
#include "robot_interfaces/srv/close_rolling_joint_session.hpp"
#include "robot_interfaces/srv/open_rolling_joint_session.hpp"
#include "robot_interfaces/srv/set_joint_control_mode.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

namespace robot_interfaces
{
namespace
{

using Point = msg::RollingJointPoint;
using Batch = msg::RollingJointTargetBatch;
using State = msg::RollingJointControlState;
using SetMode = srv::SetJointControlMode;
using Open = srv::OpenRollingJointSession;
using Close = srv::CloseRollingJointSession;
using Uuid = unique_identifier_msgs::msg::UUID;

template<std::size_t Size>
constexpr bool arraysEqual(
  const std::array<std::uint8_t, Size> & lhs,
  const std::array<std::uint8_t, Size> & rhs)
{
  for (std::size_t index = 0; index < Size; ++index) {
    if (lhs[index] != rhs[index]) {
      return false;
    }
  }
  return true;
}

constexpr std::array<std::uint8_t, 20> kExpectedServiceErrors = {
  0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U,
  10U, 11U, 12U, 13U, 14U, 15U, 16U, 17U, 18U, 19U};

#define ROLLING_SERVICE_ERRORS(Type) \
  std::array<std::uint8_t, 20>{ \
    Type::ERROR_NONE, Type::ERROR_WRONG_PROTOCOL, Type::ERROR_WRONG_REQUEST, \
    Type::ERROR_WRONG_MODE, Type::ERROR_NOT_ENABLED, Type::ERROR_SESSION_BUSY, \
    Type::ERROR_SESSION_EXISTS, Type::ERROR_WRONG_BOOT, Type::ERROR_WRONG_SESSION, \
    Type::ERROR_WRONG_CLIENT, Type::ERROR_AXIS_SET_MISMATCH, Type::ERROR_SOURCE_MOVING, \
    Type::ERROR_TAKEOVER_MISMATCH, Type::ERROR_UNSAFE_HOLD, Type::ERROR_FEEDBACK_STALE, \
    Type::ERROR_LIMITS_UNAVAILABLE, Type::ERROR_SWITCH_REJECTED, Type::ERROR_SWITCH_TIMEOUT, \
    Type::ERROR_RESTART_REQUIRED, Type::ERROR_NOT_READY}

constexpr auto kSetModeErrors = ROLLING_SERVICE_ERRORS(SetMode::Response);
constexpr auto kOpenErrors = ROLLING_SERVICE_ERRORS(Open::Response);
constexpr auto kCloseErrors = ROLLING_SERVICE_ERRORS(Close::Response);
constexpr auto kStateErrors = ROLLING_SERVICE_ERRORS(State);

static_assert(std::is_same_v<decltype(Point::positions), std::array<double, 14>>);
static_assert(std::is_same_v<decltype(Point::velocities), std::array<double, 14>>);
static_assert(std::is_same_v<decltype(Batch::controller_boot_id), Uuid>);
static_assert(std::is_same_v<decltype(Batch::session_id), Uuid>);
static_assert(std::is_same_v<decltype(Batch::client_instance_id), Uuid>);
static_assert(std::is_same_v<decltype(State::desired_positions), std::array<double, 14>>);
static_assert(std::is_same_v<decltype(State::desired_velocities), std::array<double, 14>>);
static_assert(std::is_same_v<decltype(State::axis_set_hash), std::array<std::uint8_t, 32>>);
static_assert(std::is_same_v<decltype(State::limits_version), std::array<std::uint8_t, 32>>);
static_assert(
  std::is_same_v<decltype(Open::Request::axis_set_hash), std::array<std::uint8_t, 32>>);
static_assert(
  std::is_same_v<decltype(Open::Response::axis_set_hash), std::array<std::uint8_t, 32>>);
static_assert(
  std::is_same_v<decltype(Open::Response::limits_version), std::array<std::uint8_t, 32>>);
static_assert(std::is_same_v<decltype(SetMode::Request::request_id), Uuid>);
static_assert(std::is_same_v<decltype(Open::Request::request_id), Uuid>);
static_assert(std::is_same_v<decltype(Close::Request::request_id), Uuid>);
static_assert(std::is_same_v<decltype(Open::Request::expected_controller_boot_id), Uuid>);
static_assert(std::is_same_v<decltype(Close::Request::controller_boot_id), Uuid>);
static_assert(std::is_same_v<decltype(Open::Request::client_instance_id), Uuid>);
static_assert(std::is_same_v<decltype(Close::Request::session_id), Uuid>);
static_assert(std::is_same_v<decltype(Close::Request::client_instance_id), Uuid>);
static_assert(std::is_same_v<decltype(Open::Response::session_id), Uuid>);
static_assert(arraysEqual(kSetModeErrors, kExpectedServiceErrors));
static_assert(arraysEqual(kOpenErrors, kExpectedServiceErrors));
static_assert(arraysEqual(kCloseErrors, kExpectedServiceErrors));
static_assert(arraysEqual(kStateErrors, kExpectedServiceErrors));
static_assert(Batch::TRANSPORT_MAX_POINTS == 256U);
static_assert(Batch::PROTOCOL_MAJOR == 1U);
static_assert(Batch::PROTOCOL_MINOR == 0U);
static_assert(SetMode::Request::PROTOCOL_MAJOR == Batch::PROTOCOL_MAJOR);
static_assert(Open::Request::PROTOCOL_MAJOR == Batch::PROTOCOL_MAJOR);
static_assert(Close::Request::PROTOCOL_MAJOR == Batch::PROTOCOL_MAJOR);
static_assert(State::PROTOCOL_MAJOR == Batch::PROTOCOL_MAJOR);
static_assert(SetMode::Request::PROTOCOL_MINOR == Batch::PROTOCOL_MINOR);
static_assert(Open::Request::PROTOCOL_MINOR == Batch::PROTOCOL_MINOR);
static_assert(Close::Request::PROTOCOL_MINOR == Batch::PROTOCOL_MINOR);
static_assert(State::PROTOCOL_MINOR == Batch::PROTOCOL_MINOR);

TEST(RollingJointSchema, BatchHasABoundedPayloadContract)
{
  Batch batch;
  EXPECT_EQ(batch.points.max_size(), Batch::TRANSPORT_MAX_POINTS);
  EXPECT_TRUE(batch.points.empty());
}

TEST(RollingJointSchema, ControlAndSessionStatesHaveStableValues)
{
  EXPECT_EQ(SetMode::Request::MODE_DISABLED, 0U);
  EXPECT_EQ(SetMode::Request::MODE_FJT_READY, 1U);
  EXPECT_EQ(SetMode::Request::MODE_ROLLING_READY, 2U);
  EXPECT_EQ(SetMode::Request::MODE_RESTART_REQUIRED, 3U);

  EXPECT_EQ(State::SESSION_NONE, 0U);
  EXPECT_EQ(State::SESSION_PRIMING, 1U);
  EXPECT_EQ(State::SESSION_RUNNING, 2U);
  EXPECT_EQ(State::SESSION_STOPPING, 3U);
  EXPECT_EQ(State::SESSION_HOLDING, 4U);
  EXPECT_EQ(State::SESSION_TERMINATED, 5U);
}

TEST(RollingJointSchema, ServiceErrorValuesAreSharedAcrossEndpoints)
{
  EXPECT_EQ(kSetModeErrors, kExpectedServiceErrors);
  EXPECT_EQ(kOpenErrors, kExpectedServiceErrors);
  EXPECT_EQ(kCloseErrors, kExpectedServiceErrors);
  EXPECT_EQ(kStateErrors, kExpectedServiceErrors);
}

TEST(RollingJointSchema, RejectAndStopReasonsRemainSeparate)
{
  EXPECT_EQ(State::REJECT_NONE, 0U);
  EXPECT_EQ(State::REJECT_SESSION_NOT_ACCEPTING, 19U);
  EXPECT_EQ(State::STOP_NONE, 0U);
  EXPECT_EQ(State::STOP_PRIME_TIMEOUT, 2U);
  EXPECT_EQ(State::STOP_CONTROLLER_RESTART, 10U);
}

}  // namespace
}  // namespace robot_interfaces

#undef ROLLING_SERVICE_ERRORS
