#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rolling_trajectory_controller/rolling_trajectory_controller.hpp"
#include "rolling_trajectory_controller/rolling_types.hpp"

namespace rolling_trajectory_controller
{
namespace
{

constexpr double kTestOnlyTakeoverTolerance = 0.125;

class ControllerLifecycleTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
      actual_positions_[axis] = static_cast<double>(axis) * 0.25;
      command_positions_[axis] = actual_positions_[axis] + 0.01;
      command_handles_.emplace_back(kJointNames[axis], "position", &command_positions_[axis]);
      state_handles_.emplace_back(kJointNames[axis], "position", &actual_positions_[axis]);
    }
    feedback_age_ms_ = 12.0;
    state_handles_.emplace_back(
      "ethercat_domain", "process_data_age_ms", &feedback_age_ms_);

    auto options = rclcpp::NodeOptions();
    options.parameter_overrides({
      rclcpp::Parameter("configuration_source", "test_only"),
      rclcpp::Parameter("allow_test_only_configuration", true),
      rclcpp::Parameter(
        "test_only_takeover_tolerances",
        std::vector<double>(kAxisCount, kTestOnlyTakeoverTolerance))});
    controller_ = std::make_unique<RollingTrajectoryController>();
    ASSERT_EQ(
      controller_->init("test_rolling_controller", "", options),
      controller_interface::return_type::OK);
    ASSERT_EQ(
      controller_->on_configure(rclcpp_lifecycle::State()),
      controller_interface::CallbackReturn::SUCCESS);
  }

  void TearDown() override
  {
    if (controller_) {
      (void)controller_->on_deactivate(rclcpp_lifecycle::State());
    }
    controller_.reset();
  }

  void assignInterfaces(
    std::size_t command_count = kAxisCount,
    std::size_t state_count = kAxisCount + 1U)
  {
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    std::vector<hardware_interface::LoanedStateInterface> states;
    commands.reserve(command_count);
    states.reserve(state_count);
    for (std::size_t axis = 0U; axis < command_count; ++axis) {
      commands.emplace_back(command_handles_[axis]);
    }
    for (std::size_t axis = 0U; axis < state_count; ++axis) {
      states.emplace_back(state_handles_[axis]);
    }
    controller_->assign_interfaces(std::move(commands), std::move(states));
  }

  std::array<double, kAxisCount> command_positions_{};
  std::array<double, kAxisCount> actual_positions_{};
  double feedback_age_ms_{0.0};
  std::vector<hardware_interface::CommandInterface> command_handles_{};
  std::vector<hardware_interface::StateInterface> state_handles_{};
  std::unique_ptr<RollingTrajectoryController> controller_{};
};

TEST_F(ControllerLifecycleTest, ClaimsFourteenPositionPairsAndTheSharedFeedbackAge)
{
  const auto command_configuration = controller_->command_interface_configuration();
  const auto state_configuration = controller_->state_interface_configuration();
  ASSERT_EQ(
    command_configuration.type,
    controller_interface::interface_configuration_type::INDIVIDUAL);
  ASSERT_EQ(
    state_configuration.type,
    controller_interface::interface_configuration_type::INDIVIDUAL);
  ASSERT_EQ(command_configuration.names.size(), kAxisCount);
  ASSERT_EQ(state_configuration.names.size(), kAxisCount + 1U);
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    EXPECT_EQ(
      command_configuration.names[axis],
      std::string(kJointNames[axis]) + "/position");
    EXPECT_EQ(
      state_configuration.names[axis],
      std::string(kJointNames[axis]) + "/position");
  }
  EXPECT_EQ(
    state_configuration.names.back(),
    "ethercat_domain/process_data_age_ms");
}

TEST_F(ControllerLifecycleTest, ConfigurationRefusesImplicitOrUnapprovedTestThresholds)
{
  RollingTrajectoryController implicit;
  ASSERT_EQ(
    implicit.init("implicit_rolling_controller"),
    controller_interface::return_type::OK);
  EXPECT_EQ(
    implicit.on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);

  auto options = rclcpp::NodeOptions();
  options.parameter_overrides({
    rclcpp::Parameter("configuration_source", "test_only"),
    rclcpp::Parameter("allow_test_only_configuration", false),
    rclcpp::Parameter(
      "test_only_takeover_tolerances",
      std::vector<double>(kAxisCount, kTestOnlyTakeoverTolerance))});
  RollingTrajectoryController unapproved;
  ASSERT_EQ(
    unapproved.init("unapproved_rolling_controller", "", options),
    controller_interface::return_type::OK);
  EXPECT_EQ(
    unapproved.on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(ControllerLifecycleTest, ActivationRequiresEveryClaimedInterface)
{
  assignInterfaces(kAxisCount, kAxisCount);
  EXPECT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(ControllerLifecycleTest, FirstUpdatePreservesThePersistedSourceCommandExactly)
{
  assignInterfaces();
  const auto source_commands = command_positions_;
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(command_positions_, source_commands);

  ASSERT_EQ(
    controller_->update(
      rclcpp::Time(0), rclcpp::Duration::from_nanoseconds(4'000'000)),
    controller_interface::return_type::OK);
  EXPECT_EQ(command_positions_, source_commands);
}

TEST_F(ControllerLifecycleTest, NonFiniteSourceCommandFailsWithoutPartialWrites)
{
  command_positions_[6] = std::numeric_limits<double>::quiet_NaN();
  assignInterfaces();
  const auto before = command_positions_;
  EXPECT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    if (axis == 6U) {
      EXPECT_TRUE(std::isnan(command_positions_[axis]));
    } else {
      EXPECT_DOUBLE_EQ(command_positions_[axis], before[axis]);
    }
  }
}

TEST_F(ControllerLifecycleTest, NonFiniteActualPositionFailsActivation)
{
  actual_positions_[4] = std::numeric_limits<double>::infinity();
  assignInterfaces();
  EXPECT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(ControllerLifecycleTest, ExactTakeoverTolerancePasses)
{
  command_positions_[3] = actual_positions_[3] + kTestOnlyTakeoverTolerance;
  assignInterfaces();
  EXPECT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(ControllerLifecycleTest, OneRepresentableStepPastTakeoverToleranceFails)
{
  command_positions_[3] = std::nextafter(
    actual_positions_[3] + kTestOnlyTakeoverTolerance,
    std::numeric_limits<double>::infinity());
  assignInterfaces();
  EXPECT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(ControllerLifecycleTest, DeactivationDoesNotRewriteTheHeldCommand)
{
  assignInterfaces();
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    controller_->update(
      rclcpp::Time(0), rclcpp::Duration::from_nanoseconds(4'000'000)),
    controller_interface::return_type::OK);
  const auto held = command_positions_;
  EXPECT_EQ(
    controller_->on_deactivate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(command_positions_, held);
}

}  // namespace
}  // namespace rolling_trajectory_controller
