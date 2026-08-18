#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "enable_manager/enable_manager_controller.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/srv/rt_enable.hpp"

namespace enable_manager
{

class EnableManagerTestPeer
{
public:
  static const std::vector<std::string> & motionControllerNames(
    const EnableManagerController & controller)
  {
    return controller.motion_controller_names_;
  }

  static std::string defaultMotionControllerName(
    const EnableManagerController & controller)
  {
    return controller.motion_controller_names_.at(controller.default_motion_controller_index_);
  }

  static std::string activeMotionControllerName(
    const EnableManagerController & controller)
  {
    const std::size_t index =
      controller.active_motion_controller_index_.load(std::memory_order_acquire);
    return index < controller.motion_controller_names_.size() ?
           controller.motion_controller_names_[index] : "";
  }
};

namespace
{

using namespace std::chrono_literals;

constexpr std::size_t kAxisCount = 14U;
constexpr std::int64_t kPeriodNanoseconds = 4'000'000;

const std::array<const char *, kAxisCount> kJointNames = {
  "right_joint1", "right_joint2", "right_joint3", "right_joint4", "right_joint5",
  "right_joint6", "left_joint1", "left_joint2", "left_joint3", "left_joint4",
  "left_joint5", "left_joint6", "turn", "updown"};

enum class SwitchBehavior : std::uint8_t {kSuccess, kFailure, kTimeout};

class EnableManagerCharacterizationTest : public ::testing::Test
{
protected:
  using EnableFuture = rclcpp::Client<robot_interfaces::srv::RtEnable>::SharedFuture;
  using SwitchRequest = controller_manager_msgs::srv::SwitchController::Request;

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
    command_values_.fill(-1.0);
    status_values_.fill(0x0040);
    command_handles_.reserve(kAxisCount);
    state_handles_.reserve(kAxisCount);
    for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
      command_handles_.emplace_back(
        kJointNames[axis], "control_word", &command_values_[axis]);
      state_handles_.emplace_back(
        kJointNames[axis], "status_word", &status_values_[axis]);
    }

    auto options = rclcpp::NodeOptions();
    options.parameter_overrides(
        {
          rclcpp::Parameter("batch_timeout", 0.25),
          rclcpp::Parameter("disable_stage_timeout", 0.25),
          rclcpp::Parameter("inter_batch_delay", 0.0),
          rclcpp::Parameter("fault_reset_timeout", 0.25),
          rclcpp::Parameter("controller_switch_timeout", 0.02),
          rclcpp::Parameter("service_result_timeout_ms", 2000),
          rclcpp::Parameter("jtc_name", "dual_arm_jtc")});

    controller_ = std::make_unique<EnableManagerController>();
    ASSERT_EQ(
      controller_->init("test_enable_manager", "", options),
      controller_interface::return_type::OK);
    ASSERT_EQ(
      controller_->on_configure(rclcpp_lifecycle::State()),
      controller_interface::CallbackReturn::SUCCESS);

    std::vector<hardware_interface::LoanedCommandInterface> command_interfaces;
    std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
    command_interfaces.reserve(kAxisCount);
    state_interfaces.reserve(kAxisCount);
    for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
      command_interfaces.emplace_back(command_handles_[axis]);
      state_interfaces.emplace_back(state_handles_[axis]);
    }
    controller_->assign_interfaces(
      std::move(command_interfaces), std::move(state_interfaces));

    const std::size_t fixture_id = next_fixture_id_.fetch_add(1U);
    switch_node_ = std::make_shared<rclcpp::Node>(
      "test_switch_server_" + std::to_string(fixture_id));
    switch_service_ =
      switch_node_->create_service<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller",
      [this](
        const std::shared_ptr<SwitchRequest> request,
        std::shared_ptr<controller_manager_msgs::srv::SwitchController::Response> response)
      {
        {
          std::lock_guard<std::mutex> lock(switch_requests_mutex_);
          switch_requests_.push_back(*request);
        }
        const SwitchBehavior behavior = switch_behavior_.load();
        if (behavior == SwitchBehavior::kTimeout) {
          std::this_thread::sleep_for(100ms);
        }
        response->ok = behavior == SwitchBehavior::kSuccess;
      });

    client_node_ = std::make_shared<rclcpp::Node>(
      "test_enable_client_" + std::to_string(fixture_id));
    enable_client_ =
      client_node_->create_client<robot_interfaces::srv::RtEnable>("/rt/enable");
    disable_client_ =
      client_node_->create_client<robot_interfaces::srv::RtEnable>("/rt/disable");
    reset_client_ =
      client_node_->create_client<robot_interfaces::srv::RtEnable>("/rt/reset_fault");

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 4U);
    executor_->add_node(controller_->get_node()->get_node_base_interface());
    executor_->add_node(switch_node_);
    executor_->add_node(client_node_);
    executor_thread_ = std::thread([this]() {executor_->spin();});

    ASSERT_EQ(
      controller_->on_activate(rclcpp_lifecycle::State()),
      controller_interface::CallbackReturn::SUCCESS);
    ASSERT_TRUE(enable_client_->wait_for_service(1s));
    ASSERT_TRUE(disable_client_->wait_for_service(1s));
    ASSERT_TRUE(reset_client_->wait_for_service(1s));

    for (std::size_t step = 0; step < 4U; ++step) {
      updateAndEmulateDrive();
    }
  }

  void TearDown() override
  {
    if (controller_) {
      (void)controller_->on_deactivate(rclcpp_lifecycle::State());
    }
    if (executor_) {
      executor_->cancel();
    }
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
    executor_.reset();
    enable_client_.reset();
    disable_client_.reset();
    reset_client_.reset();
    switch_service_.reset();
    client_node_.reset();
    switch_node_.reset();
    controller_.reset();
  }

  void updateController()
  {
    ASSERT_EQ(
      controller_->update(
        rclcpp::Time(now_nanoseconds_),
        rclcpp::Duration::from_nanoseconds(kPeriodNanoseconds)),
      controller_interface::return_type::OK);
    now_nanoseconds_ += kPeriodNanoseconds;
    std::this_thread::sleep_for(1ms);
  }

  void emulateDriveCommands()
  {
    for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
      const auto control_word = static_cast<std::uint16_t>(command_values_[axis]);
      switch (control_word) {
        case 0x0000U:
          status_values_[axis] = 0x0040;
          break;
        case 0x0002U:
          status_values_[axis] = 0x0007;
          break;
        case 0x0006U:
          status_values_[axis] = 0x0021;
          break;
        case 0x0007U:
          status_values_[axis] = 0x0023;
          break;
        case 0x000FU:
          status_values_[axis] = 0x0027;
          break;
        case 0x0080U:
          status_values_[axis] = 0x0040;
          break;
        default:
          FAIL() << "Unexpected control word " << control_word << " on axis " << axis;
      }
    }
  }

  void updateAndEmulateDrive()
  {
    updateController();
    emulateDriveCommands();
  }

  bool driveUntilReady(const EnableFuture & future, std::size_t max_steps = 500U)
  {
    for (std::size_t step = 0; step < max_steps; ++step) {
      if (future.wait_for(0ms) == std::future_status::ready) {
        return true;
      }
      updateAndEmulateDrive();
    }
    return future.wait_for(0ms) == std::future_status::ready;
  }

  EnableFuture callEnable()
  {
    auto request = enable_client_->async_send_request(
      std::make_shared<robot_interfaces::srv::RtEnable::Request>());
    return request.future.share();
  }

  EnableFuture callDisable()
  {
    auto request = disable_client_->async_send_request(
      std::make_shared<robot_interfaces::srv::RtEnable::Request>());
    return request.future.share();
  }

  EnableFuture callResetFault()
  {
    auto request = reset_client_->async_send_request(
      std::make_shared<robot_interfaces::srv::RtEnable::Request>());
    return request.future.share();
  }

  std::vector<SwitchRequest> switchRequests() const
  {
    std::lock_guard<std::mutex> lock(switch_requests_mutex_);
    return switch_requests_;
  }

  bool waitForSwitchRequests(std::size_t expected_count)
  {
    for (std::size_t step = 0; step < 300U; ++step) {
      if (switchRequests().size() >= expected_count) {
        return true;
      }
      updateAndEmulateDrive();
    }
    return switchRequests().size() >= expected_count;
  }

  static std::atomic_size_t next_fixture_id_;
  std::array<double, kAxisCount> command_values_{};
  std::array<double, kAxisCount> status_values_{};
  std::vector<hardware_interface::CommandInterface> command_handles_;
  std::vector<hardware_interface::StateInterface> state_handles_;
  std::unique_ptr<EnableManagerController> controller_;
  rclcpp::Node::SharedPtr switch_node_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Service<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_service_;
  rclcpp::Client<robot_interfaces::srv::RtEnable>::SharedPtr enable_client_;
  rclcpp::Client<robot_interfaces::srv::RtEnable>::SharedPtr disable_client_;
  rclcpp::Client<robot_interfaces::srv::RtEnable>::SharedPtr reset_client_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread executor_thread_;
  std::atomic<SwitchBehavior> switch_behavior_{SwitchBehavior::kSuccess};
  mutable std::mutex switch_requests_mutex_;
  std::vector<SwitchRequest> switch_requests_;
  std::int64_t now_nanoseconds_{0};
};

std::atomic_size_t EnableManagerCharacterizationTest::next_fixture_id_{0U};

TEST_F(EnableManagerCharacterizationTest, ClaimsTheFrozenFourteenAxisCia402Interfaces)
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
  ASSERT_EQ(state_configuration.names.size(), kAxisCount);
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    EXPECT_EQ(
      command_configuration.names[axis],
      std::string(kJointNames[axis]) + "/control_word");
    EXPECT_EQ(
      state_configuration.names[axis],
      std::string(kJointNames[axis]) + "/status_word");
    EXPECT_EQ(command_values_[axis], 0.0);
  }
}

TEST_F(EnableManagerCharacterizationTest, EnablesAndDisablesOnlyDualArmJtcWithStrictSwitches)
{
  const auto enable_future = callEnable();
  ASSERT_TRUE(driveUntilReady(enable_future));
  const auto enable_response = enable_future.get();
  ASSERT_NE(enable_response, nullptr);
  EXPECT_TRUE(enable_response->ok);
  EXPECT_EQ(enable_response->stage, "success");

  auto requests = switchRequests();
  ASSERT_EQ(requests.size(), 1U);
  EXPECT_EQ(requests[0].activate_controllers, std::vector<std::string>({"dual_arm_jtc"}));
  EXPECT_TRUE(requests[0].deactivate_controllers.empty());
  EXPECT_EQ(
    requests[0].strictness,
    controller_manager_msgs::srv::SwitchController::Request::STRICT);
  EXPECT_TRUE(requests[0].activate_asap);
  for (const double command : command_values_) {
    EXPECT_EQ(command, 15.0);
  }

  const auto disable_future = callDisable();
  ASSERT_TRUE(driveUntilReady(disable_future));
  const auto disable_response = disable_future.get();
  ASSERT_NE(disable_response, nullptr);
  EXPECT_TRUE(disable_response->ok);
  EXPECT_EQ(disable_response->stage, "success");

  requests = switchRequests();
  ASSERT_EQ(requests.size(), 2U);
  EXPECT_TRUE(requests[1].activate_controllers.empty());
  EXPECT_EQ(requests[1].deactivate_controllers, std::vector<std::string>({"dual_arm_jtc"}));
  EXPECT_EQ(
    requests[1].strictness,
    controller_manager_msgs::srv::SwitchController::Request::STRICT);
  for (const double command : command_values_) {
    EXPECT_EQ(command, 0.0);
  }
}

TEST_F(EnableManagerCharacterizationTest, FaultQuickStopsAndDeactivatesDualArmJtc)
{
  const auto enable_future = callEnable();
  ASSERT_TRUE(driveUntilReady(enable_future));
  ASSERT_TRUE(enable_future.get()->ok);

  status_values_[3] = 0x0008;
  updateAndEmulateDrive();
  EXPECT_EQ(command_values_[3], 0.0);
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    if (axis != 3U) {
      EXPECT_EQ(command_values_[axis], 2.0);
    }
  }

  ASSERT_TRUE(waitForSwitchRequests(2U));
  const auto requests = switchRequests();
  EXPECT_TRUE(requests[1].activate_controllers.empty());
  EXPECT_EQ(requests[1].deactivate_controllers, std::vector<std::string>({"dual_arm_jtc"}));
}

TEST_F(EnableManagerCharacterizationTest, DisablePreemptsAnEnableBeforeControllerActivation)
{
  const auto enable_future = callEnable();
  bool enable_started = false;
  for (std::size_t step = 0; step < 50U; ++step) {
    updateController();
    if (command_values_[0] == 6.0) {
      enable_started = true;
      break;
    }
  }
  ASSERT_TRUE(enable_started);

  const auto disable_future = callDisable();
  ASSERT_TRUE(driveUntilReady(disable_future));
  ASSERT_EQ(enable_future.wait_for(0ms), std::future_status::ready);

  const auto enable_response = enable_future.get();
  ASSERT_NE(enable_response, nullptr);
  EXPECT_FALSE(enable_response->ok);
  EXPECT_EQ(enable_response->stage, "preempted_by_disable");

  const auto disable_response = disable_future.get();
  ASSERT_NE(disable_response, nullptr);
  EXPECT_TRUE(disable_response->ok);
  EXPECT_EQ(disable_response->stage, "success");
  EXPECT_TRUE(switchRequests().empty());
  for (const double command : command_values_) {
    EXPECT_EQ(command, 0.0);
  }
}

TEST_F(EnableManagerCharacterizationTest, ResetsOnlyTheFaultedAxisThenReturnsIdle)
{
  constexpr std::size_t kFaultedAxis = 5U;
  status_values_[kFaultedAxis] = 0x0008;
  const auto reset_future = callResetFault();

  bool reset_pulse_observed = false;
  for (std::size_t step = 0; step < 50U; ++step) {
    updateController();
    if (command_values_[kFaultedAxis] == 128.0) {
      reset_pulse_observed = true;
      break;
    }
  }
  ASSERT_TRUE(reset_pulse_observed);
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    if (axis != kFaultedAxis) {
      EXPECT_EQ(command_values_[axis], 0.0);
    }
  }

  status_values_[kFaultedAxis] = 0x0040;
  ASSERT_TRUE(driveUntilReady(reset_future));
  const auto reset_response = reset_future.get();
  ASSERT_NE(reset_response, nullptr);
  EXPECT_TRUE(reset_response->ok);
  EXPECT_EQ(reset_response->stage, "success");

  const auto disable_future = callDisable();
  ASSERT_TRUE(driveUntilReady(disable_future));
  const auto disable_response = disable_future.get();
  ASSERT_NE(disable_response, nullptr);
  EXPECT_TRUE(disable_response->ok);
  EXPECT_EQ(disable_response->stage, "already_disabled");
}

TEST_F(EnableManagerCharacterizationTest, AmbiguousActivationRequiresRestart)
{
  switch_behavior_.store(SwitchBehavior::kTimeout);
  const auto first_enable = callEnable();
  ASSERT_TRUE(driveUntilReady(first_enable));
  const auto first_response = first_enable.get();
  ASSERT_NE(first_response, nullptr);
  EXPECT_FALSE(first_response->ok);
  EXPECT_EQ(first_response->stage, "jtc_activate_failed");

  const auto second_enable = callEnable();
  ASSERT_TRUE(driveUntilReady(second_enable));
  const auto second_response = second_enable.get();
  ASSERT_NE(second_response, nullptr);
  EXPECT_FALSE(second_response->ok);
  EXPECT_EQ(second_response->stage, "restart_required");
}

TEST_F(EnableManagerCharacterizationTest, TracksTheDefaultControllerAsActive)
{
  EXPECT_EQ(
    EnableManagerTestPeer::motionControllerNames(*controller_),
    std::vector<std::string>({"dual_arm_jtc"}));
  EXPECT_EQ(
    EnableManagerTestPeer::defaultMotionControllerName(*controller_), "dual_arm_jtc");
  EXPECT_TRUE(EnableManagerTestPeer::activeMotionControllerName(*controller_).empty());

  const auto enable_future = callEnable();
  ASSERT_TRUE(driveUntilReady(enable_future));
  ASSERT_TRUE(enable_future.get()->ok);
  EXPECT_EQ(
    EnableManagerTestPeer::activeMotionControllerName(*controller_), "dual_arm_jtc");

  const auto disable_future = callDisable();
  ASSERT_TRUE(driveUntilReady(disable_future));
  ASSERT_TRUE(disable_future.get()->ok);
  EXPECT_TRUE(EnableManagerTestPeer::activeMotionControllerName(*controller_).empty());
}

class EnableManagerRegistryConfigurationTest : public ::testing::Test
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

  static std::unique_ptr<EnableManagerController> makeController(
    const std::string & node_name, const std::vector<rclcpp::Parameter> & overrides)
  {
    auto options = rclcpp::NodeOptions();
    options.parameter_overrides(overrides);
    auto controller = std::make_unique<EnableManagerController>();
    EXPECT_EQ(
      controller->init(node_name, "", options), controller_interface::return_type::OK);
    return controller;
  }
};

TEST_F(EnableManagerRegistryConfigurationTest, AcceptsAConfiguredControllerRegistry)
{
  auto controller = makeController(
    "test_enable_manager_registry",
      {
        rclcpp::Parameter(
          "motion_controller_names",
          std::vector<std::string>({"dual_arm_jtc", "rolling_joint_controller"})),
        rclcpp::Parameter("default_motion_controller_name", "dual_arm_jtc")
      });

  ASSERT_EQ(
    controller->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    EnableManagerTestPeer::motionControllerNames(*controller),
    std::vector<std::string>({"dual_arm_jtc", "rolling_joint_controller"}));
  EXPECT_EQ(
    EnableManagerTestPeer::defaultMotionControllerName(*controller), "dual_arm_jtc");
  EXPECT_TRUE(EnableManagerTestPeer::activeMotionControllerName(*controller).empty());
}

TEST_F(EnableManagerRegistryConfigurationTest, RejectsDuplicateControllerNames)
{
  auto controller = makeController(
    "test_enable_manager_duplicate_registry",
      {
        rclcpp::Parameter(
          "motion_controller_names",
          std::vector<std::string>({"dual_arm_jtc", "dual_arm_jtc"})),
        rclcpp::Parameter("default_motion_controller_name", "dual_arm_jtc")
      });

  EXPECT_EQ(
    controller->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

TEST_F(EnableManagerRegistryConfigurationTest, RejectsADefaultOutsideTheRegistry)
{
  auto controller = makeController(
    "test_enable_manager_unknown_default",
      {
        rclcpp::Parameter(
          "motion_controller_names",
          std::vector<std::string>({"dual_arm_jtc", "rolling_joint_controller"})),
        rclcpp::Parameter("default_motion_controller_name", "missing_controller")
      });

  EXPECT_EQ(
    controller->on_configure(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::ERROR);
}

}  // namespace
}  // namespace enable_manager
