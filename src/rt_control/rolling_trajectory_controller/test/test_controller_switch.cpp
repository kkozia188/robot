#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "controller_manager/controller_manager.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rolling_trajectory_controller/rolling_trajectory_controller.hpp"
#include "rolling_trajectory_controller/rolling_types.hpp"

namespace rolling_trajectory_controller
{
namespace
{

using namespace std::chrono_literals;

class FakeStoppedJtc final : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration configuration;
    configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const char * joint : kJointNames) {
      configuration.names.emplace_back(std::string(joint) + "/position");
    }
    return configuration;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration configuration;
    configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const char * joint : kJointNames) {
      configuration.names.emplace_back(std::string(joint) + "/position");
    }
    return configuration;
  }

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &) override
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override
  {
    if (command_interfaces_.size() != kAxisCount || state_interfaces_.size() != kAxisCount) {
      return controller_interface::CallbackReturn::ERROR;
    }
    for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
      hold_[axis] = state_interfaces_[axis].get_value();
      command_interfaces_[axis].set_value(hold_[axis]);
    }
    active_.store(true, std::memory_order_release);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override
  {
    active_.store(false, std::memory_order_release);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    if (active_.load(std::memory_order_acquire)) {
      for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
        command_interfaces_[axis].set_value(hold_[axis]);
      }
    }
    return controller_interface::return_type::OK;
  }

private:
  std::array<double, kAxisCount> hold_{};
  std::atomic_bool active_{false};
};

class RclcppContextGuard
{
public:
  RclcppContextGuard()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
      owns_context_ = true;
    }
  }

  ~RclcppContextGuard()
  {
    if (owns_context_ && rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  RclcppContextGuard(const RclcppContextGuard &) = delete;
  RclcppContextGuard & operator=(const RclcppContextGuard &) = delete;

private:
  bool owns_context_{false};
};

class UpdateLoopGuard
{
public:
  explicit UpdateLoopGuard(std::shared_ptr<controller_manager::ControllerManager> manager)
  : manager_(std::move(manager)), thread_([this]() {run();})
  {
  }

  ~UpdateLoopGuard()
  {
    running_.store(false, std::memory_order_release);
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  UpdateLoopGuard(const UpdateLoopGuard &) = delete;
  UpdateLoopGuard & operator=(const UpdateLoopGuard &) = delete;

private:
  void run()
  {
    rclcpp::Time time(0);
    const rclcpp::Duration period = rclcpp::Duration::from_nanoseconds(4'000'000);
    while (running_.load(std::memory_order_acquire)) {
      (void)manager_->read(time, period);
      (void)manager_->update(time, period);
      (void)manager_->write(time, period);
      time = time + period;
      std::this_thread::sleep_for(1ms);
    }
  }

  std::shared_ptr<controller_manager::ControllerManager> manager_;
  std::atomic_bool running_{true};
  std::thread thread_;
};

std::string makeMockUrdf()
{
  std::ostringstream urdf;
  urdf << "<?xml version=\"1.0\"?><robot name=\"rolling_switch_test\">";
  urdf << "<ros2_control name=\"mock_arms\" type=\"system\"><hardware>";
  urdf << "<plugin>mock_components/GenericSystem</plugin></hardware>";
  for (const char * joint : kJointNames) {
    urdf << "<joint name=\"" << joint << "\">";
    urdf << "<command_interface name=\"position\"/>";
    urdf << "<state_interface name=\"position\">";
    urdf << "<param name=\"initial_value\">0.0</param></state_interface></joint>";
  }
  urdf << "<sensor name=\"ethercat_domain\">";
  urdf << "<state_interface name=\"process_data_age_ms\">";
  urdf << "<param name=\"initial_value\">0.0</param></state_interface></sensor>";
  urdf << "</ros2_control></robot>";
  return urdf.str();
}

bool controllerIsActive(
  const controller_manager::ControllerManager & manager, const std::string & name)
{
  for (const auto & specification : manager.get_loaded_controllers()) {
    if (specification.info.name == name) {
      return specification.c->get_state().id() ==
             lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    }
  }
  return false;
}

TEST(RollingControllerPlugin, LoadsByItsExportedClassName)
{
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
    "controller_interface", "controller_interface::ControllerInterface");
  ASSERT_TRUE(
    loader.isClassAvailable(
      "rolling_trajectory_controller/RollingTrajectoryController"));
  const auto instance = loader.createSharedInstance(
    "rolling_trajectory_controller/RollingTrajectoryController");
  EXPECT_NE(instance, nullptr);
}

TEST(RollingControllerSwitch, StrictSwitchNeverDoubleClaimsPositionCommands)
{
  RclcppContextGuard context_guard;
  auto resource_manager = std::make_unique<hardware_interface::ResourceManager>(
    makeMockUrdf(), true, true);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto manager = std::make_shared<controller_manager::ControllerManager>(
    std::move(resource_manager), executor, "test_rolling_controller_manager");

  const auto source = manager->add_controller(
    std::make_shared<FakeStoppedJtc>(), "dual_arm_jtc", "test/FakeStoppedJtc");
  const auto rolling = manager->add_controller(
    std::make_shared<RollingTrajectoryController>(), "rolling_joint_controller",
    "rolling_trajectory_controller/RollingTrajectoryController");
  ASSERT_NE(source, nullptr);
  ASSERT_NE(rolling, nullptr);
  rolling->get_node()->set_parameters(
      {
        rclcpp::Parameter("configuration_source", "test_only"),
        rclcpp::Parameter("allow_test_only_configuration", true),
        rclcpp::Parameter("test_only_takeover_tolerances", std::vector<double>(kAxisCount, 0.1))});
  ASSERT_EQ(
    manager->configure_controller("dual_arm_jtc"),
    controller_interface::return_type::OK);
  ASSERT_EQ(
    manager->configure_controller("rolling_joint_controller"),
    controller_interface::return_type::OK);

  UpdateLoopGuard update_loop(manager);

  const auto strict_switch = [&](const std::vector<std::string> & start,
      const std::vector<std::string> & stop) {
      return manager->switch_controller(
        start, stop,
        controller_manager_msgs::srv::SwitchController::Request::STRICT,
        false, rclcpp::Duration::from_seconds(1.0));
    };

  ASSERT_EQ(strict_switch({"dual_arm_jtc"}, {}), controller_interface::return_type::OK);
  EXPECT_TRUE(controllerIsActive(*manager, "dual_arm_jtc"));
  EXPECT_FALSE(controllerIsActive(*manager, "rolling_joint_controller"));

  EXPECT_EQ(
    strict_switch({"rolling_joint_controller"}, {}),
    controller_interface::return_type::ERROR);
  EXPECT_TRUE(controllerIsActive(*manager, "dual_arm_jtc"));
  EXPECT_FALSE(controllerIsActive(*manager, "rolling_joint_controller"));

  ASSERT_EQ(
    strict_switch({"rolling_joint_controller"}, {"dual_arm_jtc"}),
    controller_interface::return_type::OK);
  EXPECT_FALSE(controllerIsActive(*manager, "dual_arm_jtc"));
  EXPECT_TRUE(controllerIsActive(*manager, "rolling_joint_controller"));

  EXPECT_EQ(
    strict_switch({"dual_arm_jtc"}, {}),
    controller_interface::return_type::ERROR);
  EXPECT_FALSE(controllerIsActive(*manager, "dual_arm_jtc"));
  EXPECT_TRUE(controllerIsActive(*manager, "rolling_joint_controller"));

  EXPECT_EQ(
    strict_switch({"dual_arm_jtc"}, {"rolling_joint_controller"}),
    controller_interface::return_type::OK);
  EXPECT_TRUE(controllerIsActive(*manager, "dual_arm_jtc"));
  EXPECT_FALSE(controllerIsActive(*manager, "rolling_joint_controller"));

  (void)strict_switch({}, {"dual_arm_jtc"});
}

}  // namespace
}  // namespace rolling_trajectory_controller
