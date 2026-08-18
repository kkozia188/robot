#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/rolling_joint_control_state.hpp"
#include "robot_interfaces/msg/rolling_joint_target_batch.hpp"
#include "robot_interfaces/srv/close_rolling_joint_session.hpp"
#include "robot_interfaces/srv/open_rolling_joint_session.hpp"
#include "rolling_trajectory_controller/rolling_trajectory_controller.hpp"
#include "rolling_trajectory_controller/rolling_types.hpp"

namespace rolling_trajectory_controller
{

class RollingControllerStateTestPeer
{
public:
  using Batch = robot_interfaces::msg::RollingJointTargetBatch;
  using Close = robot_interfaces::srv::CloseRollingJointSession;
  using Open = robot_interfaces::srv::OpenRollingJointSession;
  using State = robot_interfaces::msg::RollingJointControlState;

  static Identifier bootId(RollingTrajectoryController & controller)
  {
    std::lock_guard<std::mutex> lock(controller.callback_mutex_);
    return controller.controller_boot_id_;
  }

  static Open::Response open(
    RollingTrajectoryController & controller, const Open::Request & request)
  {
    auto request_pointer = std::make_shared<Open::Request>(request);
    auto response = std::make_shared<Open::Response>();
    controller.handleOpen(request_pointer, response);
    return *response;
  }

  static Close::Response close(
    RollingTrajectoryController & controller, const Close::Request & request)
  {
    auto request_pointer = std::make_shared<Close::Request>(request);
    auto response = std::make_shared<Close::Response>();
    controller.handleClose(request_pointer, response);
    return *response;
  }

  static void submit(RollingTrajectoryController & controller, const Batch & batch)
  {
    controller.handleUpdate(std::make_shared<Batch>(batch));
  }

  static bool buildState(RollingTrajectoryController & controller, State & state)
  {
    return controller.buildPublicState(state);
  }

  static bool publishState(RollingTrajectoryController & controller)
  {
    return controller.publishPublicState();
  }

  static std::string stateTopic(const RollingTrajectoryController & controller)
  {
    return controller.state_publisher_->get_topic_name();
  }

  static rclcpp::QoS stateQos(const RollingTrajectoryController & controller)
  {
    return controller.state_publisher_->get_actual_qos();
  }

  static void activateStatePublisher(RollingTrajectoryController & controller)
  {
    controller.state_publisher_->on_activate();
  }

  static void expireAcceptedUpdate(RollingTrajectoryController & controller)
  {
    controller.rt_last_accepted_arrival_ns_ = 0U;
  }
};

namespace
{

using namespace std::chrono_literals;
using Batch = robot_interfaces::msg::RollingJointTargetBatch;
using Close = robot_interfaces::srv::CloseRollingJointSession;
using Open = robot_interfaces::srv::OpenRollingJointSession;
using State = robot_interfaces::msg::RollingJointControlState;

constexpr std::uint64_t kCycleNs = 4'000'000U;
constexpr std::uint64_t kTrajectoryDurationNs = 900'000'000U;
constexpr std::array<std::uint8_t, 32> kAxisSetHash = {
  0x25U, 0xc6U, 0xe8U, 0x2bU, 0xf5U, 0x05U, 0xcaU, 0x9eU,
  0xb9U, 0x9dU, 0xb1U, 0xc6U, 0x45U, 0xabU, 0x75U, 0xd7U,
  0xecU, 0xdeU, 0x01U, 0x53U, 0xfaU, 0xafU, 0x6aU, 0x74U,
  0x92U, 0xc6U, 0x21U, 0x0cU, 0x4dU, 0x36U, 0x25U, 0x26U};

unique_identifier_msgs::msg::UUID makeUuid(std::uint8_t seed)
{
  unique_identifier_msgs::msg::UUID id;
  for (std::size_t index = 0U; index < id.uuid.size(); ++index) {
    id.uuid[index] = static_cast<std::uint8_t>(seed + index);
  }
  return id;
}

Open::Request makeOpenRequest(const Identifier & boot_id)
{
  Open::Request request;
  request.protocol_major = Open::Request::PROTOCOL_MAJOR;
  request.protocol_minor = Open::Request::PROTOCOL_MINOR;
  request.request_id = makeUuid(20U);
  request.expected_controller_boot_id.uuid = boot_id;
  request.client_instance_id = makeUuid(40U);
  request.axis_set_hash = kAxisSetHash;
  return request;
}

Close::Request makeCloseRequest(const Open::Response & open, std::uint8_t request_seed)
{
  Close::Request request;
  request.protocol_major = Close::Request::PROTOCOL_MAJOR;
  request.protocol_minor = Close::Request::PROTOCOL_MINOR;
  request.request_id = makeUuid(request_seed);
  request.controller_boot_id = open.controller_boot_id;
  request.session_id = open.session_id;
  request.client_instance_id = open.client_instance_id;
  return request;
}

Batch makeTrajectory(const Open::Response & open)
{
  Batch batch;
  batch.protocol_major = Batch::PROTOCOL_MAJOR;
  batch.protocol_minor = Batch::PROTOCOL_MINOR;
  batch.controller_boot_id = open.controller_boot_id;
  batch.session_id = open.session_id;
  batch.client_instance_id = open.client_instance_id;
  batch.sequence = 1U;
  batch.replace_from_ns = 0U;
  batch.points.resize(2U);
  batch.points[0].time_from_session_start_ns = 0U;
  batch.points[1].time_from_session_start_ns = kTrajectoryDurationNs;
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    batch.points[0].positions[axis] = open.hold_positions[axis];
    batch.points[1].positions[axis] = open.hold_positions[axis] + 0.05;
    batch.points[0].velocities[axis] = 0.0;
    batch.points[1].velocities[axis] = 0.0;
  }
  return batch;
}

class StatePublisherTest : public ::testing::Test
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
    options.parameter_overrides(
        {
          rclcpp::Parameter("configuration_source", "test_only"),
          rclcpp::Parameter("allow_test_only_configuration", true),
          rclcpp::Parameter(
            "test_only_takeover_tolerances", std::vector<double>(kAxisCount, 0.125))});
    controller_ = std::make_unique<RollingTrajectoryController>();
    ASSERT_EQ(
      controller_->init("test_rolling_state", "", options),
      controller_interface::return_type::OK);
    ASSERT_EQ(
      controller_->on_configure(rclcpp_lifecycle::State()),
      controller_interface::CallbackReturn::SUCCESS);

    std::vector<hardware_interface::LoanedCommandInterface> commands;
    std::vector<hardware_interface::LoanedStateInterface> states;
    for (auto & handle : command_handles_) {
      commands.emplace_back(handle);
    }
    for (auto & handle : state_handles_) {
      states.emplace_back(handle);
    }
    controller_->assign_interfaces(std::move(commands), std::move(states));
    ASSERT_EQ(
      controller_->on_activate(rclcpp_lifecycle::State()),
      controller_interface::CallbackReturn::SUCCESS);
    RollingControllerStateTestPeer::activateStatePublisher(*controller_);
  }

  void TearDown() override
  {
    if (controller_) {
      (void)controller_->on_deactivate(rclcpp_lifecycle::State());
    }
  }

  Open::Response open()
  {
    return RollingControllerStateTestPeer::open(
      *controller_,
      makeOpenRequest(RollingControllerStateTestPeer::bootId(*controller_)));
  }

  controller_interface::return_type update()
  {
    return controller_->update(
      rclcpp::Time(0), rclcpp::Duration::from_nanoseconds(kCycleNs));
  }

  std::array<double, kAxisCount> command_positions_{};
  std::array<double, kAxisCount> actual_positions_{};
  double feedback_age_ms_{0.0};
  std::vector<hardware_interface::CommandInterface> command_handles_{};
  std::vector<hardware_interface::StateInterface> state_handles_{};
  std::unique_ptr<RollingTrajectoryController> controller_{};
};

TEST_F(StatePublisherTest, PrimingStateExposesTheSessionAndExplicitTestConfiguration)
{
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  const Open::Response response = open();
  ASSERT_TRUE(response.success);
  ASSERT_EQ(update(), controller_interface::return_type::OK);

  State state;
  ASSERT_TRUE(RollingControllerStateTestPeer::buildState(*controller_, state));
  EXPECT_EQ(state.protocol_major, State::PROTOCOL_MAJOR);
  EXPECT_EQ(state.protocol_minor, State::PROTOCOL_MINOR);
  EXPECT_EQ(state.controller_boot_id, response.controller_boot_id);
  EXPECT_EQ(state.session_id, response.session_id);
  EXPECT_EQ(state.client_instance_id, response.client_instance_id);
  EXPECT_EQ(state.control_mode, State::MODE_ROLLING_READY);
  EXPECT_EQ(state.session_state, State::SESSION_PRIMING);
  EXPECT_TRUE(state.has_session);
  EXPECT_FALSE(state.has_accepted_update);
  EXPECT_TRUE(state.test_only_limits);
  EXPECT_EQ(state.axis_set_hash, kAxisSetHash);
  EXPECT_EQ(state.limits_version, response.limits_version);
  EXPECT_EQ(state.buffer_capacity, response.buffer_capacity);
  EXPECT_EQ(state.active_generation, 0U);
  EXPECT_EQ(state.validation_base_generation, 0U);
  EXPECT_EQ(state.last_reject_code, State::REJECT_NONE);
  EXPECT_EQ(state.stop_reason, State::STOP_NONE);
  EXPECT_GT(state.prime_age_ns, 0U);
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    EXPECT_DOUBLE_EQ(state.desired_positions[axis], command_positions_[axis]);
    EXPECT_DOUBLE_EQ(state.desired_velocities[axis], 0.0);
  }
}

TEST_F(StatePublisherTest, StateEndpointUsesTheNamedPrototypeQosCandidate)
{
  const rclcpp::QoS qos = RollingControllerStateTestPeer::stateQos(*controller_);
  EXPECT_EQ(qos.history(), rclcpp::HistoryPolicy::KeepLast);
  EXPECT_EQ(qos.depth(), 1U);
  EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::Volatile);
}

TEST_F(StatePublisherTest, RejectProgressAndStopReasonRemainCoherentAndIndependent)
{
  const Open::Response response = open();
  ASSERT_TRUE(response.success);
  RollingControllerStateTestPeer::submit(*controller_, makeTrajectory(response));
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  for (std::size_t cycle = 0U; cycle < 5U; ++cycle) {
    ASSERT_EQ(update(), controller_interface::return_type::OK);
  }

  Batch invalid = makeTrajectory(response);
  invalid.sequence = 2U;
  invalid.points[0].positions[0] = std::numeric_limits<double>::quiet_NaN();
  RollingControllerStateTestPeer::submit(*controller_, invalid);
  ASSERT_EQ(update(), controller_interface::return_type::OK);

  State running;
  ASSERT_TRUE(RollingControllerStateTestPeer::buildState(*controller_, running));
  EXPECT_EQ(running.session_state, State::SESSION_RUNNING);
  EXPECT_EQ(running.last_reject_code, State::REJECT_NON_FINITE);
  EXPECT_EQ(running.last_rejected_sequence, 2U);
  EXPECT_EQ(running.stop_reason, State::STOP_NONE);
  EXPECT_EQ(running.active_generation, 1U);
  EXPECT_EQ(running.validation_base_generation, 1U);
  EXPECT_FALSE(running.pending_generation_valid);
  EXPECT_EQ(running.last_seen_sequence, 2U);
  EXPECT_EQ(running.last_accepted_sequence, 1U);
  EXPECT_EQ(
    running.available_horizon_ns,
    running.buffered_until_ns - running.execution_time_ns);
  EXPECT_GT(running.accepted_update_age_ns, 0U);

  const Close::Response close = RollingControllerStateTestPeer::close(
    *controller_, makeCloseRequest(response, 60U));
  ASSERT_TRUE(close.success);
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  State stopping;
  ASSERT_TRUE(RollingControllerStateTestPeer::buildState(*controller_, stopping));
  EXPECT_EQ(stopping.last_reject_code, State::REJECT_NON_FINITE);
  EXPECT_EQ(stopping.stop_reason, State::STOP_GRACEFUL_CLOSE);
  EXPECT_TRUE(
    stopping.session_state == State::SESSION_STOPPING ||
    stopping.session_state == State::SESSION_HOLDING);
}

TEST_F(StatePublisherTest, SupersededPendingGenerationIsCountedOnceAndAcknowledged)
{
  const Open::Response response = open();
  ASSERT_TRUE(response.success);
  const Batch first = makeTrajectory(response);
  RollingControllerStateTestPeer::submit(*controller_, first);
  Batch second = first;
  second.sequence = 2U;
  RollingControllerStateTestPeer::submit(*controller_, second);
  ASSERT_EQ(update(), controller_interface::return_type::OK);

  State state;
  ASSERT_TRUE(RollingControllerStateTestPeer::buildState(*controller_, state));
  EXPECT_EQ(state.session_state, State::SESSION_RUNNING);
  EXPECT_EQ(state.active_generation, 2U);
  EXPECT_EQ(state.validation_base_generation, 2U);
  EXPECT_FALSE(state.pending_generation_valid);
  EXPECT_EQ(state.pending_generation, 0U);
  EXPECT_EQ(state.last_seen_sequence, 2U);
  EXPECT_EQ(state.last_accepted_sequence, 2U);
  EXPECT_EQ(state.accepted_count, 2U);
  EXPECT_EQ(state.superseded_pending_count, 1U);
}

TEST_F(StatePublisherTest, UpdateTimeoutCountAndHoldingStateAreObservable)
{
  const Open::Response response = open();
  ASSERT_TRUE(response.success);
  RollingControllerStateTestPeer::submit(*controller_, makeTrajectory(response));
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  RollingControllerStateTestPeer::expireAcceptedUpdate(*controller_);
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  for (std::size_t cycle = 0U; cycle < 100U; ++cycle) {
    State state;
    ASSERT_TRUE(RollingControllerStateTestPeer::buildState(*controller_, state));
    if (state.session_state == State::SESSION_HOLDING) {
      EXPECT_EQ(state.stop_reason, State::STOP_UPDATE_TIMEOUT);
      EXPECT_EQ(state.timeout_count, 1U);
      return;
    }
    ASSERT_EQ(update(), controller_interface::return_type::OK);
  }
  FAIL() << "timeout stop did not reach Holding";
}

TEST_F(StatePublisherTest, TopicPublicationWorksWithoutSubscribersAndDeliversTheSameModel)
{
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  EXPECT_EQ(
    RollingControllerStateTestPeer::stateTopic(*controller_),
    "/rt/rolling_joint_control/state");
  EXPECT_TRUE(RollingControllerStateTestPeer::publishState(*controller_));

  auto observer = std::make_shared<rclcpp::Node>("rolling_state_observer");
  std::shared_ptr<State> received;
  auto subscription = observer->create_subscription<State>(
    "/rt/rolling_joint_control/state", rclcpp::QoS(1).reliable(),
    [&received](State::SharedPtr message) {received = std::move(message);});
  const auto discovery_deadline = std::chrono::steady_clock::now() + 1s;
  while (
    subscription->get_publisher_count() == 0U &&
    std::chrono::steady_clock::now() < discovery_deadline)
  {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_EQ(subscription->get_publisher_count(), 1U);

  ASSERT_TRUE(RollingControllerStateTestPeer::publishState(*controller_));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  const auto receive_deadline = std::chrono::steady_clock::now() + 1s;
  while (!received && std::chrono::steady_clock::now() < receive_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(received);
  State expected;
  ASSERT_TRUE(RollingControllerStateTestPeer::buildState(*controller_, expected));
  EXPECT_EQ(received->controller_boot_id, expected.controller_boot_id);
  EXPECT_EQ(received->session_state, expected.session_state);
  EXPECT_EQ(received->active_generation, expected.active_generation);
  EXPECT_EQ(received->stop_reason, expected.stop_reason);
}

TEST_F(StatePublisherTest, SlowStateConsumerCannotDelayTheRtUpdatePath)
{
  auto observer = std::make_shared<rclcpp::Node>("slow_rolling_state_observer");
  std::atomic_bool callback_entered{false};
  std::atomic_bool release_callback{false};
  auto subscription = observer->create_subscription<State>(
    "/rt/rolling_joint_control/state", rclcpp::QoS(1).reliable(),
    [&callback_entered, &release_callback](State::SharedPtr) {
      callback_entered.store(true, std::memory_order_release);
      while (!release_callback.load(std::memory_order_acquire)) {
        std::this_thread::sleep_for(1ms);
      }
    });

  const auto discovery_deadline = std::chrono::steady_clock::now() + 1s;
  while (
    subscription->get_publisher_count() == 0U &&
    std::chrono::steady_clock::now() < discovery_deadline)
  {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_EQ(subscription->get_publisher_count(), 1U);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2U);
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(observer);
  std::thread executor_thread([&executor]() {executor.spin();});
  const auto callback_deadline = std::chrono::steady_clock::now() + 1s;
  while (
    !callback_entered.load(std::memory_order_acquire) &&
    std::chrono::steady_clock::now() < callback_deadline)
  {
    std::this_thread::sleep_for(1ms);
  }

  std::future<controller_interface::return_type> update_result =
    std::async(std::launch::async, [this]() {return update();});
  const std::future_status update_status = update_result.wait_for(100ms);
  release_callback.store(true, std::memory_order_release);
  executor.cancel();
  executor_thread.join();

  ASSERT_TRUE(callback_entered.load(std::memory_order_acquire));
  ASSERT_EQ(update_status, std::future_status::ready);
  EXPECT_EQ(update_result.get(), controller_interface::return_type::OK);
}

TEST_F(StatePublisherTest, ConcurrentRtProgressProducesOnlyCoherentStateSnapshots)
{
  const Open::Response response = open();
  ASSERT_TRUE(response.success);
  RollingControllerStateTestPeer::submit(*controller_, makeTrajectory(response));
  ASSERT_EQ(update(), controller_interface::return_type::OK);

  std::atomic_bool start{false};
  std::atomic_bool finished{false};
  std::atomic_bool update_failed{false};
  std::thread producer([this, &start, &finished, &update_failed]() {
      while (!start.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      for (std::size_t cycle = 0U; cycle < 100U; ++cycle) {
        if (update() != controller_interface::return_type::OK) {
          update_failed.store(true, std::memory_order_release);
          break;
        }
        std::this_thread::sleep_for(50us);
      }
      finished.store(true, std::memory_order_release);
    });

  std::size_t coherent_snapshots = 0U;
  start.store(true, std::memory_order_release);
  while (!finished.load(std::memory_order_acquire)) {
    State state;
    if (!RollingControllerStateTestPeer::buildState(*controller_, state)) {
      continue;
    }
    ++coherent_snapshots;
    EXPECT_EQ(state.session_state, State::SESSION_RUNNING);
    EXPECT_EQ(state.active_generation, 1U);
    EXPECT_EQ(state.execution_time_ns % kCycleNs, 0U);
    if (state.buffered_until_ns >= state.execution_time_ns) {
      EXPECT_EQ(
        state.available_horizon_ns,
        state.buffered_until_ns - state.execution_time_ns);
    } else {
      ADD_FAILURE() << "buffered horizon preceded the execution cursor";
    }
    const double position_delta =
      state.desired_positions[0] - response.hold_positions[0];
    for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
      EXPECT_TRUE(std::isfinite(state.desired_positions[axis]));
      EXPECT_TRUE(std::isfinite(state.desired_velocities[axis]));
      EXPECT_NEAR(
        state.desired_positions[axis] - response.hold_positions[axis],
        position_delta, 1.0e-12);
      EXPECT_NEAR(state.desired_velocities[axis], state.desired_velocities[0], 1.0e-12);
    }
  }
  producer.join();

  EXPECT_FALSE(update_failed.load(std::memory_order_acquire));
  EXPECT_GT(coherent_snapshots, 10U);
}

TEST_F(StatePublisherTest, ControllerDeactivationPublishesTheTerminatedTransition)
{
  auto observer = std::make_shared<rclcpp::Node>("terminated_state_observer");
  std::shared_ptr<State> received;
  auto subscription = observer->create_subscription<State>(
    "/rt/rolling_joint_control/state", rclcpp::QoS(1).reliable(),
    [&received](State::SharedPtr message) {received = std::move(message);});
  const auto discovery_deadline = std::chrono::steady_clock::now() + 1s;
  while (
    subscription->get_publisher_count() == 0U &&
    std::chrono::steady_clock::now() < discovery_deadline)
  {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_EQ(subscription->get_publisher_count(), 1U);

  ASSERT_EQ(
    controller_->on_deactivate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  const auto receive_deadline = std::chrono::steady_clock::now() + 1s;
  while (!received && std::chrono::steady_clock::now() < receive_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(1ms);
  }

  ASSERT_TRUE(received);
  EXPECT_EQ(received->control_mode, State::MODE_DISABLED);
  EXPECT_EQ(received->session_state, State::SESSION_TERMINATED);
  EXPECT_EQ(received->stop_reason, State::STOP_CONTROLLER_DEACTIVATED);
  EXPECT_FALSE(received->has_session);
  EXPECT_TRUE(received->source_controller_deactivated);
}

}  // namespace
}  // namespace rolling_trajectory_controller
