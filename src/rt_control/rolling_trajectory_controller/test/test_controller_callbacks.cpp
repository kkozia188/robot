#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <future>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/rolling_joint_target_batch.hpp"
#include "robot_interfaces/srv/close_rolling_joint_session.hpp"
#include "robot_interfaces/srv/open_rolling_joint_session.hpp"
#include "rolling_trajectory_controller/rolling_snapshot.hpp"
#include "rolling_trajectory_controller/rolling_trajectory_controller.hpp"
#include "rolling_trajectory_controller/rolling_types.hpp"

namespace rolling_trajectory_controller
{

class RollingControllerTestPeer
{
public:
  static Identifier bootId(RollingTrajectoryController & controller)
  {
    std::lock_guard<std::mutex> lock(controller.callback_mutex_);
    return controller.controller_boot_id_;
  }

  static SessionState sessionState(RollingTrajectoryController & controller)
  {
    std::lock_guard<std::mutex> lock(controller.callback_mutex_);
    return controller.buffer_.sessionState();
  }

  static std::uint64_t lastSeenSequence(RollingTrajectoryController & controller)
  {
    std::lock_guard<std::mutex> lock(controller.callback_mutex_);
    return controller.buffer_.lastSeenSequence();
  }

  static RejectCode lastReject(RollingTrajectoryController & controller)
  {
    std::lock_guard<std::mutex> lock(controller.callback_mutex_);
    return controller.last_reject_code_;
  }

  static bool latestSnapshot(
    RollingTrajectoryController & controller, RollingSnapshot & snapshot)
  {
    SnapshotLease lease;
    if (!controller.snapshot_exchange_.acquire(lease)) {
      return false;
    }
    snapshot = *lease;
    return true;
  }

  static std::uint64_t publicationSequence(RollingTrajectoryController & controller)
  {
    return controller.snapshot_exchange_.latestPublicationSequence();
  }

  static bool markHolding(RollingTrajectoryController & controller)
  {
    std::lock_guard<std::mutex> lock(controller.callback_mutex_);
    return controller.buffer_.markHolding();
  }
};

namespace
{

using namespace std::chrono_literals;
using Batch = robot_interfaces::msg::RollingJointTargetBatch;
using Close = robot_interfaces::srv::CloseRollingJointSession;
using Open = robot_interfaces::srv::OpenRollingJointSession;

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

Open::Request makeOpenRequest(
  const Identifier & boot_id, std::uint8_t request_seed = 20U,
  std::uint8_t client_seed = 40U)
{
  Open::Request request;
  request.protocol_major = Open::Request::PROTOCOL_MAJOR;
  request.protocol_minor = Open::Request::PROTOCOL_MINOR;
  request.request_id = makeUuid(request_seed);
  request.expected_controller_boot_id.uuid = boot_id;
  request.client_instance_id = makeUuid(client_seed);
  request.axis_set_hash = kAxisSetHash;
  return request;
}

Close::Request makeCloseRequest(
  const Open::Response & open, std::uint8_t request_seed)
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

Batch makePrime(const Open::Response & open, std::uint64_t sequence)
{
  Batch batch;
  batch.protocol_major = Batch::PROTOCOL_MAJOR;
  batch.protocol_minor = Batch::PROTOCOL_MINOR;
  batch.controller_boot_id = open.controller_boot_id;
  batch.session_id = open.session_id;
  batch.client_instance_id = open.client_instance_id;
  batch.sequence = sequence;
  batch.replace_from_ns = 0U;
  batch.points.resize(2U);
  batch.points[0].time_from_session_start_ns = 0U;
  batch.points[1].time_from_session_start_ns = 8'000'000U;
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    batch.points[0].positions[axis] = open.hold_positions[axis];
    batch.points[1].positions[axis] = open.hold_positions[axis];
    batch.points[0].velocities[axis] = 0.0;
    batch.points[1].velocities[axis] = 0.0;
  }
  return batch;
}

class ControllerCallbacksTest : public ::testing::Test
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
      controller_->init("test_rolling_callbacks", "", options),
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

    client_node_ = std::make_shared<rclcpp::Node>("rolling_callback_test_client");
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 4U);
    executor_->add_node(controller_->get_node()->get_node_base_interface());
    executor_->add_node(client_node_);
    executor_thread_ = std::thread([this]() {executor_->spin();});

    open_client_ = client_node_->create_client<Open>("/rt/rolling_joint_control/open");
    close_client_ = client_node_->create_client<Close>("/rt/rolling_joint_control/close");
    update_publisher_ = client_node_->create_publisher<Batch>(
      "/rt/rolling_joint_control/update", rclcpp::QoS(1).reliable());
    ASSERT_TRUE(open_client_->wait_for_service(1s));
    ASSERT_TRUE(close_client_->wait_for_service(1s));
    ASSERT_TRUE(waitUntil([this]() {return update_publisher_->get_subscription_count() == 1U;}));
  }

  void TearDown() override
  {
    if (executor_) {
      executor_->cancel();
    }
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
    if (controller_) {
      (void)controller_->on_deactivate(rclcpp_lifecycle::State());
    }
    executor_.reset();
    client_node_.reset();
    controller_.reset();
  }

  bool waitUntil(const std::function<bool()> & predicate) const
  {
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (std::chrono::steady_clock::now() < deadline) {
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return predicate();
  }

  Open::Response callOpen(const Open::Request & request)
  {
    auto future = open_client_->async_send_request(std::make_shared<Open::Request>(request));
    if (future.wait_for(1s) != std::future_status::ready) {
      ADD_FAILURE() << "open service timed out";
      return Open::Response{};
    }
    return *future.get();
  }

  Close::Response callClose(const Close::Request & request)
  {
    auto future =
      close_client_->async_send_request(std::make_shared<Close::Request>(request));
    if (future.wait_for(1s) != std::future_status::ready) {
      ADD_FAILURE() << "close service timed out";
      return Close::Response{};
    }
    return *future.get();
  }

  void refreshAdmissionSnapshot()
  {
    ASSERT_EQ(
      controller_->update(
        rclcpp::Time(0), rclcpp::Duration::from_nanoseconds(4'000'000)),
      controller_interface::return_type::OK);
  }

  std::array<double, kAxisCount> command_positions_{};
  std::array<double, kAxisCount> actual_positions_{};
  double feedback_age_ms_{0.0};
  std::vector<hardware_interface::CommandInterface> command_handles_{};
  std::vector<hardware_interface::StateInterface> state_handles_{};
  std::unique_ptr<RollingTrajectoryController> controller_{};
  std::shared_ptr<rclcpp::Node> client_node_{};
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_{};
  std::thread executor_thread_{};
  rclcpp::Client<Open>::SharedPtr open_client_{};
  rclcpp::Client<Close>::SharedPtr close_client_{};
  rclcpp::Publisher<Batch>::SharedPtr update_publisher_{};
};

TEST_F(ControllerCallbacksTest, OpenAdmissionAndRetryAreDeterministic)
{
  const Identifier boot = RollingControllerTestPeer::bootId(*controller_);
  Open::Request request = makeOpenRequest(boot);

  request.protocol_major += 1U;
  EXPECT_EQ(callOpen(request).error_code, Open::Response::ERROR_WRONG_PROTOCOL);
  request.protocol_major = Open::Request::PROTOCOL_MAJOR;

  request.expected_controller_boot_id = makeUuid(99U);
  EXPECT_EQ(callOpen(request).error_code, Open::Response::ERROR_WRONG_BOOT);
  request.expected_controller_boot_id.uuid = boot;

  request.axis_set_hash[0] ^= 0xffU;
  EXPECT_EQ(callOpen(request).error_code, Open::Response::ERROR_AXIS_SET_MISMATCH);
  request.axis_set_hash = kAxisSetHash;

  feedback_age_ms_ = 500.001;
  refreshAdmissionSnapshot();
  EXPECT_EQ(callOpen(request).error_code, Open::Response::ERROR_FEEDBACK_STALE);
  feedback_age_ms_ = 12.0;
  refreshAdmissionSnapshot();

  const Open::Response accepted = callOpen(request);
  ASSERT_TRUE(accepted.success);
  EXPECT_EQ(accepted.error_code, Open::Response::ERROR_NONE);
  EXPECT_EQ(accepted.session_state, Open::Response::SESSION_PRIMING);
  EXPECT_EQ(accepted.controller_boot_id.uuid, boot);
  EXPECT_EQ(accepted.client_instance_id, request.client_instance_id);
  EXPECT_EQ(accepted.axis_set_hash, kAxisSetHash);
  EXPECT_EQ(accepted.buffer_capacity, 64U);
  EXPECT_TRUE(accepted.test_only_limits);
  EXPECT_TRUE(
    std::any_of(
      accepted.limits_version.begin(), accepted.limits_version.end(),
      [](std::uint8_t value) {return value != 0U;}));
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    EXPECT_DOUBLE_EQ(accepted.hold_positions[axis], command_positions_[axis]);
    EXPECT_DOUBLE_EQ(accepted.hold_velocities[axis], 0.0);
  }

  const Open::Response retried = callOpen(request);
  EXPECT_TRUE(retried.success);
  EXPECT_EQ(retried.session_id, accepted.session_id);

  Open::Request competing = request;
  competing.request_id = makeUuid(21U);
  EXPECT_EQ(callOpen(competing).error_code, Open::Response::ERROR_SESSION_BUSY);
  EXPECT_EQ(RollingControllerTestPeer::sessionState(*controller_), SessionState::kPriming);
}

TEST_F(ControllerCallbacksTest, UpdateMappingPublishesOnlyCompleteAcceptedGenerations)
{
  const Open::Response open = callOpen(
    makeOpenRequest(RollingControllerTestPeer::bootId(*controller_)));
  ASSERT_TRUE(open.success);

  Batch batch = makePrime(open, 1U);
  update_publisher_->publish(batch);
  ASSERT_TRUE(
    waitUntil(
      [this]() {return RollingControllerTestPeer::lastSeenSequence(*controller_) == 1U;}));
  ASSERT_EQ(RollingControllerTestPeer::lastReject(*controller_), RejectCode::kNone);
  RollingSnapshot first;
  ASSERT_TRUE(RollingControllerTestPeer::latestSnapshot(*controller_, first));
  EXPECT_EQ(first.identity.controller_boot_id, open.controller_boot_id.uuid);
  EXPECT_EQ(first.identity.session_id, open.session_id.uuid);
  EXPECT_EQ(first.identity.client_instance_id, open.client_instance_id.uuid);
  EXPECT_EQ(first.trajectory.generation, 1U);
  EXPECT_EQ(first.trajectory.point_count, 2U);
  const std::uint64_t publication = first.publication_sequence;

  Batch wrong_boot = batch;
  wrong_boot.sequence = 2U;
  wrong_boot.controller_boot_id = makeUuid(90U);
  update_publisher_->publish(wrong_boot);
  ASSERT_TRUE(
    waitUntil(
      [this]() {
        return RollingControllerTestPeer::lastReject(*controller_) ==
        RejectCode::kWrongBoot;
      }));
  EXPECT_EQ(RollingControllerTestPeer::lastSeenSequence(*controller_), 1U);
  EXPECT_EQ(RollingControllerTestPeer::publicationSequence(*controller_), publication);

  batch.sequence = 2U;
  batch.points[1].positions[4] = std::numeric_limits<double>::quiet_NaN();
  update_publisher_->publish(batch);
  ASSERT_TRUE(
    waitUntil(
      [this]() {return RollingControllerTestPeer::lastSeenSequence(*controller_) == 2U;}));
  EXPECT_EQ(RollingControllerTestPeer::lastReject(*controller_), RejectCode::kNonFinite);
  EXPECT_EQ(RollingControllerTestPeer::publicationSequence(*controller_), publication);

  batch.points[1].positions[4] = open.hold_positions[4];
  update_publisher_->publish(batch);
  ASSERT_TRUE(
    waitUntil(
      [this]() {
        return RollingControllerTestPeer::lastReject(*controller_) ==
        RejectCode::kStaleSequence;
      }));
  EXPECT_EQ(RollingControllerTestPeer::publicationSequence(*controller_), publication);

  batch.sequence = 3U;
  update_publisher_->publish(batch);
  ASSERT_TRUE(
    waitUntil(
      [this, publication]() {
        return RollingControllerTestPeer::publicationSequence(*controller_) == publication + 1U;
      }));
  RollingSnapshot second;
  ASSERT_TRUE(RollingControllerTestPeer::latestSnapshot(*controller_, second));
  EXPECT_EQ(second.trajectory.generation, 2U);
  EXPECT_EQ(second.identity.session_id, open.session_id.uuid);
}

TEST_F(ControllerCallbacksTest, CloseCacheIsBoundedAndFinalizeCannotAffectANewerSession)
{
  const Identifier boot = RollingControllerTestPeer::bootId(*controller_);
  const Open::Request open_request = makeOpenRequest(boot);
  const Open::Response open = callOpen(open_request);
  ASSERT_TRUE(open.success);

  const Close::Request original = makeCloseRequest(open, 60U);
  const Close::Response first = callClose(original);
  ASSERT_TRUE(first.success);
  EXPECT_TRUE(first.stop_accepted);
  EXPECT_FALSE(first.session_destroyed);
  EXPECT_EQ(first.session_state, Close::Response::SESSION_STOPPING);
  EXPECT_EQ(RollingControllerTestPeer::sessionState(*controller_), SessionState::kStopping);

  for (std::uint8_t seed = 61U; seed < 68U; ++seed) {
    const Close::Response response = callClose(makeCloseRequest(open, seed));
    EXPECT_TRUE(response.success) << "seed=" << static_cast<unsigned int>(seed);
    EXPECT_TRUE(response.stop_accepted);
  }
  EXPECT_EQ(
    callClose(makeCloseRequest(open, 68U)).error_code,
    Close::Response::ERROR_WRONG_REQUEST);

  ASSERT_TRUE(RollingControllerTestPeer::markHolding(*controller_));
  const Close::Response original_retry = callClose(original);
  EXPECT_TRUE(original_retry.success);
  EXPECT_EQ(original_retry.session_state, Close::Response::SESSION_STOPPING);
  EXPECT_FALSE(original_retry.session_destroyed);
  EXPECT_EQ(RollingControllerTestPeer::sessionState(*controller_), SessionState::kHolding);

  const Close::Request finalize = makeCloseRequest(open, 90U);
  const Close::Response finalized = callClose(finalize);
  ASSERT_TRUE(finalized.success);
  EXPECT_TRUE(finalized.session_destroyed);
  EXPECT_EQ(finalized.session_state, Close::Response::SESSION_NONE);
  EXPECT_EQ(RollingControllerTestPeer::sessionState(*controller_), SessionState::kNone);

  Open::Request next_open_request = open_request;
  next_open_request.request_id = makeUuid(100U);
  const Open::Response next_open = callOpen(next_open_request);
  ASSERT_TRUE(next_open.success);
  ASSERT_NE(next_open.session_id, open.session_id);
  const Close::Response finalize_retry = callClose(finalize);
  EXPECT_TRUE(finalize_retry.success);
  EXPECT_TRUE(finalize_retry.session_destroyed);
  EXPECT_EQ(RollingControllerTestPeer::sessionState(*controller_), SessionState::kPriming);
}

TEST_F(ControllerCallbacksTest, ConcurrentOpenRequestsCreateExactlyOneSession)
{
  constexpr std::size_t kRequestCount = 8U;
  const Identifier boot = RollingControllerTestPeer::bootId(*controller_);
  std::array<rclcpp::Client<Open>::SharedFuture, kRequestCount> futures;
  for (std::size_t index = 0U; index < kRequestCount; ++index) {
    const auto request = std::make_shared<Open::Request>(
      makeOpenRequest(
        boot, static_cast<std::uint8_t>(110U + index),
        static_cast<std::uint8_t>(140U + index)));
    futures[index] = open_client_->async_send_request(request).future.share();
  }

  std::size_t success_count = 0U;
  std::size_t busy_count = 0U;
  for (auto & future : futures) {
    ASSERT_EQ(future.wait_for(3s), std::future_status::ready);
    const auto response = future.get();
    if (response->success) {
      ++success_count;
    } else if (response->error_code == Open::Response::ERROR_SESSION_BUSY) {
      ++busy_count;
    }
  }
  EXPECT_EQ(success_count, 1U);
  EXPECT_EQ(busy_count, kRequestCount - 1U);
  EXPECT_EQ(RollingControllerTestPeer::sessionState(*controller_), SessionState::kPriming);
}

TEST_F(ControllerCallbacksTest, DeactivateInvalidatesOldBootAndSession)
{
  const Identifier old_boot = RollingControllerTestPeer::bootId(*controller_);
  const Open::Response old_open = callOpen(makeOpenRequest(old_boot));
  ASSERT_TRUE(old_open.success);

  ASSERT_EQ(
    controller_->on_deactivate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(RollingControllerTestPeer::sessionState(*controller_), SessionState::kTerminated);
  ASSERT_EQ(
    controller_->on_activate(rclcpp_lifecycle::State()),
    controller_interface::CallbackReturn::SUCCESS);
  const Identifier new_boot = RollingControllerTestPeer::bootId(*controller_);
  EXPECT_NE(new_boot, old_boot);

  EXPECT_EQ(
    callOpen(makeOpenRequest(old_boot, 120U)).error_code,
    Open::Response::ERROR_WRONG_BOOT);
  EXPECT_TRUE(callOpen(makeOpenRequest(new_boot, 121U)).success);
}

}  // namespace
}  // namespace rolling_trajectory_controller
