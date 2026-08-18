#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/rolling_joint_target_batch.hpp"
#include "robot_interfaces/srv/close_rolling_joint_session.hpp"
#include "robot_interfaces/srv/open_rolling_joint_session.hpp"
#include "rolling_trajectory_controller/cubic_hermite.hpp"
#include "rolling_trajectory_controller/rolling_trajectory_controller.hpp"
#include "rolling_trajectory_controller/rolling_types.hpp"

namespace
{

std::atomic_bool track_allocations{false};
std::atomic<std::uint64_t> tracked_allocation_count{0U};

void countAllocation() noexcept
{
  if (track_allocations.load(std::memory_order_relaxed)) {
    tracked_allocation_count.fetch_add(1U, std::memory_order_relaxed);
  }
}

void * allocateUnaligned(std::size_t size) noexcept
{
  return std::malloc(size == 0U ? 1U : size);
}

void * allocateAligned(std::size_t size, std::size_t alignment) noexcept
{
  void * memory = nullptr;
  const std::size_t allocation_size = size == 0U ? 1U : size;
  return posix_memalign(&memory, alignment, allocation_size) == 0 ? memory : nullptr;
}

}  // namespace

void * operator new(std::size_t size)
{
  countAllocation();
  if (void * memory = allocateUnaligned(size)) {
    return memory;
  }
  throw std::bad_alloc();
}

void * operator new[](std::size_t size)
{
  countAllocation();
  if (void * memory = allocateUnaligned(size)) {
    return memory;
  }
  throw std::bad_alloc();
}

void operator delete(void * memory) noexcept
{
  std::free(memory);
}

void operator delete[](void * memory) noexcept
{
  std::free(memory);
}

void operator delete(void * memory, std::size_t) noexcept
{
  std::free(memory);
}

void operator delete[](void * memory, std::size_t) noexcept
{
  std::free(memory);
}

void * operator new(std::size_t size, const std::nothrow_t &) noexcept
{
  countAllocation();
  return allocateUnaligned(size);
}

void * operator new[](std::size_t size, const std::nothrow_t &) noexcept
{
  countAllocation();
  return allocateUnaligned(size);
}

void operator delete(void * memory, const std::nothrow_t &) noexcept
{
  std::free(memory);
}

void operator delete[](void * memory, const std::nothrow_t &) noexcept
{
  std::free(memory);
}

void * operator new(std::size_t size, std::align_val_t alignment)
{
  countAllocation();
  if (void * memory = allocateAligned(size, static_cast<std::size_t>(alignment))) {
    return memory;
  }
  throw std::bad_alloc();
}

void * operator new[](std::size_t size, std::align_val_t alignment)
{
  countAllocation();
  if (void * memory = allocateAligned(size, static_cast<std::size_t>(alignment))) {
    return memory;
  }
  throw std::bad_alloc();
}

void operator delete(void * memory, std::align_val_t) noexcept
{
  std::free(memory);
}

void operator delete[](void * memory, std::align_val_t) noexcept
{
  std::free(memory);
}

void operator delete(void * memory, std::size_t, std::align_val_t) noexcept
{
  std::free(memory);
}

void operator delete[](void * memory, std::size_t, std::align_val_t) noexcept
{
  std::free(memory);
}

void * operator new(
  std::size_t size, std::align_val_t alignment, const std::nothrow_t &) noexcept
{
  countAllocation();
  return allocateAligned(size, static_cast<std::size_t>(alignment));
}

void * operator new[](
  std::size_t size, std::align_val_t alignment, const std::nothrow_t &) noexcept
{
  countAllocation();
  return allocateAligned(size, static_cast<std::size_t>(alignment));
}

void operator delete(
  void * memory, std::align_val_t, const std::nothrow_t &) noexcept
{
  std::free(memory);
}

void operator delete[](
  void * memory, std::align_val_t, const std::nothrow_t &) noexcept
{
  std::free(memory);
}

namespace rolling_trajectory_controller
{

class RollingControllerRtTestPeer
{
public:
  using Batch = robot_interfaces::msg::RollingJointTargetBatch;
  using Close = robot_interfaces::srv::CloseRollingJointSession;
  using Open = robot_interfaces::srv::OpenRollingJointSession;

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

  static SessionState sessionState(const RollingTrajectoryController & controller)
  {
    return static_cast<SessionState>(
      controller.rt_session_state_.load(std::memory_order_acquire));
  }

  static StopReason stopReason(const RollingTrajectoryController & controller)
  {
    return static_cast<StopReason>(controller.stop_reason_.load(std::memory_order_acquire));
  }

  static std::uint64_t executionTime(const RollingTrajectoryController & controller)
  {
    return controller.execution_time_ns_.load(std::memory_order_acquire);
  }

  static std::uint64_t activeGeneration(const RollingTrajectoryController & controller)
  {
    return controller.active_generation_.load(std::memory_order_acquire);
  }

  static std::uint64_t replaceableFrom(const RollingTrajectoryController & controller)
  {
    return controller.replaceable_from_ns_.load(std::memory_order_acquire);
  }

  static RejectCode lastReject(RollingTrajectoryController & controller)
  {
    std::lock_guard<std::mutex> lock(controller.callback_mutex_);
    return controller.last_reject_code_;
  }

  static std::uint8_t invariantStage(const RollingTrajectoryController & controller)
  {
    return controller.rt_invariant_stage_.load(std::memory_order_acquire);
  }

  static void expirePrime(RollingTrajectoryController & controller)
  {
    controller.prime_start_time_ns_.store(0U, std::memory_order_release);
  }

  static void expireAcceptedUpdate(RollingTrajectoryController & controller)
  {
    controller.rt_last_accepted_arrival_ns_ = 0U;
  }
};

namespace
{

using Batch = robot_interfaces::msg::RollingJointTargetBatch;
using Close = robot_interfaces::srv::CloseRollingJointSession;
using Open = robot_interfaces::srv::OpenRollingJointSession;

constexpr std::uint64_t kCycleNs = 4'000'000U;
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

Open::Request makeOpenRequest(const Identifier & boot_id, std::uint8_t request_seed = 20U)
{
  Open::Request request;
  request.protocol_major = Open::Request::PROTOCOL_MAJOR;
  request.protocol_minor = Open::Request::PROTOCOL_MINOR;
  request.request_id = makeUuid(request_seed);
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

Batch makeTrajectory(
  const Open::Response & open, std::uint64_t duration_ns, double displacement)
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
  batch.points[1].time_from_session_start_ns = duration_ns;
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    batch.points[0].positions[axis] = open.hold_positions[axis];
    batch.points[1].positions[axis] = open.hold_positions[axis] + displacement;
    batch.points[0].velocities[axis] = 0.0;
    batch.points[1].velocities[axis] = 0.0;
  }
  return batch;
}

Batch makeReplacement(
  const Open::Response & open, std::uint64_t sequence,
  std::uint64_t replace_from_ns, std::uint64_t duration_ns,
  double original_displacement, double replacement_displacement)
{
  CubicHermite original;
  EXPECT_TRUE(
    buildCubicHermite(
      0.0, 0.0, original_displacement, 0.0,
      static_cast<double>(duration_ns) * 1.0e-9, original));
  ScalarKinematicState splice;
  EXPECT_TRUE(
    sampleCubicHermite(
      original,
      static_cast<double>(replace_from_ns) / static_cast<double>(duration_ns),
      splice));

  Batch batch;
  batch.protocol_major = Batch::PROTOCOL_MAJOR;
  batch.protocol_minor = Batch::PROTOCOL_MINOR;
  batch.controller_boot_id = open.controller_boot_id;
  batch.session_id = open.session_id;
  batch.client_instance_id = open.client_instance_id;
  batch.sequence = sequence;
  batch.replace_from_ns = replace_from_ns;
  batch.points.resize(2U);
  batch.points[0].time_from_session_start_ns = replace_from_ns;
  batch.points[1].time_from_session_start_ns = duration_ns;
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    batch.points[0].positions[axis] = open.hold_positions[axis] + splice.position;
    batch.points[0].velocities[axis] = splice.velocity;
    batch.points[1].positions[axis] =
      open.hold_positions[axis] + replacement_displacement;
    batch.points[1].velocities[axis] = 0.0;
  }
  return batch;
}

class RtUpdateTest : public ::testing::Test
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
      controller_->init("test_rt_update_controller", "", options),
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
  }

  void TearDown() override
  {
    track_allocations.store(false, std::memory_order_relaxed);
    if (controller_) {
      (void)controller_->on_deactivate(rclcpp_lifecycle::State());
    }
  }

  Open::Response open(std::uint8_t request_seed = 20U)
  {
    return RollingControllerRtTestPeer::open(
      *controller_,
      makeOpenRequest(RollingControllerRtTestPeer::bootId(*controller_), request_seed));
  }

  controller_interface::return_type update(
    std::int64_t period_ns =
    static_cast<std::int64_t>(kCycleNs))
  {
    return controller_->update(
      rclcpp::Time(0), rclcpp::Duration::from_nanoseconds(period_ns));
  }

  std::array<double, kAxisCount> command_positions_{};
  std::array<double, kAxisCount> actual_positions_{};
  double feedback_age_ms_{0.0};
  std::vector<hardware_interface::CommandInterface> command_handles_{};
  std::vector<hardware_interface::StateInterface> state_handles_{};
  std::unique_ptr<RollingTrajectoryController> controller_{};
};

TEST_F(RtUpdateTest, PrimeActivatesAtTimeZeroThenSamplesTheCubic)
{
  const std::array<double, kAxisCount> initial_commands = command_positions_;
  const Open::Response response = open();
  ASSERT_TRUE(response.success);
  RollingControllerRtTestPeer::submit(
    *controller_, makeTrajectory(response, 200'000'000U, 0.02));

  ASSERT_EQ(update(), controller_interface::return_type::OK);
  EXPECT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kRunning)
    << "invariant_stage=" << static_cast<unsigned int>(
    RollingControllerRtTestPeer::invariantStage(*controller_));
  EXPECT_EQ(RollingControllerRtTestPeer::activeGeneration(*controller_), 1U);
  EXPECT_EQ(RollingControllerRtTestPeer::executionTime(*controller_), 0U);
  EXPECT_EQ(command_positions_, initial_commands);

  ASSERT_EQ(update(), controller_interface::return_type::OK);
  EXPECT_EQ(RollingControllerRtTestPeer::executionTime(*controller_), kCycleNs);
  CubicHermite segment;
  ASSERT_TRUE(buildCubicHermite(0.0, 0.0, 0.02, 0.0, 0.2, segment));
  ScalarKinematicState expected;
  ASSERT_TRUE(sampleCubicHermite(segment, 0.02, expected));
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    EXPECT_NEAR(
      command_positions_[axis], initial_commands[axis] + expected.position,
      1.0e-15);
    EXPECT_TRUE(std::isfinite(command_positions_[axis]));
  }
}

TEST_F(RtUpdateTest, RunningSuffixReplacementSwitchesGenerationWithoutRestartingTime)
{
  constexpr std::uint64_t duration_ns = 500'000'000U;
  constexpr std::uint64_t replace_from_ns = 20'000'000U;
  constexpr double original_displacement = 0.1;
  constexpr double replacement_displacement = 0.08;
  const Open::Response response = open();
  ASSERT_TRUE(response.success);
  RollingControllerRtTestPeer::submit(
    *controller_, makeTrajectory(response, duration_ns, original_displacement));
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  ASSERT_EQ(RollingControllerRtTestPeer::executionTime(*controller_), kCycleNs);

  RollingControllerRtTestPeer::submit(
    *controller_,
    makeReplacement(
      response, 2U, replace_from_ns, duration_ns,
      original_displacement, replacement_displacement));
  EXPECT_EQ(RollingControllerRtTestPeer::activeGeneration(*controller_), 1U);
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  EXPECT_EQ(RollingControllerRtTestPeer::activeGeneration(*controller_), 2U);
  EXPECT_EQ(RollingControllerRtTestPeer::executionTime(*controller_), 2U * kCycleNs);
  EXPECT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kRunning)
    << "stop_reason=" << static_cast<unsigned int>(
    RollingControllerRtTestPeer::stopReason(*controller_)) <<
    ", invariant_stage=" << static_cast<unsigned int>(
    RollingControllerRtTestPeer::invariantStage(*controller_));
  EXPECT_EQ(RollingControllerRtTestPeer::stopReason(*controller_), StopReason::kNone);

  for (std::size_t cycle = 0U; cycle < 4U; ++cycle) {
    ASSERT_EQ(update(), controller_interface::return_type::OK);
  }
  ASSERT_EQ(RollingControllerRtTestPeer::executionTime(*controller_), 24'000'000U);

  CubicHermite original;
  ASSERT_TRUE(
    buildCubicHermite(
      0.0, 0.0, original_displacement, 0.0, 0.5, original));
  ScalarKinematicState splice;
  ASSERT_TRUE(sampleCubicHermite(original, 0.04, splice));
  CubicHermite replacement;
  ASSERT_TRUE(
    buildCubicHermite(
      splice.position, splice.velocity, replacement_displacement, 0.0,
      0.48, replacement));
  ScalarKinematicState expected;
  ASSERT_TRUE(sampleCubicHermite(replacement, 4.0 / 480.0, expected));
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    EXPECT_NEAR(
      command_positions_[axis], response.hold_positions[axis] + expected.position,
      1.0e-15);
    EXPECT_TRUE(std::isfinite(command_positions_[axis]));
  }
}

TEST_F(RtUpdateTest, CloseBeforePrimeStopsAtHoldAndCanBeFinalized)
{
  const Open::Response response = open();
  ASSERT_TRUE(response.success);
  const Close::Response stopping = RollingControllerRtTestPeer::close(
    *controller_, makeCloseRequest(response, 60U));
  ASSERT_TRUE(stopping.success);
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  EXPECT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kHolding);
  EXPECT_EQ(
    RollingControllerRtTestPeer::stopReason(*controller_), StopReason::kGracefulClose);

  const Close::Response finalized = RollingControllerRtTestPeer::close(
    *controller_, makeCloseRequest(response, 61U));
  EXPECT_TRUE(finalized.success);
  EXPECT_TRUE(finalized.session_destroyed);
}

TEST_F(RtUpdateTest, LowWaterEqualityStopsBeforeTheShortBufferCanExhaust)
{
  const Open::Response response = open();
  ASSERT_TRUE(response.success);
  RollingControllerRtTestPeer::submit(
    *controller_, makeTrajectory(response, 20'000'000U, 0.0));
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  EXPECT_EQ(RollingControllerRtTestPeer::executionTime(*controller_), kCycleNs);

  ASSERT_EQ(update(), controller_interface::return_type::OK);
  EXPECT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kHolding);
  EXPECT_EQ(RollingControllerRtTestPeer::stopReason(*controller_), StopReason::kLowWater);
  EXPECT_EQ(RollingControllerRtTestPeer::executionTime(*controller_), kCycleNs);
}

TEST_F(RtUpdateTest, ClockAnomalyUsesNominalStepsInsteadOfTheBadPeriod)
{
  const Open::Response response = open();
  ASSERT_TRUE(response.success);
  RollingControllerRtTestPeer::submit(
    *controller_, makeTrajectory(response, 500'000'000U, 0.1));
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  for (std::size_t cycle = 0U; cycle < 10U; ++cycle) {
    ASSERT_EQ(update(), controller_interface::return_type::OK);
  }
  const std::uint64_t execution_before =
    RollingControllerRtTestPeer::executionTime(*controller_);
  const auto command_before = command_positions_;

  ASSERT_EQ(update(9'000'000), controller_interface::return_type::OK);
  EXPECT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kStopping);
  EXPECT_EQ(
    RollingControllerRtTestPeer::stopReason(*controller_), StopReason::kClockAnomaly);
  EXPECT_EQ(command_positions_, command_before);

  ASSERT_EQ(update(100'000'000), controller_interface::return_type::OK);
  EXPECT_EQ(
    RollingControllerRtTestPeer::executionTime(*controller_),
    execution_before + kCycleNs);
  for (double command : command_positions_) {
    EXPECT_TRUE(std::isfinite(command));
  }
}

TEST_F(RtUpdateTest, NonPositivePeriodsLatchTheSameClockAnomaly)
{
  constexpr std::array<std::int64_t, 2> invalid_periods = {0, -1};
  std::uint8_t open_seed = 70U;
  std::uint8_t close_seed = 170U;
  for (const std::int64_t invalid_period : invalid_periods) {
    const Open::Response response = open(open_seed++);
    ASSERT_TRUE(response.success);
    RollingControllerRtTestPeer::submit(
      *controller_, makeTrajectory(response, 500'000'000U, 0.1));
    ASSERT_EQ(update(), controller_interface::return_type::OK);
    ASSERT_EQ(update(), controller_interface::return_type::OK);

    ASSERT_EQ(update(invalid_period), controller_interface::return_type::OK);
    EXPECT_EQ(
      RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kStopping);
    EXPECT_EQ(
      RollingControllerRtTestPeer::stopReason(*controller_), StopReason::kClockAnomaly);
    for (std::size_t cycle = 0U;
      cycle < 100U &&
      RollingControllerRtTestPeer::sessionState(*controller_) == SessionState::kStopping;
      ++cycle)
    {
      ASSERT_EQ(update(), controller_interface::return_type::OK);
    }
    ASSERT_EQ(
      RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kHolding);
    const Close::Response finalized = RollingControllerRtTestPeer::close(
      *controller_, makeCloseRequest(response, close_seed++));
    ASSERT_TRUE(finalized.success);
    ASSERT_TRUE(finalized.session_destroyed);
  }
}

TEST_F(RtUpdateTest, MovingGracefulCloseStopsContinuouslyAndRejectsLaterUpdates)
{
  const Open::Response response = open();
  ASSERT_TRUE(response.success);
  RollingControllerRtTestPeer::submit(
    *controller_, makeTrajectory(response, 500'000'000U, 0.1));
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  for (std::size_t cycle = 0U; cycle < 10U; ++cycle) {
    ASSERT_EQ(update(), controller_interface::return_type::OK);
  }
  const auto command_before_stop = command_positions_;
  const std::uint64_t execution_before_stop =
    RollingControllerRtTestPeer::executionTime(*controller_);

  const Close::Response stopping = RollingControllerRtTestPeer::close(
    *controller_, makeCloseRequest(response, 180U));
  ASSERT_TRUE(stopping.success);
  ASSERT_TRUE(stopping.stop_accepted);
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  EXPECT_EQ(command_positions_, command_before_stop);
  EXPECT_EQ(
    RollingControllerRtTestPeer::executionTime(*controller_), execution_before_stop);
  EXPECT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kStopping);
  EXPECT_EQ(
    RollingControllerRtTestPeer::stopReason(*controller_), StopReason::kGracefulClose);

  Batch rejected = makeTrajectory(response, 500'000'000U, 0.2);
  rejected.sequence = 2U;
  RollingControllerRtTestPeer::submit(*controller_, rejected);
  EXPECT_EQ(
    RollingControllerRtTestPeer::lastReject(*controller_),
    RejectCode::kSessionNotAccepting);

  for (std::size_t cycle = 0U;
    cycle < 100U &&
    RollingControllerRtTestPeer::sessionState(*controller_) == SessionState::kStopping;
    ++cycle)
  {
    ASSERT_EQ(update(), controller_interface::return_type::OK);
    for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
      EXPECT_TRUE(std::isfinite(command_positions_[axis]));
      EXPECT_GE(command_positions_[axis], command_before_stop[axis]);
    }
  }
  ASSERT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kHolding);
  const Close::Response finalized = RollingControllerRtTestPeer::close(
    *controller_, makeCloseRequest(response, 181U));
  EXPECT_TRUE(finalized.success);
  EXPECT_TRUE(finalized.session_destroyed);
}

TEST_F(RtUpdateTest, UpdateTimeoutStopsEvenWithAPlentifulOldFuture)
{
  const Open::Response response = open();
  ASSERT_TRUE(response.success);
  RollingControllerRtTestPeer::submit(
    *controller_, makeTrajectory(response, 500'000'000U, 0.1));
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  RollingControllerRtTestPeer::expireAcceptedUpdate(*controller_);

  ASSERT_EQ(update(), controller_interface::return_type::OK);
  EXPECT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kStopping);
  EXPECT_EQ(
    RollingControllerRtTestPeer::stopReason(*controller_), StopReason::kUpdateTimeout);
}

TEST_F(RtUpdateTest, PrimeTimeoutStopsAtTheOriginalHold)
{
  const auto initial_commands = command_positions_;
  ASSERT_TRUE(open().success);
  RollingControllerRtTestPeer::expirePrime(*controller_);
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  EXPECT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kHolding);
  EXPECT_EQ(
    RollingControllerRtTestPeer::stopReason(*controller_), StopReason::kPrimeTimeout);
  EXPECT_EQ(command_positions_, initial_commands);
}

TEST_F(RtUpdateTest, UpdatePathDoesNotAllocate)
{
  const Open::Response response = open();
  ASSERT_TRUE(response.success);
  RollingControllerRtTestPeer::submit(
    *controller_, makeTrajectory(response, 900'000'000U, 0.05));
  const std::uint64_t before = tracked_allocation_count.load(std::memory_order_relaxed);
  track_allocations.store(true, std::memory_order_relaxed);
  for (std::size_t cycle = 0U; cycle < 100U; ++cycle) {
    if (update() != controller_interface::return_type::OK) {
      track_allocations.store(false, std::memory_order_relaxed);
      FAIL() << "update failed at cycle " << cycle;
    }
  }
  track_allocations.store(false, std::memory_order_relaxed);
  EXPECT_EQ(tracked_allocation_count.load(std::memory_order_relaxed), before);
}

TEST_F(RtUpdateTest, ConcurrentOpenPublicationAndRtConsumptionRemainCoherent)
{
  std::atomic_bool rt_ready{false};
  std::atomic_bool start{false};
  std::atomic_bool prime_published{false};
  std::atomic_bool producer_done{false};
  std::atomic<std::uint64_t> update_errors{0U};
  std::thread rt_thread([&]() {
      rt_ready.store(true, std::memory_order_release);
      while (!start.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      if (update() != controller_interface::return_type::OK) {
        update_errors.fetch_add(1U, std::memory_order_relaxed);
      }
      while (!prime_published.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      std::size_t cycle = 1U;
      while (
        cycle < 20U &&
        (!producer_done.load(std::memory_order_acquire) || cycle < 10U))
      {
        if (update() != controller_interface::return_type::OK) {
          update_errors.fetch_add(1U, std::memory_order_relaxed);
        }
        ++cycle;
        std::this_thread::yield();
      }
    });

  while (!rt_ready.load(std::memory_order_acquire)) {
    std::this_thread::yield();
  }
  start.store(true, std::memory_order_release);
  const Open::Response response = open();
  if (!response.success) {
    prime_published.store(true, std::memory_order_release);
    producer_done.store(true, std::memory_order_release);
    rt_thread.join();
    FAIL() << "concurrent open was rejected with code " <<
      static_cast<unsigned int>(response.error_code);
  }
  Batch prime = makeTrajectory(response, 900'000'000U, 0.05);
  RollingControllerRtTestPeer::submit(*controller_, prime);
  prime_published.store(true, std::memory_order_release);
  for (std::uint64_t sequence = 2U; sequence <= 10U; ++sequence) {
    Batch candidate = makeTrajectory(response, 900'000'000U, 0.05);
    candidate.sequence = sequence;
    RollingControllerRtTestPeer::submit(*controller_, candidate);
  }
  producer_done.store(true, std::memory_order_release);
  rt_thread.join();

  ASSERT_EQ(update_errors.load(std::memory_order_relaxed), 0U);
  if (RollingControllerRtTestPeer::sessionState(*controller_) == SessionState::kPriming) {
    ASSERT_EQ(update(), controller_interface::return_type::OK);
  }
  EXPECT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kRunning)
    << "stop_reason=" << static_cast<unsigned int>(
    RollingControllerRtTestPeer::stopReason(*controller_)) <<
    ", invariant_stage=" << static_cast<unsigned int>(
    RollingControllerRtTestPeer::invariantStage(*controller_));
  EXPECT_EQ(RollingControllerRtTestPeer::stopReason(*controller_), StopReason::kNone);
  EXPECT_GE(RollingControllerRtTestPeer::activeGeneration(*controller_), 1U);
  for (const double command : command_positions_) {
    EXPECT_TRUE(std::isfinite(command));
  }
}

TEST_F(RtUpdateTest, ConcurrentRunningReplacementNeverTurnsAProtocolRejectIntoAStop)
{
  constexpr std::uint64_t duration_ns = 900'000'000U;
  constexpr double displacement = 0.05;
  const Open::Response response = open();
  ASSERT_TRUE(response.success);
  RollingControllerRtTestPeer::submit(
    *controller_, makeTrajectory(response, duration_ns, displacement));
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  ASSERT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kRunning);

  std::atomic_bool start{false};
  std::atomic_bool producer_done{false};
  std::atomic<std::uint64_t> update_errors{0U};
  std::thread rt_thread([&]() {
      while (!start.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      std::size_t cycle = 0U;
      while (
        cycle < 100U &&
        (!producer_done.load(std::memory_order_acquire) || cycle < 40U))
      {
        if (update() != controller_interface::return_type::OK) {
          update_errors.fetch_add(1U, std::memory_order_relaxed);
        }
        ++cycle;
        std::this_thread::yield();
      }
    });

  start.store(true, std::memory_order_release);
  for (std::uint64_t sequence = 2U; sequence <= 80U; ++sequence) {
    const std::uint64_t replace_from_ns =
      RollingControllerRtTestPeer::replaceableFrom(*controller_);
    if (replace_from_ns >= duration_ns) {
      break;
    }
    RollingControllerRtTestPeer::submit(
      *controller_,
      makeReplacement(
        response, sequence, replace_from_ns, duration_ns,
        displacement, displacement));
    std::this_thread::yield();
  }
  producer_done.store(true, std::memory_order_release);
  rt_thread.join();

  EXPECT_EQ(update_errors.load(std::memory_order_relaxed), 0U);
  EXPECT_EQ(
    RollingControllerRtTestPeer::sessionState(*controller_), SessionState::kRunning)
    << "stop_reason=" << static_cast<unsigned int>(
    RollingControllerRtTestPeer::stopReason(*controller_)) <<
    ", invariant_stage=" << static_cast<unsigned int>(
    RollingControllerRtTestPeer::invariantStage(*controller_));
  EXPECT_EQ(RollingControllerRtTestPeer::stopReason(*controller_), StopReason::kNone);
  EXPECT_GE(RollingControllerRtTestPeer::activeGeneration(*controller_), 1U);
  for (const double command : command_positions_) {
    EXPECT_TRUE(std::isfinite(command));
  }
}

}  // namespace
}  // namespace rolling_trajectory_controller
