#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>

#include "rolling_trajectory_controller/limit_checker.hpp"
#include "rolling_trajectory_controller/rolling_buffer.hpp"

namespace rolling_trajectory_controller
{
namespace
{

constexpr std::uint64_t kCycleNs = 4'000'000U;

Identifier makeIdentifier(std::uint8_t seed)
{
  Identifier identifier{};
  for (std::size_t index = 0; index < identifier.size(); ++index) {
    identifier[index] = static_cast<std::uint8_t>(seed + index);
  }
  return identifier;
}

struct OwnedPoint
{
  std::uint64_t time_ns{0U};
  std::array<double, kAxisCount> positions{};
  std::array<double, kAxisCount> velocities{};

  PointView view() const noexcept
  {
    return PointView{
      time_ns, positions.data(), positions.size(), velocities.data(), velocities.size()};
  }
};

OwnedPoint makePoint(std::uint64_t time_ns, double position, double velocity = 0.0)
{
  OwnedPoint point;
  point.time_ns = time_ns;
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    point.positions[axis] = position + static_cast<double>(axis) * 0.01;
    point.velocities[axis] = velocity;
  }
  return point;
}

DynamicEnvelope makeTestOnlyEnvelope()
{
  DynamicEnvelope envelope;
  envelope.source = LimitsSource::kTestOnly;
  envelope.limits_version[0] = 0xE1U;
  envelope.limits_version[1] = 0x02U;
  for (AxisEnvelope & axis : envelope.axes) {
    axis.position_lower = -1.0e6;
    axis.position_upper = 1.0e6;
    axis.velocity_positive = 1.0e6;
    axis.velocity_negative = 1.0e6;
    axis.acceleration_positive = 1.0e9;
    axis.acceleration_negative = 1.0e9;
    axis.stop_acceleration_positive = 1.0e6;
    axis.stop_acceleration_negative = 1.0e6;
    axis.position_margin_lower = 0.0;
    axis.position_margin_upper = 0.0;
  }
  return envelope;
}

template<std::size_t Size>
std::array<PointView, Size> makeViews(const std::array<OwnedPoint, Size> & points)
{
  std::array<PointView, Size> views{};
  for (std::size_t index = 0U; index < Size; ++index) {
    views[index] = points[index].view();
  }
  return views;
}

class ProtocolVectors : public ::testing::Test
{
protected:
  void SetUp() override
  {
    BufferConfiguration configuration;
    configuration.capacity = 8U;
    configuration.splice_position_tolerance.fill(1.0e-10);
    configuration.splice_velocity_tolerance.fill(1.0e-10);
    ASSERT_TRUE(buffer_.configure(configuration));
    ASSERT_TRUE(buffer_.configureLimits(makeTestOnlyEnvelope(), true));

    identity_.controller_boot_id = makeIdentifier(1U);
    identity_.session_id = makeIdentifier(21U);
    identity_.client_instance_id = makeIdentifier(41U);
  }

  BatchView batch(
    std::uint64_t sequence, std::uint64_t replace_from_ns,
    const PointView * points, std::size_t point_count) const
  {
    return BatchView{
      kProtocolMajor, kProtocolMinor, identity_.controller_boot_id, identity_.session_id,
      identity_.client_instance_id, sequence, replace_from_ns, points, point_count};
  }

  RollingBuffer buffer_;
  SessionIdentity identity_{};
};

TEST(RollingBufferSession, RefusesToBeginWithoutAnApprovedDynamicEnvelope)
{
  RollingBuffer buffer;
  ASSERT_TRUE(buffer.configure(8U));
  SessionIdentity identity;
  identity.controller_boot_id = makeIdentifier(1U);
  identity.session_id = makeIdentifier(21U);
  identity.client_instance_id = makeIdentifier(41U);

  EXPECT_FALSE(buffer.beginSession(identity));
  EXPECT_EQ(buffer.sessionState(), SessionState::kNone);
  EXPECT_TRUE(buffer.configureLimits(makeTestOnlyEnvelope(), true));
  EXPECT_TRUE(buffer.beginSession(identity));
  EXPECT_FALSE(buffer.configureLimits(makeTestOnlyEnvelope(), true));
}

TEST_F(ProtocolVectors, WrongIdentityDoesNotConsumeSequenceButPayloadFailureDoes)
{
  ASSERT_TRUE(buffer_.beginSession(identity_));
  std::array<OwnedPoint, 2> points = {makePoint(0U, 0.0), makePoint(kCycleNs, 0.1)};
  auto views = makeViews(points);

  auto update = batch(1U, 0U, views.data(), views.size());
  update.controller_boot_id = makeIdentifier(2U);
  EXPECT_EQ(buffer_.submit(update), RejectCode::kWrongBoot);
  EXPECT_EQ(buffer_.lastSeenSequence(), 0U);

  update = batch(1U, 0U, views.data(), views.size());
  update.session_id = makeIdentifier(22U);
  EXPECT_EQ(buffer_.submit(update), RejectCode::kWrongSession);
  EXPECT_EQ(buffer_.lastSeenSequence(), 0U);

  update = batch(1U, 0U, views.data(), views.size());
  update.client_instance_id = makeIdentifier(42U);
  EXPECT_EQ(buffer_.submit(update), RejectCode::kWrongClient);
  EXPECT_EQ(buffer_.lastSeenSequence(), 0U);

  points[1].positions[3] = std::numeric_limits<double>::quiet_NaN();
  views = makeViews(points);
  EXPECT_EQ(buffer_.submit(batch(1U, 0U, views.data(), views.size())), RejectCode::kNonFinite);
  EXPECT_EQ(buffer_.lastSeenSequence(), 1U);
  EXPECT_EQ(buffer_.lastAcceptedSequence(), 0U);

  points[1] = makePoint(kCycleNs, 0.1);
  views = makeViews(points);
  EXPECT_EQ(
    buffer_.submit(batch(1U, 0U, views.data(), views.size())), RejectCode::kStaleSequence);
  EXPECT_EQ(buffer_.lastSeenSequence(), 1U);

  EXPECT_EQ(buffer_.submit(batch(3U, 0U, views.data(), views.size())), RejectCode::kNone);
  EXPECT_EQ(buffer_.lastSeenSequence(), 3U);
  EXPECT_EQ(buffer_.lastAcceptedSequence(), 3U);
  EXPECT_EQ(buffer_.pendingImage().generation, 1U);
}

TEST_F(ProtocolVectors, PrimingRequiresAnExactZeroReplacementAndFirstPoint)
{
  ASSERT_TRUE(buffer_.beginSession(identity_));
  std::array<OwnedPoint, 2> points = {makePoint(1U, 0.0), makePoint(kCycleNs, 0.1)};
  auto views = makeViews(points);

  EXPECT_EQ(buffer_.submit(batch(1U, 1U, views.data(), views.size())), RejectCode::kTimeGap);
  EXPECT_EQ(buffer_.sessionState(), SessionState::kPriming);

  EXPECT_EQ(buffer_.submit(batch(2U, 0U, views.data(), views.size())), RejectCode::kTimeGap);
  EXPECT_EQ(buffer_.sessionState(), SessionState::kPriming);

  points[0].time_ns = 0U;
  views = makeViews(points);
  EXPECT_EQ(buffer_.submit(batch(3U, 0U, views.data(), views.size())), RejectCode::kNone);
  EXPECT_TRUE(buffer_.hasPending());
  EXPECT_EQ(buffer_.sessionState(), SessionState::kPriming);

  ASSERT_TRUE(buffer_.activatePending());
  EXPECT_EQ(buffer_.sessionState(), SessionState::kRunning);
  EXPECT_EQ(buffer_.image().generation, 1U);
}

TEST_F(ProtocolVectors, PrimeMustMeetTheConfiguredInitialHorizonExactly)
{
  BufferConfiguration configuration;
  configuration.capacity = 4U;
  configuration.required_initial_horizon_ns = 2U * kCycleNs;
  configuration.splice_position_tolerance.fill(1.0e-10);
  configuration.splice_velocity_tolerance.fill(1.0e-10);
  ASSERT_TRUE(buffer_.configure(configuration));
  ASSERT_TRUE(buffer_.configureLimits(makeTestOnlyEnvelope(), true));
  ASSERT_TRUE(buffer_.beginSession(identity_));

  std::array<OwnedPoint, 2> points = {
    makePoint(0U, 0.0), makePoint(2U * kCycleNs - 1U, 0.1)};
  auto views = makeViews(points);
  EXPECT_EQ(
    buffer_.submit(batch(1U, 0U, views.data(), views.size())),
    RejectCode::kInsufficientHorizon);
  EXPECT_EQ(buffer_.lastSeenSequence(), 1U);
  EXPECT_FALSE(buffer_.hasPending());

  points[1] = makePoint(2U * kCycleNs, 0.1);
  views = makeViews(points);
  EXPECT_EQ(
    buffer_.submit(batch(2U, 0U, views.data(), views.size())), RejectCode::kNone);
  EXPECT_TRUE(buffer_.hasPending());
}

TEST_F(ProtocolVectors, LateSuffixIsRejectedAfterSequenceConsumption)
{
  ASSERT_TRUE(buffer_.beginSession(identity_));
  const std::array<OwnedPoint, 3> prime = {
    makePoint(0U, 0.0), makePoint(kCycleNs, 0.1), makePoint(2U * kCycleNs, 0.2)};
  const auto prime_views = makeViews(prime);
  ASSERT_EQ(
    buffer_.submit(batch(1U, 0U, prime_views.data(), prime_views.size())),
    RejectCode::kNone);
  ASSERT_TRUE(buffer_.activatePending());
  buffer_.setReplaceableFromNs(kCycleNs);

  const std::array<OwnedPoint, 2> suffix = {
    makePoint(kCycleNs - 1U, 0.1), makePoint(2U * kCycleNs, 0.2)};
  const auto suffix_views = makeViews(suffix);
  EXPECT_EQ(
    buffer_.submit(batch(2U, kCycleNs - 1U, suffix_views.data(), suffix_views.size())),
    RejectCode::kLateReplace);
  EXPECT_EQ(buffer_.lastSeenSequence(), 2U);
  EXPECT_EQ(buffer_.lastAcceptedSequence(), 1U);
  EXPECT_FALSE(buffer_.hasPending());
  EXPECT_EQ(buffer_.image().generation, 1U);
}

TEST_F(ProtocolVectors, FinalCommitRechecksBoundaryWithoutMutatingTheAcceptedHead)
{
  ASSERT_TRUE(buffer_.beginSession(identity_));
  const std::array<OwnedPoint, 3> prime = {
    makePoint(0U, 0.0), makePoint(kCycleNs, 0.1), makePoint(2U * kCycleNs, 0.2)};
  const auto prime_views = makeViews(prime);
  ASSERT_EQ(
    buffer_.submit(batch(1U, 0U, prime_views.data(), prime_views.size())),
    RejectCode::kNone);
  ASSERT_TRUE(buffer_.activatePending());
  const TrajectoryImage accepted_before = buffer_.image();

  JointPoint splice;
  ASSERT_TRUE(buffer_.sampleValidationHead(kCycleNs, splice));
  std::array<OwnedPoint, 2> suffix = {
    makePoint(kCycleNs, 0.0), makePoint(3U * kCycleNs, 0.3)};
  suffix[0].positions = splice.positions;
  suffix[0].velocities = splice.velocities;
  const auto suffix_views = makeViews(suffix);

  PreparedSubmission prepared;
  ASSERT_EQ(
    buffer_.prepare(
      batch(2U, kCycleNs, suffix_views.data(), suffix_views.size()), prepared),
    RejectCode::kNone);
  ASSERT_TRUE(prepared.valid);
  EXPECT_EQ(buffer_.lastSeenSequence(), 2U);
  EXPECT_EQ(buffer_.lastAcceptedSequence(), 1U);
  EXPECT_FALSE(buffer_.hasPending());

  EXPECT_EQ(
    buffer_.commit(prepared, kCycleNs + 1U), RejectCode::kLateReplace);
  EXPECT_FALSE(prepared.valid);
  EXPECT_EQ(buffer_.lastSeenSequence(), 2U);
  EXPECT_EQ(buffer_.lastAcceptedSequence(), 1U);
  EXPECT_FALSE(buffer_.hasPending());
  EXPECT_EQ(
    std::memcmp(&accepted_before, &buffer_.image(), sizeof(TrajectoryImage)), 0);
}

TEST_F(ProtocolVectors, ValidSuffixRetainsOnlyTheImmutablePrefix)
{
  ASSERT_TRUE(buffer_.beginSession(identity_));
  const std::array<OwnedPoint, 4> prime = {
    makePoint(0U, 0.0), makePoint(kCycleNs, 0.1), makePoint(2U * kCycleNs, 0.2),
    makePoint(3U * kCycleNs, 0.3)};
  const auto prime_views = makeViews(prime);
  ASSERT_EQ(
    buffer_.submit(batch(1U, 0U, prime_views.data(), prime_views.size())),
    RejectCode::kNone);
  ASSERT_TRUE(buffer_.activatePending());
  buffer_.setReplaceableFromNs(kCycleNs);

  JointPoint splice{};
  ASSERT_TRUE(buffer_.sampleValidationHead(2U * kCycleNs, splice));
  std::array<OwnedPoint, 3> suffix = {
    makePoint(2U * kCycleNs, 0.0), makePoint(3U * kCycleNs, 0.8),
    makePoint(4U * kCycleNs, 0.9)};
  suffix[0].positions = splice.positions;
  suffix[0].velocities = splice.velocities;
  const auto suffix_views = makeViews(suffix);

  ASSERT_EQ(
    buffer_.submit(batch(2U, 2U * kCycleNs, suffix_views.data(), suffix_views.size())),
    RejectCode::kNone);
  ASSERT_TRUE(buffer_.hasPending());
  EXPECT_EQ(buffer_.image().generation, 1U);
  EXPECT_EQ(buffer_.pendingImage().generation, 2U);
  ASSERT_EQ(buffer_.pendingImage().point_count, 5U);
  EXPECT_EQ(buffer_.pendingImage().points[0].time_ns, 0U);
  EXPECT_EQ(buffer_.pendingImage().points[1].time_ns, kCycleNs);
  EXPECT_EQ(buffer_.pendingImage().points[2].time_ns, 2U * kCycleNs);
  EXPECT_EQ(buffer_.pendingImage().points[3].positions[0], 0.8);

  ASSERT_TRUE(buffer_.activatePending());
  EXPECT_EQ(buffer_.image().generation, 2U);
  EXPECT_FALSE(buffer_.hasPending());
}

TEST_F(ProtocolVectors, SuffixMustMatchBothPositionAndVelocityAtTheSplice)
{
  ASSERT_TRUE(buffer_.beginSession(identity_));
  const std::array<OwnedPoint, 3> prime = {
    makePoint(0U, 0.0, 0.1), makePoint(kCycleNs, 0.1, 0.1),
    makePoint(2U * kCycleNs, 0.2, 0.1)};
  const auto prime_views = makeViews(prime);
  ASSERT_EQ(
    buffer_.submit(batch(1U, 0U, prime_views.data(), prime_views.size())),
    RejectCode::kNone);
  ASSERT_TRUE(buffer_.activatePending());

  JointPoint splice{};
  ASSERT_TRUE(buffer_.sampleValidationHead(kCycleNs, splice));
  std::array<OwnedPoint, 2> suffix = {
    makePoint(kCycleNs, 0.0), makePoint(3U * kCycleNs, 0.3)};
  suffix[0].positions = splice.positions;
  suffix[0].velocities = splice.velocities;
  suffix[0].positions[6] += 1.0e-9;
  auto suffix_views = makeViews(suffix);
  EXPECT_EQ(
    buffer_.submit(batch(2U, kCycleNs, suffix_views.data(), suffix_views.size())),
    RejectCode::kPositionDiscontinuity);

  suffix[0].positions = splice.positions;
  suffix[0].velocities[6] += 1.0e-9;
  suffix_views = makeViews(suffix);
  EXPECT_EQ(
    buffer_.submit(batch(3U, kCycleNs, suffix_views.data(), suffix_views.size())),
    RejectCode::kVelocityDiscontinuity);
  EXPECT_EQ(buffer_.image().generation, 1U);
  EXPECT_FALSE(buffer_.hasPending());
}

TEST_F(ProtocolVectors, NewUpdateBuildsAgainstPendingValidationHead)
{
  ASSERT_TRUE(buffer_.beginSession(identity_));
  const std::array<OwnedPoint, 3> prime = {
    makePoint(0U, 0.0), makePoint(kCycleNs, 0.1), makePoint(2U * kCycleNs, 0.2)};
  const auto prime_views = makeViews(prime);
  ASSERT_EQ(
    buffer_.submit(batch(1U, 0U, prime_views.data(), prime_views.size())),
    RejectCode::kNone);
  ASSERT_TRUE(buffer_.hasPending());
  ASSERT_EQ(buffer_.pendingImage().generation, 1U);

  JointPoint splice{};
  ASSERT_TRUE(buffer_.sampleValidationHead(kCycleNs, splice));
  std::array<OwnedPoint, 2> replacement = {
    makePoint(kCycleNs, 0.0), makePoint(3U * kCycleNs, 0.7)};
  replacement[0].positions = splice.positions;
  replacement[0].velocities = splice.velocities;
  const auto replacement_views = makeViews(replacement);

  ASSERT_EQ(
    buffer_.submit(batch(2U, kCycleNs, replacement_views.data(), replacement_views.size())),
    RejectCode::kNone);
  EXPECT_EQ(buffer_.image().generation, 0U);
  EXPECT_EQ(buffer_.pendingImage().generation, 2U);
  EXPECT_EQ(buffer_.validationBaseGeneration(), 2U);

  ASSERT_TRUE(buffer_.acknowledgeActiveGeneration(1U));
  EXPECT_EQ(buffer_.sessionState(), SessionState::kRunning);
  EXPECT_EQ(buffer_.image().generation, 0U);
  EXPECT_TRUE(buffer_.hasPending());
  EXPECT_EQ(buffer_.pendingImage().generation, 2U);

  ASSERT_TRUE(buffer_.acknowledgeActiveGeneration(2U));
  EXPECT_EQ(buffer_.image().generation, 2U);
  EXPECT_FALSE(buffer_.hasPending());
  EXPECT_EQ(buffer_.image().points[0].time_ns, 0U);
  EXPECT_EQ(buffer_.image().points[1].time_ns, kCycleNs);
  EXPECT_EQ(buffer_.image().points[2].positions[0], 0.7);
}

TEST_F(ProtocolVectors, PrefixPlusSuffixCannotExceedRuntimeCapacity)
{
  BufferConfiguration configuration;
  configuration.capacity = 4U;
  configuration.splice_position_tolerance.fill(1.0e-10);
  configuration.splice_velocity_tolerance.fill(1.0e-10);
  ASSERT_TRUE(buffer_.configure(configuration));
  ASSERT_TRUE(buffer_.configureLimits(makeTestOnlyEnvelope(), true));
  ASSERT_TRUE(buffer_.beginSession(identity_));

  const std::array<OwnedPoint, 4> prime = {
    makePoint(0U, 0.0), makePoint(kCycleNs, 0.1), makePoint(2U * kCycleNs, 0.2),
    makePoint(3U * kCycleNs, 0.3)};
  const auto prime_views = makeViews(prime);
  ASSERT_EQ(
    buffer_.submit(batch(1U, 0U, prime_views.data(), prime_views.size())),
    RejectCode::kNone);
  ASSERT_TRUE(buffer_.activatePending());

  JointPoint splice{};
  ASSERT_TRUE(buffer_.sampleValidationHead(3U * kCycleNs, splice));
  std::array<OwnedPoint, 2> suffix = {
    makePoint(3U * kCycleNs, 0.0), makePoint(4U * kCycleNs, 0.4)};
  suffix[0].positions = splice.positions;
  suffix[0].velocities = splice.velocities;
  const auto suffix_views = makeViews(suffix);

  EXPECT_EQ(
    buffer_.submit(batch(2U, 3U * kCycleNs, suffix_views.data(), suffix_views.size())),
    RejectCode::kCapacityExceeded);
  EXPECT_EQ(buffer_.image().generation, 1U);
  EXPECT_FALSE(buffer_.hasPending());
}

TEST_F(ProtocolVectors, InteriorVelocityViolationCannotBecomePending)
{
  DynamicEnvelope envelope = makeTestOnlyEnvelope();
  envelope.axes[0].velocity_positive = 1.4;
  ASSERT_TRUE(buffer_.configureLimits(envelope, true));
  ASSERT_TRUE(buffer_.beginSession(identity_));

  std::array<OwnedPoint, 2> points{};
  points[0].time_ns = 0U;
  points[1].time_ns = 1'000'000'000U;
  points[1].positions[0] = 1.0;
  const auto views = makeViews(points);
  EXPECT_EQ(
    buffer_.submit(batch(1U, 0U, views.data(), views.size())),
    RejectCode::kVelocityLimit);
  EXPECT_EQ(buffer_.lastSeenSequence(), 1U);
  EXPECT_EQ(buffer_.lastAcceptedSequence(), 0U);
  EXPECT_FALSE(buffer_.hasPending());
  EXPECT_EQ(buffer_.pendingImage().generation, 0U);
}

TEST_F(ProtocolVectors, StopUnsafeDirectlyValidSegmentCannotBecomePending)
{
  DynamicEnvelope envelope = makeTestOnlyEnvelope();
  envelope.axes[0].position_upper = 1.0;
  envelope.axes[0].stop_acceleration_positive = 1.0;
  ASSERT_TRUE(buffer_.configureLimits(envelope, true));
  ASSERT_TRUE(buffer_.beginSession(identity_));

  std::array<OwnedPoint, 2> points{};
  points[0].time_ns = 0U;
  points[0].velocities[0] = 0.8;
  points[1].time_ns = 1'000'000'000U;
  points[1].positions[0] = 0.8;
  points[1].velocities[0] = 0.8;
  const auto views = makeViews(points);
  EXPECT_EQ(
    buffer_.submit(batch(1U, 0U, views.data(), views.size())),
    RejectCode::kNotStoppingViable);
  EXPECT_EQ(buffer_.lastSeenSequence(), 1U);
  EXPECT_EQ(buffer_.lastAcceptedSequence(), 0U);
  EXPECT_FALSE(buffer_.hasPending());
  EXPECT_EQ(buffer_.pendingImage().generation, 0U);
}

TEST_F(ProtocolVectors, ActiveSessionCannotBeResetThroughConfiguration)
{
  ASSERT_TRUE(buffer_.beginSession(identity_));
  const std::array<OwnedPoint, 2> points = {
    makePoint(0U, 0.0), makePoint(kCycleNs, 0.1)};
  const auto views = makeViews(points);
  ASSERT_EQ(buffer_.submit(batch(1U, 0U, views.data(), views.size())), RejectCode::kNone);

  const TrajectoryImage pending_before = buffer_.pendingImage();
  EXPECT_FALSE(buffer_.configure(4U));
  EXPECT_EQ(buffer_.capacity(), 8U);
  EXPECT_EQ(buffer_.sessionState(), SessionState::kPriming);
  EXPECT_TRUE(buffer_.hasPending());
  EXPECT_EQ(
    std::memcmp(&pending_before, &buffer_.pendingImage(), sizeof(TrajectoryImage)), 0);
  EXPECT_EQ(buffer_.lastSeenSequence(), 1U);
  EXPECT_EQ(buffer_.lastAcceptedSequence(), 1U);
}

TEST_F(ProtocolVectors, ActiveSessionCannotBypassProtocolThroughDirectReplace)
{
  ASSERT_TRUE(buffer_.beginSession(identity_));
  const std::array<OwnedPoint, 2> prime = {
    makePoint(0U, 0.0), makePoint(kCycleNs, 0.1)};
  const auto prime_views = makeViews(prime);
  ASSERT_EQ(
    buffer_.submit(batch(1U, 0U, prime_views.data(), prime_views.size())),
    RejectCode::kNone);
  ASSERT_TRUE(buffer_.activatePending());
  const TrajectoryImage active_before = buffer_.image();

  const std::array<OwnedPoint, 2> replacement = {
    makePoint(0U, 9.0), makePoint(kCycleNs, 10.0)};
  const auto replacement_views = makeViews(replacement);
  EXPECT_EQ(
    buffer_.replace(replacement_views.data(), replacement_views.size()),
    RejectCode::kSessionNotAccepting);
  EXPECT_EQ(std::memcmp(&active_before, &buffer_.image(), sizeof(TrajectoryImage)), 0);
  EXPECT_EQ(buffer_.sessionState(), SessionState::kRunning);
  EXPECT_EQ(buffer_.lastSeenSequence(), 1U);
  EXPECT_EQ(buffer_.lastAcceptedSequence(), 1U);
}

TEST_F(ProtocolVectors, GracefulCloseDiscardsPendingFutureAndConsumesLaterSequences)
{
  ASSERT_TRUE(buffer_.beginSession(identity_));
  const std::array<OwnedPoint, 2> prime = {
    makePoint(0U, 0.0), makePoint(kCycleNs, 0.1)};
  const auto prime_views = makeViews(prime);
  ASSERT_EQ(
    buffer_.submit(batch(1U, 0U, prime_views.data(), prime_views.size())),
    RejectCode::kNone);
  ASSERT_TRUE(buffer_.hasPending());

  ASSERT_TRUE(buffer_.requestStop());
  EXPECT_EQ(buffer_.sessionState(), SessionState::kStopping);
  EXPECT_FALSE(buffer_.hasPending());
  EXPECT_FALSE(buffer_.activatePending());

  EXPECT_EQ(
    buffer_.submit(batch(2U, 0U, prime_views.data(), prime_views.size())),
    RejectCode::kSessionNotAccepting);
  EXPECT_EQ(buffer_.lastSeenSequence(), 2U);
  EXPECT_EQ(buffer_.lastAcceptedSequence(), 1U);
  EXPECT_EQ(buffer_.validationBaseGeneration(), 0U);
}

TEST_F(ProtocolVectors, SessionCanOnlyBeDestroyedAfterHolding)
{
  ASSERT_TRUE(buffer_.beginSession(identity_));
  EXPECT_EQ(buffer_.identity().session_id, identity_.session_id);
  EXPECT_FALSE(buffer_.finishSession());
  ASSERT_TRUE(buffer_.requestStop());
  EXPECT_FALSE(buffer_.finishSession());
  ASSERT_TRUE(buffer_.markHolding());
  EXPECT_EQ(buffer_.sessionState(), SessionState::kHolding);
  ASSERT_TRUE(buffer_.finishSession());
  EXPECT_EQ(buffer_.sessionState(), SessionState::kNone);
  EXPECT_EQ(buffer_.identity().controller_boot_id, Identifier{});
  EXPECT_EQ(buffer_.identity().session_id, Identifier{});
  EXPECT_EQ(buffer_.identity().client_instance_id, Identifier{});
  EXPECT_EQ(buffer_.image().point_count, 0U);
  EXPECT_EQ(buffer_.validationBaseGeneration(), 0U);
  EXPECT_EQ(buffer_.lastSeenSequence(), 0U);
}

TEST_F(ProtocolVectors, TerminationInvalidatesTheSessionUntilExplicitReset)
{
  ASSERT_TRUE(buffer_.beginSession(identity_));
  buffer_.terminateSession();
  EXPECT_EQ(buffer_.sessionState(), SessionState::kTerminated);
  EXPECT_EQ(buffer_.identity().controller_boot_id, Identifier{});
  EXPECT_EQ(buffer_.identity().session_id, Identifier{});
  EXPECT_EQ(buffer_.identity().client_instance_id, Identifier{});
  EXPECT_FALSE(buffer_.beginSession(identity_));
  EXPECT_TRUE(buffer_.resetTerminated());
  EXPECT_EQ(buffer_.sessionState(), SessionState::kNone);
  EXPECT_TRUE(buffer_.beginSession(identity_));
}

TEST_F(ProtocolVectors, RandomArrivalOrderMatchesTheSequenceReferenceModel)
{
  constexpr std::uint32_t kSeed = 0xE102U;
  ASSERT_TRUE(buffer_.beginSession(identity_));
  std::array<OwnedPoint, 2> points = {makePoint(0U, 0.0), makePoint(kCycleNs, 0.1)};
  points[1].velocities[0] = std::numeric_limits<double>::quiet_NaN();
  const auto views = makeViews(points);

  std::mt19937 generator(kSeed);
  std::uniform_int_distribution<std::uint64_t> sequence_distribution(1U, 200U);
  std::bernoulli_distribution wrong_client_distribution(0.25);
  std::uint64_t reference_last_seen = 0U;

  for (std::size_t event = 0U; event < 1'000U; ++event) {
    const std::uint64_t sequence = sequence_distribution(generator);
    const bool wrong_client = wrong_client_distribution(generator);
    auto update = batch(sequence, 0U, views.data(), views.size());
    if (wrong_client) {
      update.client_instance_id = makeIdentifier(99U);
    }

    const RejectCode expected = wrong_client ? RejectCode::kWrongClient :
      (sequence <= reference_last_seen ? RejectCode::kStaleSequence : RejectCode::kNonFinite);
    EXPECT_EQ(buffer_.submit(update), expected) << "seed=" << kSeed << " event=" << event;
    if (!wrong_client && sequence > reference_last_seen) {
      reference_last_seen = sequence;
    }
    EXPECT_EQ(buffer_.lastSeenSequence(), reference_last_seen)
      << "seed=" << kSeed << " event=" << event;
    EXPECT_EQ(buffer_.lastAcceptedSequence(), 0U);
    EXPECT_FALSE(buffer_.hasPending());
  }
}

}  // namespace
}  // namespace rolling_trajectory_controller
