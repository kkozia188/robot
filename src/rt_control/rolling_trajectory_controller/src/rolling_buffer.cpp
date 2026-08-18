#include "rolling_trajectory_controller/rolling_buffer.hpp"

#include "rolling_trajectory_controller/cubic_hermite.hpp"
#include "rolling_trajectory_controller/limit_checker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rolling_trajectory_controller
{

bool RollingBuffer::configure(std::size_t capacity) noexcept
{
  BufferConfiguration configuration;
  configuration.capacity = capacity;
  return configure(configuration);
}

bool RollingBuffer::configure(const BufferConfiguration & configuration) noexcept
{
  if (configured_ && session_state_ != SessionState::kNone) {
    return false;
  }
  if (configuration.capacity == 0U || configuration.capacity > kTransportMaxPoints) {
    return false;
  }
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    if (
      !std::isfinite(configuration.splice_position_tolerance[axis]) ||
      !std::isfinite(configuration.splice_velocity_tolerance[axis]) ||
      configuration.splice_position_tolerance[axis] < 0.0 ||
      configuration.splice_velocity_tolerance[axis] < 0.0)
    {
      return false;
    }
  }

  configuration_ = configuration;
  image_ = TrajectoryImage{};
  pending_image_ = TrajectoryImage{};
  pending_valid_ = false;
  identity_ = SessionIdentity{};
  session_state_ = SessionState::kNone;
  last_seen_sequence_ = 0U;
  last_accepted_sequence_ = 0U;
  replaceable_from_ns_ = 0U;
  limit_checker_ = LimitChecker{};
  configured_ = true;
  return true;
}

bool RollingBuffer::configureLimits(
  const DynamicEnvelope & envelope, bool allow_test_only_limits) noexcept
{
  if (!configured_ || session_state_ != SessionState::kNone) {
    return false;
  }
  return limit_checker_.configure(envelope, allow_test_only_limits);
}

bool RollingBuffer::configured() const noexcept
{
  return configured_;
}

std::size_t RollingBuffer::capacity() const noexcept
{
  return configuration_.capacity;
}

const TrajectoryImage & RollingBuffer::image() const noexcept
{
  return image_;
}

const TrajectoryImage & RollingBuffer::pendingImage() const noexcept
{
  return pending_image_;
}

bool RollingBuffer::hasPending() const noexcept
{
  return pending_valid_;
}

SessionState RollingBuffer::sessionState() const noexcept
{
  return session_state_;
}

std::uint64_t RollingBuffer::lastSeenSequence() const noexcept
{
  return last_seen_sequence_;
}

std::uint64_t RollingBuffer::lastAcceptedSequence() const noexcept
{
  return last_accepted_sequence_;
}

std::uint64_t RollingBuffer::validationBaseGeneration() const noexcept
{
  return validationHead().generation;
}

const SessionIdentity & RollingBuffer::identity() const noexcept
{
  return identity_;
}

RejectCode RollingBuffer::validateInput(
  const PointView * points, std::size_t point_count) const noexcept
{
  if (points == nullptr || point_count == 0U) {
    return RejectCode::kInvalidShape;
  }
  if (point_count > configuration_.capacity) {
    return RejectCode::kCapacityExceeded;
  }

  for (std::size_t point_index = 0U; point_index < point_count; ++point_index) {
    const PointView & input = points[point_index];
    if (
      input.positions == nullptr || input.velocities == nullptr ||
      input.position_count != kAxisCount || input.velocity_count != kAxisCount)
    {
      return RejectCode::kInvalidShape;
    }
  }

  for (std::size_t point_index = 0U; point_index < point_count; ++point_index) {
    const PointView & input = points[point_index];
    for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
      if (!std::isfinite(input.positions[axis]) || !std::isfinite(input.velocities[axis])) {
        return RejectCode::kNonFinite;
      }
    }
  }

  for (std::size_t point_index = 1U; point_index < point_count; ++point_index) {
    const PointView & input = points[point_index];
    if (input.time_ns <= points[point_index - 1U].time_ns) {
      return RejectCode::kNonMonotonicTime;
    }
  }
  return RejectCode::kNone;
}

void RollingBuffer::copyPoint(const PointView & input, JointPoint & output) const noexcept
{
  output.time_ns = input.time_ns;
  for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
    output.positions[axis] = input.positions[axis];
    output.velocities[axis] = input.velocities[axis];
  }
}

RejectCode RollingBuffer::replace(
  const PointView * points, std::size_t point_count) noexcept
{
  if (!configured_) {
    return RejectCode::kSessionNotAccepting;
  }
  if (session_state_ != SessionState::kNone) {
    return RejectCode::kSessionNotAccepting;
  }
  const RejectCode validation = validateInput(points, point_count);
  if (validation != RejectCode::kNone) {
    return validation;
  }
  if (image_.generation == std::numeric_limits<std::uint64_t>::max()) {
    return RejectCode::kSessionNotAccepting;
  }

  TrajectoryImage candidate{};
  candidate.point_count = point_count;
  candidate.generation = image_.generation + 1U;

  for (std::size_t point_index = 0U; point_index < point_count; ++point_index) {
    copyPoint(points[point_index], candidate.points[point_index]);
  }

  image_ = candidate;
  return RejectCode::kNone;
}

bool RollingBuffer::beginSession(const SessionIdentity & identity) noexcept
{
  if (
    !configured_ || !limit_checker_.configured() ||
    session_state_ != SessionState::kNone)
  {
    return false;
  }
  identity_ = identity;
  image_ = TrajectoryImage{};
  pending_image_ = TrajectoryImage{};
  pending_valid_ = false;
  last_seen_sequence_ = 0U;
  last_accepted_sequence_ = 0U;
  replaceable_from_ns_ = 0U;
  session_state_ = SessionState::kPriming;
  return true;
}

RejectCode RollingBuffer::prepare(
  const BatchView & batch, PreparedSubmission & submission) noexcept
{
  submission = PreparedSubmission{};
  if (!configured_ || session_state_ == SessionState::kNone ||
    session_state_ == SessionState::kTerminated)
  {
    return RejectCode::kSessionNotAccepting;
  }
  if (batch.protocol_major != kProtocolMajor || batch.protocol_minor != kProtocolMinor) {
    return RejectCode::kWrongProtocol;
  }
  if (batch.controller_boot_id != identity_.controller_boot_id) {
    return RejectCode::kWrongBoot;
  }
  if (batch.session_id != identity_.session_id) {
    return RejectCode::kWrongSession;
  }
  if (batch.client_instance_id != identity_.client_instance_id) {
    return RejectCode::kWrongClient;
  }
  if (batch.sequence <= last_seen_sequence_) {
    return RejectCode::kStaleSequence;
  }
  last_seen_sequence_ = batch.sequence;

  if (session_state_ != SessionState::kPriming && session_state_ != SessionState::kRunning) {
    return RejectCode::kSessionNotAccepting;
  }
  const RejectCode input_validation = validateInput(batch.points, batch.point_count);
  if (input_validation != RejectCode::kNone) {
    return input_validation;
  }

  const TrajectoryImage & head = validationHead();
  if (head.generation == std::numeric_limits<std::uint64_t>::max()) {
    return RejectCode::kSessionNotAccepting;
  }

  TrajectoryImage & candidate = submission.candidate;
  candidate.generation = head.generation + 1U;
  if (head.point_count == 0U) {
    if (batch.replace_from_ns != 0U || batch.points[0].time_ns != 0U) {
      return RejectCode::kTimeGap;
    }
    candidate.point_count = batch.point_count;
    candidate.earliest_changed_ns = 0U;
    for (std::size_t index = 0U; index < batch.point_count; ++index) {
      copyPoint(batch.points[index], candidate.points[index]);
    }
  } else {
    if (batch.replace_from_ns < replaceable_from_ns_) {
      return RejectCode::kLateReplace;
    }
    if (batch.points[0].time_ns != batch.replace_from_ns) {
      return RejectCode::kTimeGap;
    }

    JointPoint splice{};
    if (!sampleTrajectoryImage(head, batch.replace_from_ns, splice)) {
      return RejectCode::kTimeGap;
    }
    JointPoint replacement_splice{};
    copyPoint(batch.points[0], replacement_splice);
    const RejectCode continuity = checkSpliceContinuity(
      splice, replacement_splice,
      configuration_.splice_position_tolerance,
      configuration_.splice_velocity_tolerance);
    if (continuity != RejectCode::kNone) {
      return continuity;
    }

    std::size_t prefix_count = 0U;
    while (
      prefix_count < head.point_count &&
      head.points[prefix_count].time_ns < batch.replace_from_ns)
    {
      ++prefix_count;
    }
    if (prefix_count + batch.point_count > configuration_.capacity) {
      return RejectCode::kCapacityExceeded;
    }

    candidate.point_count = prefix_count + batch.point_count;
    candidate.earliest_changed_ns = pending_valid_ ?
      std::min(head.earliest_changed_ns, batch.replace_from_ns) : batch.replace_from_ns;
    for (std::size_t index = 0U; index < prefix_count; ++index) {
      candidate.points[index] = head.points[index];
    }
    for (std::size_t index = 0U; index < batch.point_count; ++index) {
      copyPoint(batch.points[index], candidate.points[prefix_count + index]);
    }
  }

  if (
    head.point_count == 0U &&
    candidate.points[candidate.point_count - 1U].time_ns <
    configuration_.required_initial_horizon_ns)
  {
    return RejectCode::kInsufficientHorizon;
  }

  const RejectCode candidate_validation = validateCandidate(candidate);
  if (candidate_validation != RejectCode::kNone) {
    return candidate_validation;
  }

  submission.identity = identity_;
  submission.validation_base_generation = head.generation;
  submission.sequence = batch.sequence;
  submission.valid = true;
  return RejectCode::kNone;
}

RejectCode RollingBuffer::commit(
  PreparedSubmission & submission, std::uint64_t replaceable_from_ns) noexcept
{
  if (!submission.valid) {
    return RejectCode::kSessionNotAccepting;
  }
  submission.valid = false;
  if (
    !configured_ ||
    (session_state_ != SessionState::kPriming && session_state_ != SessionState::kRunning) ||
    submission.identity.controller_boot_id != identity_.controller_boot_id ||
    submission.identity.session_id != identity_.session_id ||
    submission.identity.client_instance_id != identity_.client_instance_id ||
    submission.sequence != last_seen_sequence_ ||
    submission.sequence <= last_accepted_sequence_)
  {
    return RejectCode::kSessionNotAccepting;
  }

  const TrajectoryImage & head = validationHead();
  if (
    head.generation != submission.validation_base_generation ||
    submission.candidate.generation != head.generation + 1U)
  {
    return RejectCode::kSessionNotAccepting;
  }

  replaceable_from_ns_ = std::max(replaceable_from_ns_, replaceable_from_ns);
  if (submission.candidate.earliest_changed_ns < replaceable_from_ns_) {
    return RejectCode::kLateReplace;
  }

  pending_image_ = submission.candidate;
  pending_valid_ = true;
  last_accepted_sequence_ = submission.sequence;
  return RejectCode::kNone;
}

RejectCode RollingBuffer::submit(const BatchView & batch) noexcept
{
  PreparedSubmission submission;
  const RejectCode prepared = prepare(batch, submission);
  if (prepared != RejectCode::kNone) {
    return prepared;
  }
  return commit(submission, replaceable_from_ns_);
}

bool RollingBuffer::activatePending() noexcept
{
  if (
    !pending_valid_ ||
    (session_state_ != SessionState::kPriming && session_state_ != SessionState::kRunning))
  {
    return false;
  }
  image_ = pending_image_;
  pending_image_ = TrajectoryImage{};
  pending_valid_ = false;
  if (session_state_ == SessionState::kPriming) {
    session_state_ = SessionState::kRunning;
  }
  return true;
}

bool RollingBuffer::acknowledgeActiveGeneration(std::uint64_t generation) noexcept
{
  if (
    generation == 0U ||
    (session_state_ != SessionState::kPriming && session_state_ != SessionState::kRunning))
  {
    return false;
  }
  if (pending_valid_ && pending_image_.generation < generation) {
    return false;
  }
  if (image_.generation > generation) {
    return false;
  }
  if (!pending_valid_ && image_.generation != generation) {
    return false;
  }
  if (pending_valid_ && pending_image_.generation == generation) {
    image_ = pending_image_;
    pending_image_ = TrajectoryImage{};
    pending_valid_ = false;
  }
  if (session_state_ == SessionState::kPriming) {
    session_state_ = SessionState::kRunning;
  }
  return true;
}

bool RollingBuffer::requestStop() noexcept
{
  if (session_state_ == SessionState::kStopping) {
    return true;
  }
  if (session_state_ != SessionState::kPriming && session_state_ != SessionState::kRunning) {
    return false;
  }
  pending_image_ = TrajectoryImage{};
  pending_valid_ = false;
  session_state_ = SessionState::kStopping;
  return true;
}

bool RollingBuffer::markHolding() noexcept
{
  if (session_state_ == SessionState::kHolding) {
    return true;
  }
  if (session_state_ != SessionState::kStopping) {
    return false;
  }
  session_state_ = SessionState::kHolding;
  return true;
}

bool RollingBuffer::finishSession() noexcept
{
  if (session_state_ != SessionState::kHolding) {
    return false;
  }
  clearSessionData();
  session_state_ = SessionState::kNone;
  return true;
}

void RollingBuffer::terminateSession() noexcept
{
  clearSessionData();
  session_state_ = SessionState::kTerminated;
}

bool RollingBuffer::resetTerminated() noexcept
{
  if (session_state_ != SessionState::kTerminated) {
    return false;
  }
  session_state_ = SessionState::kNone;
  return true;
}

void RollingBuffer::setReplaceableFromNs(std::uint64_t replaceable_from_ns) noexcept
{
  replaceable_from_ns_ = std::max(replaceable_from_ns_, replaceable_from_ns);
}

bool RollingBuffer::sampleValidationHead(
  std::uint64_t time_ns, JointPoint & point) const noexcept
{
  return sampleTrajectoryImage(validationHead(), time_ns, point);
}

const TrajectoryImage & RollingBuffer::validationHead() const noexcept
{
  return pending_valid_ ? pending_image_ : image_;
}

void RollingBuffer::clearSessionData() noexcept
{
  image_ = TrajectoryImage{};
  pending_image_ = TrajectoryImage{};
  pending_valid_ = false;
  identity_ = SessionIdentity{};
  last_seen_sequence_ = 0U;
  last_accepted_sequence_ = 0U;
  replaceable_from_ns_ = 0U;
}

RejectCode RollingBuffer::validateCandidate(const TrajectoryImage & candidate) const noexcept
{
  SegmentExtrema extrema;
  for (std::size_t index = 0U; index + 1U < candidate.point_count; ++index) {
    const SegmentCheckResult direct =
      limit_checker_.checkSegment(candidate.points[index], candidate.points[index + 1U], extrema);
    if (direct.code != RejectCode::kNone) {
      return direct.code;
    }
  }

  for (std::size_t index = 0U; index + 1U < candidate.point_count; ++index) {
    const SegmentCheckResult direct =
      limit_checker_.checkSegment(candidate.points[index], candidate.points[index + 1U], extrema);
    if (direct.code != RejectCode::kNone) {
      return direct.code;
    }
    StoppingEnvelope stopping_envelope;
    const SegmentCheckResult stopping =
      limit_checker_.checkStoppingViability(extrema, stopping_envelope);
    if (stopping.code != RejectCode::kNone) {
      return stopping.code;
    }
  }
  return RejectCode::kNone;
}

bool sampleTrajectoryImage(
  const TrajectoryImage & image, std::uint64_t time_ns, JointPoint & point) noexcept
{
  if (
    image.point_count == 0U || time_ns < image.points[0].time_ns ||
    time_ns > image.points[image.point_count - 1U].time_ns)
  {
    return false;
  }

  for (std::size_t index = 0U; index < image.point_count; ++index) {
    if (image.points[index].time_ns == time_ns) {
      point = image.points[index];
      return true;
    }
  }

  for (std::size_t index = 0U; index + 1U < image.point_count; ++index) {
    const JointPoint & start = image.points[index];
    const JointPoint & end = image.points[index + 1U];
    if (time_ns <= start.time_ns || time_ns >= end.time_ns) {
      continue;
    }

    const std::uint64_t duration_ns = end.time_ns - start.time_ns;
    const std::uint64_t elapsed_ns = time_ns - start.time_ns;
    const double duration_seconds = static_cast<double>(duration_ns) * 1.0e-9;
    const double s = static_cast<double>(elapsed_ns) / static_cast<double>(duration_ns);

    point.time_ns = time_ns;
    for (std::size_t axis = 0U; axis < kAxisCount; ++axis) {
      CubicHermite segment;
      ScalarKinematicState state;
      if (
        !buildCubicHermite(
          start.positions[axis], start.velocities[axis],
          end.positions[axis], end.velocities[axis],
          duration_seconds, segment) ||
        !sampleCubicHermite(segment, s, state))
      {
        return false;
      }
      point.positions[axis] = state.position;
      point.velocities[axis] = state.velocity;
    }
    return true;
  }
  return false;
}

}  // namespace rolling_trajectory_controller
