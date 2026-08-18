#include "rolling_trajectory_controller/rolling_snapshot.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>

namespace rolling_trajectory_controller
{
namespace
{

constexpr std::uint64_t kSlotIndexBits = 2U;
constexpr std::uint64_t kSlotIndexMask = (1U << kSlotIndexBits) - 1U;
constexpr std::uint64_t kMaxPublicationSequence =
  std::numeric_limits<std::uint64_t>::max() >> kSlotIndexBits;

std::uint64_t makeToken(std::uint64_t sequence, std::size_t slot_index) noexcept
{
  return (sequence << kSlotIndexBits) | static_cast<std::uint64_t>(slot_index);
}

std::size_t tokenSlot(std::uint64_t token) noexcept
{
  return static_cast<std::size_t>(token & kSlotIndexMask);
}

std::uint64_t tokenSequence(std::uint64_t token) noexcept
{
  return token >> kSlotIndexBits;
}

class WriterGuard
{
public:
  explicit WriterGuard(std::atomic_flag & flag) noexcept
  : flag_(flag), owns_(!flag_.test_and_set(std::memory_order_acquire))
  {
  }

  ~WriterGuard() noexcept
  {
    if (owns_) {
      flag_.clear(std::memory_order_release);
    }
  }

  [[nodiscard]] bool owns() const noexcept
  {
    return owns_;
  }

private:
  std::atomic_flag & flag_;
  bool owns_{false};
};

}  // namespace

SnapshotLease::~SnapshotLease() noexcept
{
  reset();
}

SnapshotLease::SnapshotLease(SnapshotLease && other) noexcept
: exchange_(std::exchange(other.exchange_, nullptr)),
  snapshot_(std::exchange(other.snapshot_, nullptr)),
  slot_index_(std::exchange(other.slot_index_, kSnapshotSlotCount))
{
}

SnapshotLease & SnapshotLease::operator=(SnapshotLease && other) noexcept
{
  if (this != &other) {
    reset();
    exchange_ = std::exchange(other.exchange_, nullptr);
    snapshot_ = std::exchange(other.snapshot_, nullptr);
    slot_index_ = std::exchange(other.slot_index_, kSnapshotSlotCount);
  }
  return *this;
}

bool SnapshotLease::valid() const noexcept
{
  return exchange_ != nullptr && snapshot_ != nullptr && slot_index_ < kSnapshotSlotCount;
}

const RollingSnapshot * SnapshotLease::get() const noexcept
{
  return snapshot_;
}

const RollingSnapshot & SnapshotLease::operator*() const noexcept
{
  return *snapshot_;
}

const RollingSnapshot * SnapshotLease::operator->() const noexcept
{
  return snapshot_;
}

void SnapshotLease::reset() noexcept
{
  if (valid()) {
    exchange_->release(slot_index_);
  }
  exchange_ = nullptr;
  snapshot_ = nullptr;
  slot_index_ = kSnapshotSlotCount;
}

bool RollingSnapshotExchange::publish(
  const SessionIdentity & identity, const TrajectoryImage & trajectory,
  std::uint64_t arrival_time_ns) noexcept
{
  WriterGuard writer(writer_active_);
  if (!writer.owns() || publication_sequence_ >= kMaxPublicationSequence) {
    return false;
  }

  const std::uint64_t current_token = published_token_.load(std::memory_order_acquire);
  const std::size_t current_slot = current_token == 0U ?
    kSnapshotSlotCount : tokenSlot(current_token);
  std::size_t selected_slot = kSnapshotSlotCount;
  for (std::size_t slot_index = 0U; slot_index < kSnapshotSlotCount; ++slot_index) {
    if (slot_index == current_slot) {
      continue;
    }
    std::uint8_t expected = static_cast<std::uint8_t>(SlotState::kReady);
    if (slots_[slot_index].state.compare_exchange_strong(
        expected, static_cast<std::uint8_t>(SlotState::kWriting),
        std::memory_order_acquire, std::memory_order_relaxed))
    {
      selected_slot = slot_index;
      break;
    }
  }
  if (selected_slot == kSnapshotSlotCount) {
    return false;
  }

  const std::uint64_t sequence = publication_sequence_ + 1U;
  RollingSnapshot & snapshot = slots_[selected_slot].snapshot;
  snapshot.identity = identity;
  snapshot.trajectory = trajectory;
  snapshot.arrival_time_ns = arrival_time_ns;
  snapshot.publication_sequence = sequence;
  slots_[selected_slot].state.store(
    static_cast<std::uint8_t>(SlotState::kReady), std::memory_order_release);
  published_token_.store(makeToken(sequence, selected_slot), std::memory_order_release);
  publication_sequence_ = sequence;
  return true;
}

bool RollingSnapshotExchange::acquire(SnapshotLease & lease) const noexcept
{
  if (lease.valid()) {
    return false;
  }

  for (std::size_t attempt = 0U; attempt < kSnapshotSlotCount; ++attempt) {
    const std::uint64_t token = published_token_.load(std::memory_order_acquire);
    if (token == 0U) {
      return false;
    }
    const std::size_t slot_index = tokenSlot(token);
    if (slot_index >= kSnapshotSlotCount) {
      return false;
    }

    std::uint8_t expected = static_cast<std::uint8_t>(SlotState::kReady);
    if (!slots_[slot_index].state.compare_exchange_strong(
        expected, static_cast<std::uint8_t>(SlotState::kReading),
        std::memory_order_acquire, std::memory_order_relaxed))
    {
      continue;
    }

    const RollingSnapshot & snapshot = slots_[slot_index].snapshot;
    if (snapshot.publication_sequence != tokenSequence(token)) {
      slots_[slot_index].state.store(
        static_cast<std::uint8_t>(SlotState::kReady), std::memory_order_release);
      continue;
    }
    lease.exchange_ = this;
    lease.snapshot_ = &snapshot;
    lease.slot_index_ = slot_index;
    return true;
  }
  return false;
}

std::uint64_t RollingSnapshotExchange::latestPublicationSequence() const noexcept
{
  return tokenSequence(published_token_.load(std::memory_order_acquire));
}

void RollingSnapshotExchange::release(std::size_t slot_index) const noexcept
{
  if (slot_index < kSnapshotSlotCount) {
    slots_[slot_index].state.store(
      static_cast<std::uint8_t>(SlotState::kReady), std::memory_order_release);
  }
}

}  // namespace rolling_trajectory_controller
