#ifndef ROLLING_TRAJECTORY_CONTROLLER__ROLLING_SNAPSHOT_HPP_
#define ROLLING_TRAJECTORY_CONTROLLER__ROLLING_SNAPSHOT_HPP_

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "rolling_trajectory_controller/rolling_types.hpp"

namespace rolling_trajectory_controller
{

inline constexpr std::size_t kSnapshotSlotCount = 3U;

struct RollingSnapshot
{
  SessionIdentity identity{};
  TrajectoryImage trajectory{};
  std::uint64_t arrival_time_ns{0U};
  std::uint64_t publication_sequence{0U};
};

class RollingSnapshotExchange;

class SnapshotLease
{
public:
  SnapshotLease() noexcept = default;
  ~SnapshotLease() noexcept;
  SnapshotLease(const SnapshotLease &) = delete;
  SnapshotLease & operator=(const SnapshotLease &) = delete;
  SnapshotLease(SnapshotLease && other) noexcept;
  SnapshotLease & operator=(SnapshotLease && other) noexcept;

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] const RollingSnapshot * get() const noexcept;
  [[nodiscard]] const RollingSnapshot & operator*() const noexcept;
  [[nodiscard]] const RollingSnapshot * operator->() const noexcept;
  void reset() noexcept;

private:
  friend class RollingSnapshotExchange;

  const RollingSnapshotExchange * exchange_{nullptr};
  const RollingSnapshot * snapshot_{nullptr};
  std::size_t slot_index_{kSnapshotSlotCount};
};

class RollingSnapshotExchange
{
public:
  RollingSnapshotExchange() noexcept = default;
  RollingSnapshotExchange(const RollingSnapshotExchange &) = delete;
  RollingSnapshotExchange & operator=(const RollingSnapshotExchange &) = delete;

  bool publish(
    const SessionIdentity & identity, const TrajectoryImage & trajectory,
    std::uint64_t arrival_time_ns) noexcept;
  bool acquire(SnapshotLease & lease) const noexcept;
  [[nodiscard]] std::uint64_t latestPublicationSequence() const noexcept;

private:
  friend class SnapshotLease;

  enum class SlotState : std::uint8_t
  {
    kReady = 0U,
    kReading = 1U,
    kWriting = 2U
  };

  struct alignas (64) Slot
  {
    mutable std::atomic<std::uint8_t> state{
      static_cast<std::uint8_t>(SlotState::kReady)};
    RollingSnapshot snapshot{};
  };

  void release(std::size_t slot_index) const noexcept;

  mutable std::array<Slot, kSnapshotSlotCount> slots_{};
  std::atomic<std::uint64_t> published_token_{0U};
  std::atomic_flag writer_active_ = ATOMIC_FLAG_INIT;
  std::uint64_t publication_sequence_{0U};
};

static_assert(std::atomic<std::uint8_t>::is_always_lock_free);
static_assert(std::atomic<std::uint64_t>::is_always_lock_free);

}  // namespace rolling_trajectory_controller

#endif  // ROLLING_TRAJECTORY_CONTROLLER__ROLLING_SNAPSHOT_HPP_
