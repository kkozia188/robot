#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <new>
#include <thread>

#include "rolling_trajectory_controller/rolling_snapshot.hpp"
#include "rolling_trajectory_controller/rolling_types.hpp"

namespace
{

std::atomic<bool> track_allocations{false};
std::atomic<std::uint64_t> tracked_allocation_count{0U};

void countAllocation() noexcept
{
  if (track_allocations.load(std::memory_order_relaxed)) {
    tracked_allocation_count.fetch_add(1U, std::memory_order_relaxed);
  }
}

}  // namespace

void * operator new(std::size_t size)
{
  countAllocation();
  if (void * memory = std::malloc(size)) {
    return memory;
  }
  throw std::bad_alloc();
}

void * operator new[](std::size_t size)
{
  countAllocation();
  if (void * memory = std::malloc(size)) {
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

namespace rolling_trajectory_controller
{
namespace
{

TrajectoryImage makeImage(std::uint64_t generation)
{
  TrajectoryImage image;
  image.generation = generation;
  image.point_count = 1U;
  image.earliest_changed_ns = generation * 2U;
  image.points[0].time_ns = generation;
  image.points[0].positions[0] = static_cast<double>(generation);
  image.points[0].velocities[13] = -static_cast<double>(generation);
  return image;
}

SessionIdentity makeIdentity(std::uint64_t generation)
{
  SessionIdentity identity;
  identity.controller_boot_id[0] = static_cast<std::uint8_t>(generation);
  identity.session_id[0] = static_cast<std::uint8_t>(generation * 3U);
  identity.client_instance_id[0] = static_cast<std::uint8_t>(generation * 7U);
  return identity;
}

TEST(RollingSnapshotExchange, EmptyExchangeHasNoReadableSnapshot)
{
  RollingSnapshotExchange exchange;
  SnapshotLease lease;
  EXPECT_FALSE(exchange.acquire(lease));
  EXPECT_FALSE(lease.valid());
  EXPECT_EQ(exchange.latestPublicationSequence(), 0U);
}

TEST(RollingSnapshotExchange, GenerationAndArrivalTimestampAreOneCoherentPublication)
{
  RollingSnapshotExchange exchange;
  const TrajectoryImage image = makeImage(7U);
  const SessionIdentity identity = makeIdentity(7U);
  ASSERT_TRUE(exchange.publish(identity, image, 123'456U));
  EXPECT_EQ(exchange.latestPublicationSequence(), 1U);

  SnapshotLease lease;
  ASSERT_TRUE(exchange.acquire(lease));
  ASSERT_TRUE(lease.valid());
  ASSERT_NE(lease.get(), nullptr);
  EXPECT_EQ(lease->trajectory.generation, 7U);
  EXPECT_EQ(lease->trajectory.points[0].time_ns, 7U);
  EXPECT_EQ(lease->identity.controller_boot_id, identity.controller_boot_id);
  EXPECT_EQ(lease->identity.session_id, identity.session_id);
  EXPECT_EQ(lease->identity.client_instance_id, identity.client_instance_id);
  EXPECT_EQ(lease->arrival_time_ns, 123'456U);
  EXPECT_EQ(lease->publication_sequence, 1U);
}

TEST(RollingSnapshotExchange, ProducerNeverOverwritesASlotHeldByTheReader)
{
  RollingSnapshotExchange exchange;
  ASSERT_TRUE(exchange.publish(makeIdentity(1U), makeImage(1U), 10U));
  SnapshotLease held;
  ASSERT_TRUE(exchange.acquire(held));
  ASSERT_EQ(held->trajectory.generation, 1U);

  for (std::uint64_t generation = 2U; generation <= 100U; ++generation) {
    ASSERT_TRUE(
      exchange.publish(makeIdentity(generation), makeImage(generation), generation * 10U));
    EXPECT_EQ(held->trajectory.generation, 1U);
    EXPECT_EQ(held->arrival_time_ns, 10U);
    EXPECT_EQ(held->trajectory.points[0].positions[0], 1.0);
  }

  held.reset();
  SnapshotLease latest;
  ASSERT_TRUE(exchange.acquire(latest));
  EXPECT_EQ(latest->trajectory.generation, 100U);
  EXPECT_EQ(latest->arrival_time_ns, 1'000U);
  EXPECT_EQ(latest->publication_sequence, 100U);
}

TEST(RollingSnapshotExchange, PublishAcquireAndReleaseDoNotAllocate)
{
  RollingSnapshotExchange exchange;
  const TrajectoryImage image = makeImage(1U);
  SnapshotLease lease;
  const std::uint64_t before = tracked_allocation_count.load(std::memory_order_relaxed);
  track_allocations.store(true, std::memory_order_relaxed);
  const bool published = exchange.publish(makeIdentity(1U), image, 5U);
  const bool acquired = exchange.acquire(lease);
  const RollingSnapshot * snapshot = lease.get();
  lease.reset();
  track_allocations.store(false, std::memory_order_relaxed);

  EXPECT_TRUE(published);
  EXPECT_TRUE(acquired);
  EXPECT_NE(snapshot, nullptr);
  EXPECT_EQ(tracked_allocation_count.load(std::memory_order_relaxed), before);
}

TEST(RollingSnapshotExchange, MillionPublicationStressHasNoTornOrAbaSnapshot)
{
  constexpr std::uint64_t kIterations = 1'000'000U;
  RollingSnapshotExchange exchange;
  std::atomic<bool> producer_finished{false};
  std::atomic<std::uint64_t> errors{0U};
  std::atomic<std::uint64_t> consumer_last_sequence{0U};

  std::thread consumer([&]() {
      std::uint64_t last_sequence = 0U;
      while (!producer_finished.load(std::memory_order_acquire) || last_sequence < kIterations) {
        SnapshotLease lease;
        if (!exchange.acquire(lease)) {
          continue;
        }
        const RollingSnapshot & snapshot = *lease;
        const std::uint64_t sequence = snapshot.publication_sequence;
        const std::uint64_t generation = snapshot.trajectory.generation;
        if (
          sequence < last_sequence || generation != sequence ||
          snapshot.arrival_time_ns != generation * 3U ||
          snapshot.trajectory.point_count != 1U ||
          snapshot.trajectory.earliest_changed_ns != generation * 2U ||
          snapshot.identity.controller_boot_id[0] != static_cast<std::uint8_t>(generation) ||
          snapshot.identity.session_id[0] != static_cast<std::uint8_t>(generation * 3U) ||
          snapshot.identity.client_instance_id[0] != static_cast<std::uint8_t>(generation * 7U) ||
          snapshot.trajectory.points[0].time_ns != generation ||
          snapshot.trajectory.points[0].positions[0] != static_cast<double>(generation) ||
          snapshot.trajectory.points[0].velocities[13] != -static_cast<double>(generation))
        {
          errors.fetch_add(1U, std::memory_order_relaxed);
        }
        last_sequence = std::max(last_sequence, sequence);
        consumer_last_sequence.store(last_sequence, std::memory_order_release);
      }
    });

  std::thread producer([&]() {
      for (std::uint64_t generation = 1U; generation <= kIterations; ++generation) {
        if (!exchange.publish(makeIdentity(generation), makeImage(generation), generation * 3U)) {
          errors.fetch_add(1U, std::memory_order_relaxed);
        }
      }
      producer_finished.store(true, std::memory_order_release);
    });

  producer.join();
  consumer.join();
  EXPECT_EQ(errors.load(std::memory_order_relaxed), 0U);
  EXPECT_EQ(consumer_last_sequence.load(std::memory_order_acquire), kIterations);
  EXPECT_EQ(exchange.latestPublicationSequence(), kIterations);
}

}  // namespace
}  // namespace rolling_trajectory_controller
