#include "rolling_trajectory_controller/cubic_hermite.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace rolling_trajectory_controller
{
namespace
{

struct Roots
{
  std::array<double, 2> values{};
  std::size_t count{0U};
};

bool allFinite(const CubicHermite & segment) noexcept
{
  return
    std::isfinite(segment.start_position) && std::isfinite(segment.start_velocity) &&
    std::isfinite(segment.end_position) && std::isfinite(segment.end_velocity) &&
    std::isfinite(segment.duration_seconds) && segment.duration_seconds > 0.0 &&
    std::isfinite(segment.c0) && std::isfinite(segment.c1) &&
    std::isfinite(segment.c2) && std::isfinite(segment.c3);
}

bool normalizedCandidate(double root, double & candidate) noexcept
{
  if (!std::isfinite(root) || root < -kNormalizedRootTolerance ||
    root > 1.0 + kNormalizedRootTolerance)
  {
    return false;
  }
  candidate = std::clamp(root, 0.0, 1.0);
  return true;
}

Roots realRoots(double quadratic, double linear, double constant) noexcept
{
  Roots roots;
  const double scale = std::max(
    {std::abs(quadratic), std::abs(linear), std::abs(constant)});
  if (!std::isfinite(scale) || scale == 0.0) {
    return roots;
  }

  const double a = quadratic / scale;
  const double b = linear / scale;
  const double c = constant / scale;
  if (std::abs(a) <= kNormalizedRootTolerance) {
    if (std::abs(b) > kNormalizedRootTolerance) {
      roots.values[roots.count++] = -c / b;
    }
    return roots;
  }

  double discriminant = std::fma(-4.0 * a, c, b * b);
  const double discriminant_scale = b * b + std::abs(4.0 * a * c);
  if (
    discriminant < 0.0 &&
    -discriminant <= kNormalizedRootTolerance * discriminant_scale)
  {
    discriminant = 0.0;
  }
  if (discriminant < 0.0 || !std::isfinite(discriminant)) {
    return roots;
  }

  const double square_root = std::sqrt(discriminant);
  if (square_root == 0.0) {
    roots.values[roots.count++] = -b / (2.0 * a);
    return roots;
  }

  const double q = -0.5 * (b + std::copysign(square_root, b));
  roots.values[roots.count++] = q / a;
  if (q != 0.0) {
    roots.values[roots.count++] = c / q;
  }
  return roots;
}

bool addExtremumCandidate(
  const CubicHermite & segment, double root,
  ScalarKinematicExtrema & extrema) noexcept
{
  double normalized_time = 0.0;
  if (!normalizedCandidate(root, normalized_time)) {
    return true;
  }

  ScalarKinematicState state;
  if (!sampleCubicHermite(segment, normalized_time, state)) {
    return false;
  }
  extrema.position_min = std::min(extrema.position_min, state.position);
  extrema.position_max = std::max(extrema.position_max, state.position);
  return true;
}

bool addVelocityCandidate(
  const CubicHermite & segment, double root,
  ScalarKinematicExtrema & extrema) noexcept
{
  double normalized_time = 0.0;
  if (!normalizedCandidate(root, normalized_time)) {
    return true;
  }

  ScalarKinematicState state;
  if (!sampleCubicHermite(segment, normalized_time, state)) {
    return false;
  }
  extrema.velocity_positive = std::max(extrema.velocity_positive, state.velocity);
  extrema.velocity_negative = std::max(extrema.velocity_negative, -state.velocity);
  return true;
}

}  // namespace

bool buildCubicHermite(
  double start_position, double start_velocity,
  double end_position, double end_velocity,
  double duration_seconds, CubicHermite & segment) noexcept
{
  if (
    !std::isfinite(start_position) || !std::isfinite(start_velocity) ||
    !std::isfinite(end_position) || !std::isfinite(end_velocity) ||
    !std::isfinite(duration_seconds) || duration_seconds <= 0.0)
  {
    return false;
  }

  CubicHermite candidate;
  candidate.start_position = start_position;
  candidate.start_velocity = start_velocity;
  candidate.end_position = end_position;
  candidate.end_velocity = end_velocity;
  candidate.duration_seconds = duration_seconds;
  candidate.c0 = start_position;
  candidate.c1 = duration_seconds * start_velocity;
  candidate.c2 =
    -3.0 * start_position - 2.0 * duration_seconds * start_velocity +
    3.0 * end_position - duration_seconds * end_velocity;
  candidate.c3 =
    2.0 * start_position + duration_seconds * start_velocity -
    2.0 * end_position + duration_seconds * end_velocity;
  if (!allFinite(candidate)) {
    return false;
  }

  segment = candidate;
  return true;
}

bool sampleCubicHermite(
  const CubicHermite & segment, double normalized_time,
  ScalarKinematicState & state) noexcept
{
  if (
    !allFinite(segment) || !std::isfinite(normalized_time) ||
    normalized_time < 0.0 || normalized_time > 1.0)
  {
    return false;
  }

  ScalarKinematicState candidate;
  if (normalized_time == 0.0) {
    candidate.position = segment.start_position;
    candidate.velocity = segment.start_velocity;
  } else if (normalized_time == 1.0) {
    candidate.position = segment.end_position;
    candidate.velocity = segment.end_velocity;
  } else {
    candidate.position =
      ((segment.c3 * normalized_time + segment.c2) * normalized_time + segment.c1) *
      normalized_time + segment.c0;
    candidate.velocity =
      ((3.0 * segment.c3 * normalized_time + 2.0 * segment.c2) * normalized_time +
      segment.c1) / segment.duration_seconds;
  }
  candidate.acceleration =
    (2.0 * segment.c2 + 6.0 * segment.c3 * normalized_time) /
    (segment.duration_seconds * segment.duration_seconds);
  if (
    !std::isfinite(candidate.position) || !std::isfinite(candidate.velocity) ||
    !std::isfinite(candidate.acceleration))
  {
    return false;
  }

  state = candidate;
  return true;
}

bool computeCubicHermiteExtrema(
  const CubicHermite & segment, ScalarKinematicExtrema & extrema) noexcept
{
  ScalarKinematicState start;
  ScalarKinematicState end;
  if (
    !sampleCubicHermite(segment, 0.0, start) ||
    !sampleCubicHermite(segment, 1.0, end))
  {
    return false;
  }

  ScalarKinematicExtrema candidate;
  candidate.position_min = std::min(start.position, end.position);
  candidate.position_max = std::max(start.position, end.position);
  candidate.velocity_positive = std::max({start.velocity, end.velocity, 0.0});
  candidate.velocity_negative = std::max({-start.velocity, -end.velocity, 0.0});
  candidate.acceleration_positive = std::max({start.acceleration, end.acceleration, 0.0});
  candidate.acceleration_negative = std::max({-start.acceleration, -end.acceleration, 0.0});

  const Roots position_roots = realRoots(3.0 * segment.c3, 2.0 * segment.c2, segment.c1);
  for (std::size_t index = 0U; index < position_roots.count; ++index) {
    if (!addExtremumCandidate(segment, position_roots.values[index], candidate)) {
      return false;
    }
  }

  const Roots velocity_roots = realRoots(0.0, 6.0 * segment.c3, 2.0 * segment.c2);
  for (std::size_t index = 0U; index < velocity_roots.count; ++index) {
    if (!addVelocityCandidate(segment, velocity_roots.values[index], candidate)) {
      return false;
    }
  }

  candidate.velocity_positive = std::max(candidate.velocity_positive, 0.0);
  candidate.velocity_negative = std::max(candidate.velocity_negative, 0.0);
  if (
    !std::isfinite(candidate.position_min) || !std::isfinite(candidate.position_max) ||
    !std::isfinite(candidate.velocity_positive) ||
    !std::isfinite(candidate.velocity_negative) ||
    !std::isfinite(candidate.acceleration_positive) ||
    !std::isfinite(candidate.acceleration_negative))
  {
    return false;
  }

  extrema = candidate;
  return true;
}

}  // namespace rolling_trajectory_controller
