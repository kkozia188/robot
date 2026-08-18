#ifndef ROLLING_TRAJECTORY_CONTROLLER__CUBIC_HERMITE_HPP_
#define ROLLING_TRAJECTORY_CONTROLLER__CUBIC_HERMITE_HPP_

#include <limits>

namespace rolling_trajectory_controller
{

inline constexpr double kNormalizedRootTolerance =
  64.0 * std::numeric_limits<double>::epsilon();

struct CubicHermite
{
  double start_position{0.0};
  double start_velocity{0.0};
  double end_position{0.0};
  double end_velocity{0.0};
  double duration_seconds{0.0};
  double c0{0.0};
  double c1{0.0};
  double c2{0.0};
  double c3{0.0};
};

struct ScalarKinematicState
{
  double position{0.0};
  double velocity{0.0};
  double acceleration{0.0};
};

struct ScalarKinematicExtrema
{
  double position_min{0.0};
  double position_max{0.0};
  double velocity_positive{0.0};
  double velocity_negative{0.0};
  double acceleration_positive{0.0};
  double acceleration_negative{0.0};
};

bool buildCubicHermite(
  double start_position, double start_velocity,
  double end_position, double end_velocity,
  double duration_seconds, CubicHermite & segment) noexcept;

bool sampleCubicHermite(
  const CubicHermite & segment, double normalized_time,
  ScalarKinematicState & state) noexcept;

bool computeCubicHermiteExtrema(
  const CubicHermite & segment, ScalarKinematicExtrema & extrema) noexcept;

}  // namespace rolling_trajectory_controller

#endif  // ROLLING_TRAJECTORY_CONTROLLER__CUBIC_HERMITE_HPP_
