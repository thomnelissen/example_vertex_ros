// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//

#include "figure8_references.hpp"

#include <spdlog/spdlog.h>
#include "eigen3/Eigen/Geometry"

#include <algorithm>
#include <cassert>
#include <cmath>

namespace
{
// The unit figure 8. It starts at its crossing heading +45 degrees from its long axis.
struct UnitFigure8
{
    double x, y;   // position, along and across the long axis
    double dx, dy; // first derivative
    double ddx, ddy;
};

UnitFigure8 unitFigure8(const double theta)
{
    const double f = std::sin(theta);
    const double g = std::cos(theta);
    const double d = 1.0 + g * g;

    UnitFigure8 c;
    c.x   = f / d;
    c.y   = f * g / d;
    c.dx  = g * (3.0 - g * g) / (d * d);
    c.dy  = (3.0 * g * g - 1.0) / (d * d);
    c.ddx = f * (-3.0 * f * f * d + 4.0 * g * g * (3.0 - g * g)) / (d * d * d);
    c.ddy = g * f * (6.0 * g * g - 10.0) / (d * d * d);
    return c;
}
} // namespace

Figure8References::Figure8References(double update_frequency_hz,
                                     double size_m,
                                     double speed_mps,
                                     double accel_mps2,
                                     double climb_m,
                                     bool   frontal)
    : time_step_s_(1.0 / update_frequency_hz),
      size_m_(size_m),
      speed_mps_(speed_mps),
      accel_mps2_(accel_mps2),
      climb_m_(climb_m),
      frontal_(frontal)
{
    // Only support update frequencies above 10Hz.
    assert(update_frequency_hz >= 10);
}

double Figure8References::PeakAccelFactor()
{
    double peak = 0.0;
    for(int i = 0; i < 3600; ++i)
    {
        const auto c = unitFigure8(2.0 * std::numbers::pi * i / 3600.0);
        peak         = std::max(peak, std::hypot(c.ddx, c.ddy));
    }
    return peak;
}

void Figure8References::Reset(const std::array<float, 3> position, const double yaw_heading)
{
    initial_heading_ = yaw_heading;
    start_point_     = position;

    spdlog::debug("Figure8References: Start point: {}, {}, Heading: {}",
                  start_point_[AxisIndex::kXAxis], start_point_[AxisIndex::kYAxis],
                  initial_heading_);

    // Restart time so you will begin from the start of the figure 8
    time_s_     = 0.0;
    time_scale_ = 1.0;
}

creos_messages::StateReference Figure8References::GetNewStateReference(
    const std::array<float, 3> &drone_position)
{
    // Compute the current point on the figure 8.
    auto reference = computeNewPosition(start_point_, initial_heading_, time_s_);

    reference.timestamp = creos::RobotClock::now();
    reference.frame_id  = "map";

    // The reference is a function of time. When the drone falls behind it, slow down time so
    // the reference waits for the drone instead of running away along the figure 8. The
    // feedforward is the derivative in time, so it slows down with it.
    updateTimeScale(reference, drone_position);
    reference.velocity.linear.x *= time_scale_;
    reference.velocity.linear.y *= time_scale_;
    reference.velocity.linear.z *= time_scale_;
    reference.acceleration.linear.x *= time_scale_ * time_scale_;
    reference.acceleration.linear.y *= time_scale_ * time_scale_;
    reference.acceleration.linear.z *= time_scale_ * time_scale_;

    // Fly the figure 8 in position mode and attitude mode.
    reference.translation_mode = creos_messages::StateReference::TranslationMode::kPosition;
    reference.orientation_mode = creos_messages::StateReference::OrientationMode::kAttitude;

    spdlog::debug("StateReference -> Position: [{}, {}, {}] Heading: [{}] Time scale: [{}] "
                  "Start point: [{}, {}, {}]",
                  reference.pose.position.x, reference.pose.position.y, reference.pose.position.z,
                  initial_heading_, time_scale_, start_point_[AxisIndex::kXAxis],
                  start_point_[AxisIndex::kYAxis], start_point_[AxisIndex::kZAxis]);

    // Update time for next iteration, slowed down when the drone is behind
    time_s_ += time_scale_ * time_step_s_;

    return reference;
}

void Figure8References::updateTimeScale(const creos_messages::StateReference &reference,
                                       const std::array<float, 3>           &drone_position)
{
    // Distance between the setpoint and where the drone is
    const double error = std::sqrt(
        std::pow(reference.pose.position.x - drone_position[AxisIndex::kXAxis], 2) +
        std::pow(reference.pose.position.y - drone_position[AxisIndex::kYAxis], 2) +
        std::pow(reference.pose.position.z - drone_position[AxisIndex::kZAxis], 2));

    // Full speed when close, waiting when far, and linear in between
    const double target =
        std::clamp((kStopErrorM - error) / (kStopErrorM - kFullSpeedErrorM), 0.0, 1.0);

    // Change the time scale gradually, so the setpoint does not jerk
    const double max_step = kTimeScaleRate * time_step_s_;
    time_scale_ += std::clamp(target - time_scale_, -max_step, max_step);
}

const creos_messages::StateReference Figure8References::computeNewPosition(
    const std::array<float, 3> &start,
    const double                initial_heading,
    const double                time_s) const
{
    creos_messages::StateReference reference;

    // Compute the curve angle (and its time derivatives) based on time, speed, and acceleration,
    // as the circle example does. The speed is reached at the tips, where the figure 8 is
    // sharpest; through the crossing it is 0.71 of it.
    const double rate               = speed_mps_ / size_m_;
    const double speed_accel_time_s = speed_mps_ / accel_mps2_;
    const double angle              = rate * (std::hypot(time_s, speed_accel_time_s) - speed_accel_time_s);
    const double d_angle_dt         = rate * (time_s / std::hypot(time_s, speed_accel_time_s));
    const double d2_angle_dt2 =
        rate * (std::pow(speed_accel_time_s, 2) / std::pow(std::hypot(time_s, speed_accel_time_s), 3));

    // The long axis points 45 degrees right of the initial heading, so the figure 8 starts
    // straight ahead.
    const double axis   = initial_heading - std::numbers::pi / 4.0;
    const double ux     = std::cos(axis), uy = std::sin(axis);  // along the long axis
    const double wx     = -std::sin(axis), wy = std::cos(axis); // across it, to the left
    const auto   c      = unitFigure8(angle);

    // Compute the position of the drone on the figure 8
    reference.pose.position.x = start[AxisIndex::kXAxis] + size_m_ * (ux * c.x + wx * c.y);
    reference.pose.position.y = start[AxisIndex::kYAxis] + size_m_ * (uy * c.x + wy * c.y);

    // Compute feedforward velocity
    const double tx = size_m_ * (ux * c.dx + wx * c.dy); // d position / d angle
    const double ty = size_m_ * (uy * c.dx + wy * c.dy);
    reference.velocity.linear.x = tx * d_angle_dt;
    reference.velocity.linear.y = ty * d_angle_dt;

    // Compute feedforward acceleration
    reference.acceleration.linear.x =
        size_m_ * (ux * c.ddx + wx * c.ddy) * std::pow(d_angle_dt, 2) + tx * d2_angle_dt2;
    reference.acceleration.linear.y =
        size_m_ * (uy * c.ddx + wy * c.ddy) * std::pow(d_angle_dt, 2) + ty * d2_angle_dt2;

    // Height: climb up over the first lobe and down over the second, around the start height
    reference.pose.position.z   = start[AxisIndex::kZAxis] + climb_m_ * std::sin(angle);
    reference.velocity.linear.z = climb_m_ * std::cos(angle) * d_angle_dt;
    reference.acceleration.linear.z = -climb_m_ * std::sin(angle) * std::pow(d_angle_dt, 2) +
                                      climb_m_ * std::cos(angle) * d2_angle_dt2;

    // Keep heading constant, or turn the nose along the path
    const double heading = frontal_ ? std::atan2(ty, tx) : initial_heading;
    Eigen::Quaterniond orientation(Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()));
    reference.pose.orientation.x = orientation.x();
    reference.pose.orientation.y = orientation.y();
    reference.pose.orientation.z = orientation.z();
    reference.pose.orientation.w = orientation.w();
    return reference;
}
