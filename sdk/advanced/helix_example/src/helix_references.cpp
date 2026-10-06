// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//

#include "helix_references.hpp"

#include <spdlog/spdlog.h>
#include "eigen3/Eigen/Geometry"

HelixReferences::HelixReferences(double update_frequency_hz,
                                 double circle_radius_m,
                                 double speed_mps,
                                 double accel_mps2,
                                 double climb_m,
                                 double turns,
                                 bool   frontal)
    : time_step_s_(1.0 / update_frequency_hz),
      circle_radius_m_(circle_radius_m),
      speed_mps_(speed_mps),
      accel_mps2_(accel_mps2),
      climb_m_(climb_m),
      turns_(turns),
      frontal_(frontal)
{
    // Only support update frequencies above 10Hz.
    assert(update_frequency_hz >= 10);
}

void HelixReferences::Reset(const std::array<float, 3> position, const double yaw_heading)
{
    initial_heading_     = yaw_heading;
    middle_point_circle_ = computeCircleMiddle(position, initial_heading_);

    spdlog::debug("HelixReferences: Middle point: {}, {}, Heading: {}",
                  middle_point_circle_[AxisIndex::kXAxis], middle_point_circle_[AxisIndex::kYAxis],
                  initial_heading_);

    // Restart time so you will begin from the start of the helix
    time_s_ = 0.0;
}

creos_messages::StateReference HelixReferences::GetNewStateReference()
{
    // Compute the current angle in the helix.
    auto reference = computeNewPosition(middle_point_circle_, initial_heading_, time_s_);

    reference.timestamp = creos::RobotClock::now();
    reference.frame_id  = "map";

    // Fly the helix in position mode and attitude mode.
    reference.translation_mode = creos_messages::StateReference::TranslationMode::kPosition;
    reference.orientation_mode = creos_messages::StateReference::OrientationMode::kAttitude;

    spdlog::debug(
        "StateReference -> Position: [{}, {}, {}] Heading: [{}] Middle point: [{}, {}, {}]",
        reference.pose.position.x, reference.pose.position.y, reference.pose.position.z,
        initial_heading_, middle_point_circle_[AxisIndex::kXAxis],
        middle_point_circle_[AxisIndex::kYAxis], middle_point_circle_[AxisIndex::kZAxis]);

    // Update time for next iteration
    time_s_ += time_step_s_;

    return reference;
}

const creos_messages::StateReference HelixReferences::computeNewPosition(
    const std::array<float, 3> &circle_middle,
    const double                initial_heading,
    const double                time_s) const
{
    creos_messages::StateReference reference;

    // Compute the circle angle (and their time derivatives) based on time, speed, and acceleration.
    const double speed_accel_time_s = speed_mps_ / accel_mps2_;
    const double circle_angle       = -speed_mps_ / circle_radius_m_ *
                                (std::hypot(time_s, speed_accel_time_s) - speed_accel_time_s);
    const double d_angle_dt =
        (-speed_mps_ / circle_radius_m_) * (time_s / std::hypot(time_s, speed_accel_time_s));
    const double d2_angle_dt2 =
        (-speed_mps_ / circle_radius_m_) *
        (std::pow(speed_accel_time_s, 2) / std::pow(std::hypot(time_s, speed_accel_time_s), 3));

    // Compute the position of the drone in the circle. Nose along the path, the circle is turned
    // a quarter so that it starts straight ahead.
    const double base         = circleBase(initial_heading);
    double       cos_angle    = std::cos(circle_angle + base);
    double       sin_angle    = std::sin(circle_angle + base);
    reference.pose.position.x = circle_middle[AxisIndex::kXAxis] + (-circle_radius_m_ * cos_angle);
    reference.pose.position.y = circle_middle[AxisIndex::kYAxis] + (-circle_radius_m_ * sin_angle);

    // Compute feedforward velocity
    reference.velocity.linear.x = -circle_radius_m_ * -sin_angle * d_angle_dt;
    reference.velocity.linear.y = -circle_radius_m_ * cos_angle * d_angle_dt;

    // Compute feedforward acceleration
    reference.acceleration.linear.x = (-circle_radius_m_ * -cos_angle) * std::pow(d_angle_dt, 2) +
                                      (-circle_radius_m_ * -sin_angle) * d2_angle_dt2;
    reference.acceleration.linear.y = (-circle_radius_m_ * -sin_angle) * std::pow(d_angle_dt, 2) +
                                      (-circle_radius_m_ * cos_angle) * d2_angle_dt2;

    // Climb over the given turns and descend over as many, from the start height
    const double k     = 1.0 / (2.0 * turns_);
    const double turn  = -circle_angle; // >= 0, and its time derivatives
    const double dturn = -d_angle_dt;
    const double d2turn = -d2_angle_dt2;
    reference.pose.position.z   = circle_middle[AxisIndex::kZAxis] +
                                climb_m_ * (1.0 - std::cos(k * turn)) / 2.0;
    reference.velocity.linear.z = climb_m_ * k * std::sin(k * turn) * dturn / 2.0;
    reference.acceleration.linear.z =
        climb_m_ * k * k * std::cos(k * turn) * std::pow(dturn, 2) / 2.0 +
        climb_m_ * k * std::sin(k * turn) * d2turn / 2.0;

    // Keep heading constant, or turn the nose along with the circle
    const double heading = frontal_ ? initial_heading + circle_angle : initial_heading;
    Eigen::Quaterniond orientation(Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()));
    reference.pose.orientation.x = orientation.x();
    reference.pose.orientation.y = orientation.y();
    reference.pose.orientation.z = orientation.z();
    reference.pose.orientation.w = orientation.w();
    return reference;
}

const std::array<float, 3> HelixReferences::computeCircleMiddle(
    const std::array<float, 3> &begin_position,
    const double                initial_heading) const
{
    // Compute the middle point of the circle based on the heading of the drone: ahead, or to the
    // right with the nose along the path.
    const double         base = circleBase(initial_heading);
    std::array<float, 3> middle_point;
    middle_point[AxisIndex::kXAxis] =
        begin_position[AxisIndex::kXAxis] + std::cos(base) * circle_radius_m_; // x
    middle_point[AxisIndex::kYAxis] =
        begin_position[AxisIndex::kYAxis] + std::sin(base) * circle_radius_m_; // y
    middle_point[AxisIndex::kZAxis] = begin_position[AxisIndex::kZAxis];                  // z
    return middle_point;
}

double HelixReferences::circleBase(const double initial_heading) const
{
    // The circle starts sideways, to the left of the line to its middle. With the nose along the
    // path, turn it a quarter so that it starts straight ahead.
    return frontal_ ? initial_heading - std::numbers::pi / 2.0 : initial_heading;
}
