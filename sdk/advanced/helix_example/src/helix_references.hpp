// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//

#pragma once

#include <creos/messages/state_reference.hpp>
#include <numbers>

class HelixReferences
{
public:
    enum AxisIndex
    {
        kXAxis = 0,
        kYAxis = 1,
        kZAxis = 2
    };

    HelixReferences(double update_frequency_hz,
                    double circle_radius_m_ = 2,
                    double speed_mps        = 0.5,
                    double accel_mps2       = 1.0,
                    double climb_m          = 1.0,
                    double turns            = 2.0,
                    bool   frontal          = false);
    ~HelixReferences() = default;

    void Reset(const std::array<float, 3> position, const double yaw_heading);
    // Returns the next setpoint. The drone position is used to wait for the drone when it
    // falls behind the reference.
    creos_messages::StateReference GetNewStateReference(
        const std::array<float, 3> &drone_position);

private:
    // Below this distance between the reference and the drone, the reference moves at full
    // speed. Above the stop distance it waits for the drone; slower in between.
    static constexpr double kFullSpeedErrorM = 0.3;
    static constexpr double kStopErrorM      = 1.0;
    // How fast the time scale may change, per second (from 1 to 0 in 0.5 s).
    static constexpr double kTimeScaleRate = 2.0;

    const double time_step_s_;
    const double circle_radius_m_;
    const double speed_mps_;
    const double accel_mps2_;
    const double climb_m_;
    const double turns_;
    const bool   frontal_;

    double               time_scale_          = 1.0; // 1 = full speed, 0 = waiting for the drone
    double               time_s_              = 0.0;
    float                initial_heading_     = 0.0f;
    std::array<float, 3> middle_point_circle_ = {0.0f, 0.0f, 0.0f};

    // Helper functions
    void updateTimeScale(const creos_messages::StateReference &reference,
                         const std::array<float, 3>           &drone_position);
    const creos_messages::StateReference computeNewPosition(
        const std::array<float, 3> &circle_middle,
        const double                initial_heading,
        const double                time_s) const;
    double                     circleBase(const double initial_heading) const;
    const std::array<float, 3> computeCircleMiddle(const std::array<float, 3> &begin_position,
                                                   const double initial_heading) const;
};
