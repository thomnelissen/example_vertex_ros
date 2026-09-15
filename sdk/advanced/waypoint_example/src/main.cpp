// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//
/*****************************************************************************
 * Example of how to use the CreOS SDK to automatically take off and fly
 * a waypoint mission. This example uses the Job API to define a Job.
 ****************************************************************************/
#include <CLI11.hpp>
#include <iostream>
#include <optional>
#include <array>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <unistd.h>
#include <limits>
#include <stdexcept>


#include <creos/client.hpp>
#include <creos/autopilot/navigation.hpp>
#include <creos/job/job.hpp>
#include <creos/job/wait.hpp>
#include <creos/job/waypoint.hpp>
#include <creos/messages/controller_state.hpp>
#include <creos/messages/pose.hpp>

#include <common/logging.hpp>
#include <common/drone_state.hpp>
#include <common/remote_controller_interface.hpp>
#include <common/flight_controller.hpp>

enum class JobState
{
    kStartJob,
    kJobRunning,
};

struct GpsJobConfig
{
    double lat1 = 0.0;
    double lon1 = 0.0;
    double alt1 = 10.0;

    double lat2 = 0.0;
    double lon2 = 0.0;
    double alt2 = 10.0;
};

GpsJobConfig gps_job_config;

struct HugeLtpJobConfig
{
    // These values are offsets from the current local/LTP position.
    // Defaults keep the original huge-coordinate stress-test behavior.
    double x_offset = 9990000.0;
    double y_offset = 1100000.0;
    double z_offset = 10.0;

    // Safe climb before sending the stress waypoint.
    double safe_z_offset = 10.0;

    double safe_climb_speed = 2.0;
    double target_speed     = 3.0;
};

HugeLtpJobConfig huge_ltp_job_config;

void spin(unsigned update_frequency_hz)
{
    usleep(1000000 / update_frequency_hz);
}

void validateGpsJobInputs()
{
    if(gps_job_config.lat1 == 0.0 || gps_job_config.lon1 == 0.0 || gps_job_config.lat2 == 0.0 ||
       gps_job_config.lon2 == 0.0)
    {
        throw std::runtime_error(
            "GPS jobs require --lat1 --lon1 --alt1 --lat2 --lon2 --alt2 arguments.");
    }
}


creos::Client createClient(const std::string_view host)
{
    if(host.empty())
    {
        return creos::Client();
    }
    return creos::Client(host);
}

creos::Job createJob1(creos::Client &client, const std::array<float, 3> current_position)
{
    // Create a new job with a unique identifier
    creos::Job job(client, "example-job-001");

    //! [job_creation]
    // Set maximum speed for the entire job. This can be overridden for individual waypoints.
    // If a max speed is not set a value of 1 m/s is used by default.
    job.setMaxSpeed(4.0);

    //! [takeoff_task]
    // Add a takeoff task at the beginning of the mission.
    // Take of height will be the configured takeoff height in the settings.
    job.addTakeOff(creos::TakeOff());
    //! [takeoff_task]

    //! [waypoint_basic]
    // Add a waypoint with custom navigation settings using fluent interface
    // Coordinates are in the robot's local coordinate frame (meters)
    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromLtp(current_position[0], current_position[1] + 2.0,
                                                   current_position[2] + 10.0))
            .setHeading(90.0)                                        // Face east (90 degrees)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth)); // Heading relative to north

    //! [waypoint_basic]
    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromLtp(current_position[0], current_position[1] + 10.0,
                                                   current_position[2] + 10.0))
            .setHeading(0.0)                                       // Face north (0 degrees)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth) // Heading relative to north
            .setMaxSpeed(1.0));                                    // Slow down for this waypoint

    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromLtp(current_position[0], current_position[1] + 10.0,
                                                   current_position[2] + 14.0))
            .setHeading(-90.0)                                       // Face west (-90 degrees)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth)); // Heading relative to north

    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromLtp(current_position[0], current_position[1] + 2.0,
                                                   current_position[2] + 14.0))
            .setHeading(0.0)                                         // Face north (0 degrees)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth)); // Heading relative to north

    //! [waypoint_return]
    // Add a final waypoint to return to start
    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromLtp(current_position[0], current_position[1] + 2.0,
                                                   current_position[2] + 10.0))
            .setHeading(180.0)                                       // Face south (180 degrees)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth)); // Heading relative to north
    //! [waypoint_return]

    //! [land_task]
    // Add a land task to finish the mission.
    job.addLand(creos::Land());
    //! [land_task]

    // Print job description before starting
    spdlog::info("{}", job.describe());

    return job;
}

// -----------------------------------------------------------------------------
// Job 2 - Near-ground aggressive horizontal movement
// Tests low-altitude lateral stability close to ground.
// -----------------------------------------------------------------------------
creos::Job createJob2(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "beta-job-002-near-ground-aggressive-horizontal");

    job.setMaxSpeed(3.0);
    job.addTakeOff(creos::TakeOff());

    const double x0 = current_position[0];
    const double y0 = current_position[1];
    const double z0 = current_position[2];

    const double low_z = z0 + 2.0;

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0, y0, low_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.8));

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0 + 8.0, y0, low_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(3.0));

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0 + 8.0, y0 + 8.0, low_z))
                        .setHeading(90.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(3.0));

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0 - 4.0, y0 - 4.0, low_z))
                        .setHeading(-90.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(3.0));

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0, y0, low_z))
                        .setHeading(180.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(2.0));

    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}


// -----------------------------------------------------------------------------
// Job 3 - Dense heading changes during movement
// Tests yaw + XY coupling while the drone is continuously moving.
// -----------------------------------------------------------------------------
creos::Job createJob3(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "beta-job-003-dense-heading-changes-moving");

    job.setMaxSpeed(1.5);
    job.addTakeOff(creos::TakeOff());

    const double x0        = current_position[0];
    const double y0        = current_position[1];
    const double z0        = current_position[2];
    const double mission_z = z0 + 10.0;

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0, y0, mission_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(1.0));

    const std::array<double, 12> headings = {0.0,   180.0, -90.0, 90.0,   45.0, -135.0,
                                             135.0, -45.0, 170.0, -170.0, 30.0, -150.0};

    for(int i = 0; i < 60; ++i)
    {
        const double x       = x0 + (0.7 * i);
        const double y       = y0 + ((i % 2 == 0) ? 2.0 : -2.0);
        const double heading = headings[i % headings.size()];

        job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x, y, mission_z))
                            .setHeading(heading)
                            .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                            .setMaxSpeed(1.2));
    }

    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}

// -----------------------------------------------------------------------------
// Job 4 - Set max Speed with negative value
// Simple back-and-forth movement to verify max speed behavior
// -----------------------------------------------------------------------------
creos::Job createJob4(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "example-job-004-max-speed");

    // High speed to test behavior
    job.setMaxSpeed(10.0);

    job.addTakeOff(creos::TakeOff());

    const double x0        = current_position[0];
    const double y0        = current_position[1];
    const double z0        = current_position[2];
    const double mission_z = z0 + 10.0;

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0, y0, mission_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.5));

    // Move right 25m
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0, y0 + 25.0, mission_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth));

    // Move left  25m
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0, y0 - 25.0, mission_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(-1.0));

    // Return to start
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0, y0, mission_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth));

    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}

// -----------------------------------------------------------------------------
// Job 5 - Invalid mission after valid mission
// Run a normal valid job first, then run this invalid job immediately after.
// Expected: invalid mission must be rejected cleanly and must not corrupt mission state.
// -----------------------------------------------------------------------------
creos::Job createJob5(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "beta-job-005-invalid-after-valid-mission");

    job.setMaxSpeed(1.0);
    job.addTakeOff(creos::TakeOff());

    const double x0        = current_position[0];
    const double y0        = current_position[1];
    const double z0        = current_position[2];
    const double mission_z = z0 + 10.0;

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0, y0, mission_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(1.0));

    auto invalid_wp = creos::Waypoint(creos::Coordinate::FromLtp(x0 + 5.0, y0, mission_z));

    invalid_wp.setHeading(std::numeric_limits<double>::quiet_NaN());
    invalid_wp.setHeadingBehavior(creos::HeadingBehavior::FromNorth);
    invalid_wp.setMaxSpeed(1.0);

    job.addWaypoint(std::move(invalid_wp));
    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}


// -----------------------------------------------------------------------------
// Job 8 - Takeoff task twice
// Tests task-state handling when TakeOff is requested while already flying.
// Expected: second takeoff should be rejected, ignored safely, or handled without crash.
// -----------------------------------------------------------------------------
creos::Job createJob8(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "beta-job-008-double-takeoff-task");

    job.setMaxSpeed(1.0);

    job.addTakeOff(creos::TakeOff());
    job.addTakeOff(creos::TakeOff()); // intentional edge case

    const double x0        = current_position[0];
    const double y0        = current_position[1];
    const double z0        = current_position[2];
    const double mission_z = z0 + 5.0;

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0 + 3.0, y0, mission_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.8));

    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}

// -----------------------------------------------------------------------------
// Job 9 - Takeoff task twice between waypoint
// Tests task-state handling when TakeOff is requested while already flying.
// Expected: second takeoff should be rejected, ignored safely, or handled without crash.
// -----------------------------------------------------------------------------
creos::Job createJob9(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "beta-job-009-double-takeoff-task");

    job.setMaxSpeed(1.0);

    job.addTakeOff(creos::TakeOff());


    const double x0        = current_position[0];
    const double y0        = current_position[1];
    const double z0        = current_position[2];
    const double mission_z = z0 + 10.0;

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0 + 3.0, y0, mission_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.8));

    job.addTakeOff(creos::TakeOff()); // intentional edge case

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0 + 8.0, y0, mission_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.8));

    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}

// -----------------------------------------------------------------------------
// Job 10 - Actual GPS/WGS84 coordinate waypoint test
// Takes off from the current ground position, first flies to a safe LTP waypoint,
// then flies to two defined WGS84 GPS waypoints, then lands.
// -----------------------------------------------------------------------------
creos::Job createJob10(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "beta-job-010-actual-gps-coordinate-test");

    job.setMaxSpeed(1.0);

    // Take off from the current physical position.
    job.addTakeOff(creos::TakeOff());

    // First GPS waypoint inside the test area.
    const double lat1 = 51.320649599999996;
    const double lon1 = 5.1765292;
    const double alt1 = 30.0;

    // Second GPS waypoint inside the test area.
    const double lat2 = 51.320814999999996;
    const double lon2 = 5.1764603;
    const double alt2 = 30.0;

    const double x0        = current_position[0];
    const double y0        = current_position[1];
    const double z0        = current_position[2];
    const double mission_z = z0 + 10.0;

    // First fly to a safe local/LTP altitude before going to GPS waypoints.
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0, y0, mission_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(1.0));

    // Then fly to the first WGS84 GPS waypoint.
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromWgs84(lat1, lon1, alt1))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(1.0));

    // Then fly to the second WGS84 GPS waypoint.
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromWgs84(lat2, lon2, alt2))
                        .setHeading(90.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(1.0));

    // Land after reaching the second GPS waypoint.
    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}


// -----------------------------------------------------------------------------
// Job 11 - Parameterized WGS84/GPS coordinate waypoint test
// GPS coordinates and altitude are provided from terminal arguments.
// Example:
// ./waypoint_example --job 11 \
//   --lat1 51.321361 --lon1 5.175306 --alt1 10 \
//   --lat2 51.321500 --lon2 5.175450 --alt2 10
// -----------------------------------------------------------------------------
creos::Job createJob11(creos::Client &client, const std::array<float, 3> /*current_position*/)
{
    creos::Job job(client, "beta-job-011-parameterized-wgs84-gps-test");

    // Safety check: prevent accidentally running Job 11 with default 0.0 coordinates.
    // If lat/lon arguments are not provided, the job would otherwise target (0,0),
    // which is not a valid/safe test location for this scenario.
    if(gps_job_config.lat1 == 0.0 || gps_job_config.lon1 == 0.0 || gps_job_config.lat2 == 0.0 ||
       gps_job_config.lon2 == 0.0)
    {
        throw std::runtime_error(
            "Job 11 requires --lat1 --lon1 --alt1 --lat2 --lon2 --alt2 arguments.");
    }

    job.setMaxSpeed(1.0);

    job.addTakeOff(creos::TakeOff());

    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromWgs84(gps_job_config.lat1, gps_job_config.lon1,
                                                     gps_job_config.alt1))
            .setHeading(0.0)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
            .setMaxSpeed(0.8));

    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromWgs84(gps_job_config.lat2, gps_job_config.lon2,
                                                     gps_job_config.alt2))
            .setHeading(90.0)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
            .setMaxSpeed(0.8));

    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}


// -----------------------------------------------------------------------------
// Job 12 - Intersecting geofence path
// Both waypoints inside/near valid areas, but the straight segment should
// cross the geofence boundary
// Expected: system should not allow path segment to breach geofence.
// -----------------------------------------------------------------------------
creos::Job createJob12(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "beta-job-012-intersecting-geofence-path");

    job.setMaxSpeed(1.0);
    job.addTakeOff(creos::TakeOff());

    const double x0        = current_position[0];
    const double y0        = current_position[1];
    const double z0        = current_position[2];
    const double mission_z = z0 + 10.0;

    // Adjust these based on actual geofence geometry.
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0, y0 + 45.0, mission_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.8));

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0, y0 - 45.0, mission_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.8));

    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}

// -----------------------------------------------------------------------------
// Job 13 - Geofence definition outside / invalid geofence config
// Use with an intentionally wrong geofence configuration.
// Expected: mission start should be rejected, or system should report geofence config error.
// -----------------------------------------------------------------------------
creos::Job createJob13(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "beta-job-013-invalid-geofence-definition");

    job.setMaxSpeed(1.0);
    job.addTakeOff(creos::TakeOff());

    const double x0        = current_position[0];
    const double y0        = current_position[1];
    const double z0        = current_position[2];
    const double mission_z = z0 + 10.0;

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0, y0, mission_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.8));

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0 + 5.0, y0, mission_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.8));

    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}

// -----------------------------------------------------------------------------
// Job 15 - WGS84 invalid latitude
// Latitude valid range is -90 to +90 degrees.
// This intentionally uses 91.0 and should be rejected safely.
// -----------------------------------------------------------------------------
creos::Job createJob15(creos::Client &client, const std::array<float, 3> /*current_position*/)
{
    creos::Job job(client, "beta-job-015-wgs84-invalid-latitude");

    job.setMaxSpeed(1.0);
    job.addTakeOff(creos::TakeOff());

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromWgs84(91.0, 5.161504, 90.0))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.8));

    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}


// -----------------------------------------------------------------------------
// Job 16 - WGS84 invalid longitude
// Longitude valid range is -180 to +180 degrees.
// This intentionally uses 181.0 and should be rejected safely.
// -----------------------------------------------------------------------------
creos::Job createJob16(creos::Client &client, const std::array<float, 3> /*current_position*/)
{
    creos::Job job(client, "beta-job-016-wgs84-invalid-longitude");

    job.setMaxSpeed(1.0);
    job.addTakeOff(creos::TakeOff());

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromWgs84(51.311748, 181.0, 90.0))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.8));

    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}

// -----------------------------------------------------------------------------
// Job 24 - Same waypoint 200 times with changing heading
// Tests whether same position + different heading is skipped or executed.
// -----------------------------------------------------------------------------
creos::Job createJob24(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "beta-job-024-same-waypoint-changing-heading");

    job.setMaxSpeed(1.0);
    job.addTakeOff(creos::TakeOff());

    const double x0        = current_position[0];
    const double y0        = current_position[1];
    const double z0        = current_position[2];
    const double mission_z = z0 + 10.0;

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0, y0, mission_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(1.0));

    for(int i = 0; i < 200; ++i)
    {
        double heading = static_cast<double>((i * 37) % 360);

        if(heading > 180.0)
        {
            heading -= 360.0;
        }

        job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0 + 5.0, y0 + 5.0, mission_z))
                            .setHeading(heading)
                            .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                            .setMaxSpeed(0.8));
    }

    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}


// -----------------------------------------------------------------------------
// Job 30 - Parameterized huge LTP coordinate command
// Intentionally commands a configurable local/LTP coordinate offset.
// Defaults keep the original huge-coordinate stress-test behavior.
//
// Example:
// ./waypoint_example --job 30 \
//   --job30-x-offset 9990000 --job30-y-offset 1100000 \
//   --job30-z-offset 10 --job30-target-speed 3
// -----------------------------------------------------------------------------
creos::Job createJob30(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "beta-job-030-parameterized-huge-ltp-coordinate");

    job.setMaxSpeed(huge_ltp_job_config.target_speed);
    job.addTakeOff(creos::TakeOff());

    const double x0 = current_position[0];
    const double y0 = current_position[1];
    const double z0 = current_position[2];

    const double safe_z   = z0 + huge_ltp_job_config.safe_z_offset;
    const double target_x = x0 + huge_ltp_job_config.x_offset;
    const double target_y = y0 + huge_ltp_job_config.y_offset;
    const double target_z = z0 + huge_ltp_job_config.z_offset;

    // Safe climb first, before sending the parameterized stress waypoint.
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0, y0, safe_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(huge_ltp_job_config.safe_climb_speed));

    // Parameterized LTP target. Use large values to stress validation/overflow/geofence logic,
    // or smaller values for controlled retesting without rebuilding the code.
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(target_x, target_y, target_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(huge_ltp_job_config.target_speed));

    job.addLand(creos::Land());

    spdlog::info("Job 30 parameters: x_offset={}, y_offset={}, z_offset={}, safe_z_offset={}, "
                 "safe_climb_speed={}, target_speed={}",
                 huge_ltp_job_config.x_offset, huge_ltp_job_config.y_offset,
                 huge_ltp_job_config.z_offset, huge_ltp_job_config.safe_z_offset,
                 huge_ltp_job_config.safe_climb_speed, huge_ltp_job_config.target_speed);
    spdlog::info("{}", job.describe());
    return job;
}


// -----------------------------------------------------------------------------
// Job 31 - Below-ground altitude command
// Intentionally commands a waypoint below the captured start altitude.
// Expected: system should reject, clamp, fail safely, or avoid unsafe descent.
// -----------------------------------------------------------------------------
creos::Job createJob31(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "beta-job-031-below-ground-altitude");

    job.setMaxSpeed(0.8);
    job.addTakeOff(creos::TakeOff());

    const double x0        = current_position[0];
    const double y0        = current_position[1];
    const double z0        = current_position[2];
    const double mission_z = z0 + 10.0;

    // Safe climb first.
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0, y0, mission_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.8));

    // Intentionally unsafe/below-start altitude command.
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0 + 3.0, y0, z0 - 1.0))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.3));

    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}

// -----------------------------------------------------------------------------
// Job 32 - Mixed LTP + WGS84 mission
// First waypoint uses LTP, second waypoint uses WGS84, final waypoint uses LTP.
// Tests whether coordinate-frame switching inside one mission is handled correctly.
// Example:
// ./waypoint_example --job 32 --lat1 51.321361 --lon1 5.175306 --alt1 10 --lat2 51.321500
// --lon2 5.175450 --alt2 10
// -----------------------------------------------------------------------------
creos::Job createJob32(creos::Client &client, const std::array<float, 3> current_position)
{
    validateGpsJobInputs();

    creos::Job job(client, "beta-job-032-mixed-ltp-wgs84-mission");

    job.setMaxSpeed(1.0);
    job.addTakeOff(creos::TakeOff());

    const double x0        = current_position[0];
    const double y0        = current_position[1];
    const double z0        = current_position[2];
    const double mission_z = z0 + 10.0;

    // LTP waypoint first.
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0 + 5.0, y0, mission_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.8));

    // WGS84 waypoint second.
    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromWgs84(gps_job_config.lat1, gps_job_config.lon1,
                                                     gps_job_config.alt1))
            .setHeading(90.0)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
            .setMaxSpeed(0.8));

    // LTP waypoint again.
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0, y0 + 5.0, mission_z))
                        .setHeading(180.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.8));

    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}


// -----------------------------------------------------------------------------
// Job 33 - WGS84 same lat/lon, different altitude
// Tests altitude-only behavior using WGS84 coordinates.
// Same GPS position is used twice, but altitude changes.
// Example:
// ./waypoint_example --job 33 --lat1 51.321361 --lon1 5.175306 --alt1 10 --lat2 51.321361
// --lon2 5.175306 --alt2 20
// -----------------------------------------------------------------------------
creos::Job createJob33(creos::Client &client, const std::array<float, 3> /*current_position*/)
{
    validateGpsJobInputs();

    creos::Job job(client, "beta-job-033-wgs84-same-position-different-altitude");

    job.setMaxSpeed(0.8);
    job.addTakeOff(creos::TakeOff());

    // First WGS84 waypoint.
    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromWgs84(gps_job_config.lat1, gps_job_config.lon1,
                                                     gps_job_config.alt1))
            .setHeading(0.0)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
            .setMaxSpeed(0.6));

    // Same lat/lon but different altitude.
    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromWgs84(gps_job_config.lat1, gps_job_config.lon1,
                                                     gps_job_config.alt2))
            .setHeading(0.0)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
            .setMaxSpeed(0.6));

    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}


// -----------------------------------------------------------------------------
// Job 34 - LTP first, WGS84 return-to-start
// Drone first flies using LTP, then returns using WGS84 coordinate.
// Use lat1/lon1/alt1 as the GPS coordinate close to the start/takeoff location.
// Tests whether LTP -> WGS84 transition and return behavior are consistent.
// Example:
// ./waypoint_example --job 34 --lat1 <start_lat> --lon1 <start_lon> --alt1 10 --lat2 <unused_lat>
// --lon2 <unused_lon> --alt2 10
// -----------------------------------------------------------------------------
creos::Job createJob34(creos::Client &client, const std::array<float, 3> current_position)
{
    validateGpsJobInputs();

    creos::Job job(client, "beta-job-034-ltp-first-wgs84-return");

    job.setMaxSpeed(1.0);
    job.addTakeOff(creos::TakeOff());

    const double x0        = current_position[0];
    const double y0        = current_position[1];
    const double z0        = current_position[2];
    const double mission_z = z0 + 10.0;

    // Leave start area using LTP.
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0 + 10.0, y0, mission_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.8));

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0 + 10.0, y0 + 8.0, mission_z))
                        .setHeading(90.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.8));

    // Return using WGS84 coordinate close to start/takeoff location.
    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromWgs84(gps_job_config.lat1, gps_job_config.lon1,
                                                     gps_job_config.alt1))
            .setHeading(180.0)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
            .setMaxSpeed(0.8));

    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}


// -----------------------------------------------------------------------------
// Job 35 - Mission starts with WGS84 waypoint equal to current GPS
// Use lat1/lon1/alt1 as the drone's current GPS/global pose before mission start.
// Tests whether already-at-waypoint behavior is handled cleanly.
// Example:
// ./waypoint_example --job 35 --lat1 <current_lat> --lon1 <current_lon> --alt1 10 --lat2
// <nearby_lat> --lon2 <nearby_lon> --alt2 10
// -----------------------------------------------------------------------------
creos::Job createJob35(creos::Client &client, const std::array<float, 3> /*current_position*/)
{
    validateGpsJobInputs();

    creos::Job job(client, "beta-job-035-wgs84-current-position-first");

    job.setMaxSpeed(0.8);
    job.addTakeOff(creos::TakeOff());

    // First waypoint should be equal or very close to current GPS position.
    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromWgs84(gps_job_config.lat1, gps_job_config.lon1,
                                                     gps_job_config.alt1))
            .setHeading(0.0)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
            .setMaxSpeed(0.5));

    // Second waypoint should be a nearby valid GPS point.
    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromWgs84(gps_job_config.lat2, gps_job_config.lon2,
                                                     gps_job_config.alt2))
            .setHeading(90.0)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
            .setMaxSpeed(0.5));

    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}


// -----------------------------------------------------------------------------
// Job 36 - Land-only job while already airborne
// Start this job only when the drone is already airborne/hovering.
// No TakeOff and no Waypoint are added on purpose.
// Tests whether Land-only task is accepted and handled safely.
// -----------------------------------------------------------------------------
creos::Job createJob36(creos::Client &client, const std::array<float, 3> /*current_position*/)
{
    creos::Job job(client, "beta-job-036-land-only-while-airborne");

    job.setMaxSpeed(0.5);

    // Intentionally no TakeOff and no Waypoint.
    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}


// -----------------------------------------------------------------------------
// Job 37 - Waypoint-only job while already airborne
// Start this job only when the drone is already airborne/hovering.
// No TakeOff and no Land are added on purpose.
// Tests whether SDK-auto can execute waypoints from airborne state cleanly.
// -----------------------------------------------------------------------------
creos::Job createJob37(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "beta-job-037-waypoint-only-while-airborne");

    job.setMaxSpeed(1.0);

    const double x0 = current_position[0];
    const double y0 = current_position[1];
    const double z0 = current_position[2];

    // Use current airborne altitude.
    const double current_z = z0;

    // Intentionally no TakeOff.
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0 + 5.0, y0, current_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.8));

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0 + 5.0, y0 + 5.0, current_z))
                        .setHeading(90.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.8));

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0, y0, current_z))
                        .setHeading(180.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.8));

    // Intentionally no Land.

    spdlog::info("{}", job.describe());
    return job;
}

// -----------------------------------------------------------------------------
// Job 38 - Waypoint-only mission while on ground
// No TakeOff task is added intentionally.
// Tests whether the system rejects or safely handles waypoint execution while
// the drone is still on the ground.
// Expected: mission should be rejected or should not cause unsafe movement.
// -----------------------------------------------------------------------------
creos::Job createJob38(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "beta-job-038-waypoint-only-while-on-ground");

    job.setMaxSpeed(0.8);

    const double x0 = current_position[0];
    const double y0 = current_position[1];
    const double z0 = current_position[2];

    // Intentionally no TakeOff task.
    // Command a waypoint above and slightly forward from current ground position.
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0 + 3.0, y0, z0 + 5.0))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.5));

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0 + 6.0, y0, z0 + 5.0))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.5));

    // Intentionally no Land task because the drone should not have taken off.

    spdlog::info("{}", job.describe());
    return job;
}


// -----------------------------------------------------------------------------
// Job 39 - Land-only mission while on ground
// No TakeOff and no Waypoint tasks are added intentionally.
// Tests whether a Land task while already on the ground completes safely,
// is rejected safely, or causes a stuck/invalid job state.
// Expected: no unsafe behavior, no crash, no stuck mission state.
// -----------------------------------------------------------------------------
creos::Job createJob39(creos::Client &client, const std::array<float, 3> /*current_position*/)
{
    creos::Job job(client, "beta-job-039-land-only-while-on-ground");

    job.setMaxSpeed(0.5);

    // Intentionally no TakeOff and no Waypoint.
    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}


// -----------------------------------------------------------------------------
// Job 40 - TakeOff -> Land -> Waypoint
// Invalid/strange task order test.
// Tests whether the system safely rejects, ignores, or prevents waypoint execution
// after landing has already been requested.
// Expected: after Land, the later waypoint should not cause unsafe takeoff/movement
// or mission-state corruption.
// -----------------------------------------------------------------------------
creos::Job createJob40(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "beta-job-040-takeoff-land-then-waypoint");

    job.setMaxSpeed(0.8);

    const double x0 = current_position[0];
    const double y0 = current_position[1];
    const double z0 = current_position[2];

    // Normal takeoff.
    job.addTakeOff(creos::TakeOff());

    // Immediately request landing.
    job.addLand(creos::Land());

    // Intentionally add a waypoint after Land.
    // This is an invalid/edge task order and should be handled safely.
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0 + 5.0, y0, z0 + 5.0))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.5));

    spdlog::info("{}", job.describe());
    return job;
}

// -----------------------------------------------------------------------------
// Job 41 - Empty job after valid job
// Run a normal valid job first, then run this empty job immediately after.
// Tests whether the empty-job fix also handles scheduler/state cleanup correctly
// after a previously valid mission.
// Expected: empty job should be rejected, completed, or handled safely without
// runtime crash, stuck mission state, or stale waypoint execution.
// -----------------------------------------------------------------------------
creos::Job createJob41(creos::Client &client, const std::array<float, 3> /*current_position*/)
{
    creos::Job job(client, "beta-job-041-empty-job-after-valid-job");

    // Intentionally no TakeOff, no Waypoint, and no Land.
    // This tests empty job handling after a valid mission was executed before.

    spdlog::info("{}", job.describe());
    return job;
}


// -----------------------------------------------------------------------------
// Job 42 - TakeOff -> invalid waypoint only -> Land
// Minimal chain with one intentionally invalid waypoint between TakeOff and Land.
// Tests whether invalid input is rejected safely in a minimal otherwise-valid task chain.
// Expected: invalid waypoint should be rejected safely before unsafe movement,
// without backend crash or mission-state corruption.
// -----------------------------------------------------------------------------
creos::Job createJob42(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "beta-job-042-takeoff-invalid-waypoint-land");

    job.setMaxSpeed(1.0);
    job.addTakeOff(creos::TakeOff());

    const double x0        = current_position[0];
    const double y0        = current_position[1];
    const double z0        = current_position[2];
    const double mission_z = z0 + 10.0;

    auto invalid_wp = creos::Waypoint(creos::Coordinate::FromLtp(x0 + 5.0, y0, mission_z));

    // Intentionally invalid heading.
    invalid_wp.setHeading(std::numeric_limits<double>::quiet_NaN());
    invalid_wp.setHeadingBehavior(creos::HeadingBehavior::FromNorth);
    invalid_wp.setMaxSpeed(0.8);

    job.addWaypoint(std::move(invalid_wp));

    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}


// -----------------------------------------------------------------------------
// Job 43 - Invalid waypoint only, no TakeOff/Land
// Tests validation behavior when the job contains only an invalid waypoint,
// without any safety-wrapper tasks.
// Expected: invalid waypoint should be rejected safely without takeoff,
// movement, runtime crash, or stuck mission state.
// -----------------------------------------------------------------------------
creos::Job createJob43(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "beta-job-043-invalid-waypoint-only");

    job.setMaxSpeed(1.0);

    const double x0 = current_position[0];
    const double y0 = current_position[1];
    const double z0 = current_position[2];

    auto invalid_wp = creos::Waypoint(creos::Coordinate::FromLtp(x0 + 5.0, y0, z0 + 10.0));

    // Intentionally invalid heading.
    invalid_wp.setHeading(std::numeric_limits<double>::quiet_NaN());
    invalid_wp.setHeadingBehavior(creos::HeadingBehavior::FromNorth);
    invalid_wp.setMaxSpeed(0.8);

    // Intentionally no TakeOff and no Land.
    job.addWaypoint(std::move(invalid_wp));

    spdlog::info("{}", job.describe());
    return job;
}

// -----------------------------------------------------------------------------
// Job 44 - Job max speed = infinity
// Tests whether an infinite job-level max speed is rejected safely.
// Expected: mission should be rejected, clamped, or handled safely without
// unsafe movement, backend crash, or mission-state corruption.
// -----------------------------------------------------------------------------
creos::Job createJob44(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "beta-job-044-job-max-speed-infinity");

    job.setMaxSpeed(std::numeric_limits<double>::infinity());

    job.addTakeOff(creos::TakeOff());

    const double x0        = current_position[0];
    const double y0        = current_position[1];
    const double z0        = current_position[2];
    const double mission_z = z0 + 10.0;

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0 + 5.0, y0, mission_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.8));

    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}


// -----------------------------------------------------------------------------
// Job 45 - Heading slightly outside expected range
// Tests whether headings slightly outside the typical [-180, 180] range are
// normalized, rejected, or handled safely.
// Expected: system should handle 181 and -181 consistently without yaw instability.
// -----------------------------------------------------------------------------
creos::Job createJob45(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "beta-job-045-heading-outside-range");

    job.setMaxSpeed(1.0);
    job.addTakeOff(creos::TakeOff());

    const double x0        = current_position[0];
    const double y0        = current_position[1];
    const double z0        = current_position[2];
    const double mission_z = z0 + 10.0;

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0, y0, mission_z))
                        .setHeading(181.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.8));

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0 + 5.0, y0, mission_z))
                        .setHeading(-181.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.8));

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0 + 10.0, y0, mission_z))
                        .setHeading(181.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.8));

    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}


// -----------------------------------------------------------------------------
// Job 46 - Waypoint speed = 0.0
// Tests whether zero waypoint speed is rejected, clamped, or causes stuck mission.
// Expected: system should not hang indefinitely or execute unsafe behavior.
// -----------------------------------------------------------------------------
creos::Job createJob46(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "beta-job-046-waypoint-speed-zero");

    job.setMaxSpeed(1.0);
    job.addTakeOff(creos::TakeOff());

    const double x0        = current_position[0];
    const double y0        = current_position[1];
    const double z0        = current_position[2];
    const double mission_z = z0 + 10.0;

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0, y0, mission_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.8));

    // Intentionally zero waypoint speed.
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0 + 5.0, y0, mission_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.0));

    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}


// -----------------------------------------------------------------------------
// Job 47 - Waypoint speed = NaN
// Tests whether NaN waypoint speed is rejected safely.
// Expected: mission should be rejected safely before unsafe movement.
// -----------------------------------------------------------------------------
creos::Job createJob47(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "beta-job-047-waypoint-speed-nan");

    job.setMaxSpeed(1.0);
    job.addTakeOff(creos::TakeOff());

    const double x0        = current_position[0];
    const double y0        = current_position[1];
    const double z0        = current_position[2];
    const double mission_z = z0 + 10.0;

    auto invalid_wp = creos::Waypoint(creos::Coordinate::FromLtp(x0 + 5.0, y0, mission_z));

    invalid_wp.setHeading(0.0);
    invalid_wp.setHeadingBehavior(creos::HeadingBehavior::FromNorth);
    invalid_wp.setMaxSpeed(std::numeric_limits<double>::quiet_NaN());

    job.addWaypoint(std::move(invalid_wp));

    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}


// -----------------------------------------------------------------------------
// Job 48 - Waypoint with negative infinity coordinate
// Tests whether negative infinity in LTP coordinate is rejected safely.
// Expected: mission should be rejected safely without unsafe movement or crash.
// -----------------------------------------------------------------------------
creos::Job createJob48(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "beta-job-048-negative-infinity-coordinate");

    job.setMaxSpeed(1.0);
    job.addTakeOff(creos::TakeOff());

    const double x0        = current_position[0];
    const double y0        = current_position[1];
    const double z0        = current_position[2];
    const double mission_z = z0 + 10.0;

    // Safe first waypoint.
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0, y0, mission_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.8));

    // Intentionally invalid coordinate.
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(
                                        -std::numeric_limits<double>::infinity(), y0, mission_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.8));

    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}


// -----------------------------------------------------------------------------
// Job 49 - Job max speed = 0.0
// Tests whether zero job-level max speed is rejected, clamped, or causes stuck mission.
// Expected: system should not hang indefinitely or execute unsafe behavior.
// -----------------------------------------------------------------------------
creos::Job createJob49(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "beta-job-049-job-max-speed-zero");

    job.setMaxSpeed(0.0);

    job.addTakeOff(creos::TakeOff());

    const double x0        = current_position[0];
    const double y0        = current_position[1];
    const double z0        = current_position[2];
    const double mission_z = z0 + 10.0;

    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(x0 + 5.0, y0, mission_z))
                        .setHeading(0.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(0.8));

    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());
    return job;
}

creos::Job createJob101(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "example-job-001");

    // Global default speed (overridden per waypoint)
    job.setMaxSpeed(4.0);

    // Takeoff
    job.addTakeOff(creos::TakeOff());

    constexpr float altitude = 10.0f;
    constexpr float heading  = 90.0f;

    // WP1 - 4 m/s
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + 20.0,
                                                               current_position[1] + 0.0,
                                                               current_position[2] + altitude))
                        .setHeading(heading)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(4.0));

    // WP2 - 6 m/s
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + 40.0,
                                                               current_position[1] + 20.0,
                                                               current_position[2] + altitude))
                        .setHeading(heading)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(6.0));

    // WP3 - 8 m/s
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + 60.0,
                                                               current_position[1] + 0.0,
                                                               current_position[2] + altitude))
                        .setHeading(heading)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(8.0));

    // WP4 - 10 m/s
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + 80.0,
                                                               current_position[1] + 20.0,
                                                               current_position[2] + altitude))
                        .setHeading(heading)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(10.0));

    // WP5 - 12 m/s
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + 100.0,
                                                               current_position[1] + 0.0,
                                                               current_position[2] + altitude))
                        .setHeading(heading)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(12.0));

    // WP6 - 14 m/s
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + 120.0,
                                                               current_position[1] + 20.0,
                                                               current_position[2] + altitude))
                        .setHeading(heading)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(14.0));

    // WP7 - 12 m/s
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + 140.0,
                                                               current_position[1] + 0.0,
                                                               current_position[2] + altitude))
                        .setHeading(heading)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(12.0));

    // WP8 - 10 m/s
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + 160.0,
                                                               current_position[1] + 20.0,
                                                               current_position[2] + altitude))
                        .setHeading(heading)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(10.0));

    // WP9 - 8 m/s
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + 180.0,
                                                               current_position[1] + 0.0,
                                                               current_position[2] + altitude))
                        .setHeading(heading)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(8.0));

    // WP10 - 6 m/s
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + 200.0,
                                                               current_position[1] + 20.0,
                                                               current_position[2] + altitude))
                        .setHeading(heading)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(6.0));

    // WP11 - 4 m/s
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + 220.0,
                                                               current_position[1] + 0.0,
                                                               current_position[2] + altitude))
                        .setHeading(heading)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth)
                        .setMaxSpeed(4.0));

    // Land
    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());

    return job;
}

creos::Job createJob102(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "example-job-002");

    job.setMaxSpeed(4.0);

    // Takeoff
    job.addTakeOff(creos::TakeOff());

    constexpr float heading = 90.0f;

    // WP1 - 10 m altitude
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + 20.0,
                                                               current_position[1] + 0.0,
                                                               current_position[2] + 10.0))
                        .setHeading(heading)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth));

    // WP2 - 15 m altitude
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + 40.0,
                                                               current_position[1] + 20.0,
                                                               current_position[2] + 15.0))
                        .setHeading(heading)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth));

    // WP3 - 20 m altitude
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + 60.0,
                                                               current_position[1] + 0.0,
                                                               current_position[2] + 20.0))
                        .setHeading(heading)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth));

    // WP4 - 25 m altitude
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + 80.0,
                                                               current_position[1] + 20.0,
                                                               current_position[2] + 25.0))
                        .setHeading(heading)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth));

    // WP5 - 30 m altitude
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + 100.0,
                                                               current_position[1] + 0.0,
                                                               current_position[2] + 30.0))
                        .setHeading(heading)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth));

    // WP6 - 35 m altitude
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + 120.0,
                                                               current_position[1] + 20.0,
                                                               current_position[2] + 35.0))
                        .setHeading(heading)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth));

    // WP7 - 40 m altitude
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + 140.0,
                                                               current_position[1] + 0.0,
                                                               current_position[2] + 40.0))
                        .setHeading(heading)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth));

    // WP8 - 45 m altitude
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + 160.0,
                                                               current_position[1] + 20.0,
                                                               current_position[2] + 45.0))
                        .setHeading(heading)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth));

    // WP9 - 50 m altitude
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + 180.0,
                                                               current_position[1] + 0.0,
                                                               current_position[2] + 50.0))
                        .setHeading(heading)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth));

    // WP10 - 55 m altitude
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + 200.0,
                                                               current_position[1] + 20.0,
                                                               current_position[2] + 55.0))
                        .setHeading(heading)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth));

    // Land at mission completion
    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());

    return job;
}

creos::Job createJob103(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "example-job-003");

    job.setMaxSpeed(4.0);

    // Takeoff
    job.addTakeOff(creos::TakeOff());

    constexpr float pointA_X = 20.0f;
    constexpr float pointB_X = 50.0f;
    constexpr float heading  = 90.0f;

    // WP1 - Point A, 10 m
    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + pointA_X,
                                                   current_position[1], current_position[2] + 10.0))
            .setHeading(heading)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth));

    // WP2 - Point B, 20 m
    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + pointB_X,
                                                   current_position[1], current_position[2] + 20.0))
            .setHeading(heading)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth));

    // WP3 - Point A, 10 m
    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + pointA_X,
                                                   current_position[1], current_position[2] + 10.0))
            .setHeading(heading)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth));

    // WP4 - Point B, 20 m
    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + pointB_X,
                                                   current_position[1], current_position[2] + 20.0))
            .setHeading(heading)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth));

    // WP5 - Point A, 10 m
    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + pointA_X,
                                                   current_position[1], current_position[2] + 10.0))
            .setHeading(heading)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth));

    // WP6 - Point B, 20 m
    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + pointB_X,
                                                   current_position[1], current_position[2] + 20.0))
            .setHeading(heading)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth));

    // WP7 - Point A, 10 m
    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + pointA_X,
                                                   current_position[1], current_position[2] + 10.0))
            .setHeading(heading)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth));

    // WP8 - Point B, 20 m
    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + pointB_X,
                                                   current_position[1], current_position[2] + 20.0))
            .setHeading(heading)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth));

    // WP9 - Point A, 10 m
    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + pointA_X,
                                                   current_position[1], current_position[2] + 10.0))
            .setHeading(heading)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth));

    // WP10 - Point B, 20 m
    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + pointB_X,
                                                   current_position[1], current_position[2] + 20.0))
            .setHeading(heading)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth));

    // Land
    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());

    return job;
}

creos::Job createJob104(creos::Client &client, const std::array<float, 3> current_position)
{
    creos::Job job(client, "example-job-004");

    job.setMaxSpeed(4.0);

    // Takeoff as part of the mission
    job.addTakeOff(creos::TakeOff());

    // Waypoint 1
    job.addWaypoint(
        creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + 25.0, current_position[1],
                                                   current_position[2] + 10.0))
            .setHeading(90.0)
            .setHeadingBehavior(creos::HeadingBehavior::FromNorth));

    // Waypoint 2
    job.addWaypoint(creos::Waypoint(creos::Coordinate::FromLtp(current_position[0] + 50.0,
                                                               current_position[1] + 25.0,
                                                               current_position[2] + 10.0))
                        .setHeading(90.0)
                        .setHeadingBehavior(creos::HeadingBehavior::FromNorth));

    // Land at the end of the mission
    job.addLand(creos::Land());

    spdlog::info("{}", job.describe());

    return job;
}

void reportJobStatus(creos_messages::JobStatus status, size_t &finished_tasks, int &waypoint_index)
{
    if(finished_tasks < status.tasks.size())
    {
        const auto task = status.tasks[finished_tasks];

        // Log task completion
        if(task.status == creos_messages::ExecutionStatus::kSucceeded)
        {
            switch(task.type)
            {
            case creos_messages::TaskType::kTakeOff:
                spdlog::info("Takeoff completed");
                break;
            case creos_messages::TaskType::kMoveTroughPoses:
                waypoint_index++;
                spdlog::info("Waypoint {} reached", waypoint_index);
                break;
            case creos_messages::TaskType::kLand:
                spdlog::info("Landing completed");
                break;
            default:
                break;
            }
            finished_tasks++;
        }
    }
}

int main(int argc, char **argv)
{
    CLI::App app{"Waypoint Example - Showcase how to use the CreOS SDK to automatically take off "
                 "and fly through a series of waypoints, and then land. The example uses the Job "
                 "API to define a Job consisting of waypoint tasks with custom heading and speed "
                 "settings."};

    unsigned       update_frequency_hz = 100;
    ControllerType controller_type     = ControllerType::kHerelink;
    std::string    host                = "";
    unsigned       job_number          = 1;
    std::map<unsigned, std::function<creos::Job(creos::Client &, const std::array<float, 3>)>>
        job_map = {
            {1, createJob1},     {2, createJob2},    {3, createJob3},     {4, createJob4},
            {5, createJob5},     {8, createJob8},    {9, createJob9},     {10, createJob10},
            {11, createJob11},   {12, createJob12},  {13, createJob13},   {15, createJob15},
            {16, createJob16},   {24, createJob24},  {30, createJob30},   {31, createJob31},
            {32, createJob32},   {33, createJob33},  {34, createJob34},   {35, createJob35},
            {36, createJob36},   {37, createJob37},  {38, createJob38},   {39, createJob39},
            {40, createJob40},   {41, createJob41},  {42, createJob42},   {43, createJob43},
            {44, createJob44},   {45, createJob45},  {46, createJob46},   {47, createJob47},
            {48, createJob48},   {49, createJob49},  {101, createJob101}, {102, createJob102},
            {103, createJob103}, {104, createJob104}};

    app.add_flag(
        "-v,--verbose", [](auto) { spdlog::set_level(spdlog::level::debug); },
        "Enable verbose output");
    app.add_option("--host", host,
                   "<hostname:port> of the CREOS agent. When omitted, the 'CREOS_HOST:CREOS_PORT' "
                   "environment variables are used or localhost:7200 is used as a fallback.");
    app.add_option("-f,--frequency", update_frequency_hz,
                   "Update frequency in Hz. Default is 100 Hz.");

    std::string valid_jobs;
    for(const auto &job : job_map)
    {
        valid_jobs += std::to_string(job.first) + ",";
    }
    valid_jobs.pop_back(); // remove the last comma

    app.add_option("--job", job_number, "Select which job to run. Valid jobs are " + valid_jobs);
    app.add_option("--lat1", gps_job_config.lat1, "Job 11 first WGS84 latitude in degrees.");
    app.add_option("--lon1", gps_job_config.lon1, "Job 11 first WGS84 longitude in degrees.");
    app.add_option("--alt1", gps_job_config.alt1, "Job 11 first WGS84 altitude in meters.");
    app.add_option("--lat2", gps_job_config.lat2, "Job 11 second WGS84 latitude in degrees.");
    app.add_option("--lon2", gps_job_config.lon2, "Job 11 second WGS84 longitude in degrees.");
    app.add_option("--alt2", gps_job_config.alt2, "Job 11 second WGS84 altitude in meters.");

    app.add_option("--job30-x-offset", huge_ltp_job_config.x_offset,
                   "Job 30 LTP X offset from current position in meters.");
    app.add_option("--job30-y-offset", huge_ltp_job_config.y_offset,
                   "Job 30 LTP Y offset from current position in meters.");
    app.add_option("--job30-z-offset", huge_ltp_job_config.z_offset,
                   "Job 30 LTP Z offset from current position in meters.");
    app.add_option("--job30-safe-z-offset", huge_ltp_job_config.safe_z_offset,
                   "Job 30 safe climb altitude offset from current position in meters.");
    app.add_option("--job30-safe-climb-speed", huge_ltp_job_config.safe_climb_speed,
                   "Job 30 safe climb waypoint speed in m/s.");
    app.add_option("--job30-target-speed", huge_ltp_job_config.target_speed,
                   "Job 30 target waypoint speed in m/s.");

    app.add_flag_callback(
        "--jeti", [&controller_type]() { controller_type = ControllerType::kJeti; },
        "Use Jeti controller instead of Herelink controller");
    CLI11_PARSE(app, argc, argv);

    setup_logging("waypoint_example");

    // Connect to the CreOS server
    creos::Client client = createClient(host);

    // Setup DroneState
    std::shared_ptr<DroneState> drone_state = std::make_shared<DroneState>();
    client.sensors()->subscribeToPose(drone_state->GetGlobalPoseCallback());
    client.setpoint_control()->subscribeToCurrentControlSource(
        drone_state->GetControlSourceCallback());
    client.diagnostics()->subscribeToState(drone_state->GetStateCallback());

    // Setup Controller input
    std::shared_ptr<IRemoteController> controller = CreateRemoteController(controller_type);
    client.sensors()->subscribeToRemoteController(controller->GetControllerStateCallback());

    bool execution_active = false;
    controller->RegisterActivationButtonCallback(
        [&execution_active]()
        {
            execution_active = !execution_active;
            spdlog::info("Execution active: {}", execution_active ? "true" : "false");
        });

    JobState job_state = JobState::kStartJob;

    // Function for create job
    std::function<creos::Job(creos::Client &, const std::array<float, 3>)> createJobFunc;

    if(job_map.find(job_number) != job_map.end())
    {
        createJobFunc = job_map[job_number];
    }
    else
    {
        spdlog::error("Invalid job number: {}. Valid job numbers are {}", job_number, valid_jobs);
        return 1;
    }

    spdlog::info("Selected job number: {}. Press the activation button on the controller to start "
                 "execution.",
                 job_number);

    size_t                      finished_tasks = 0;
    int                         waypoint_index = 0;
    creos_messages::JobStatus   status = {creos_messages::ExecutionStatus::kNotLoaded, {}, {}};
    std::shared_ptr<creos::Job> job    = nullptr;

    try
    {
        while((status.status != creos_messages::ExecutionStatus::kSucceeded) &&
              (status.status != creos_messages::ExecutionStatus::kCanceled) &&
              (status.status != creos_messages::ExecutionStatus::kFailed))
        {
            if(execution_active)
            {
                switch(job_state)
                {
                case JobState::kStartJob:
                {
                    // Create the job when execution is activated. We create a new job each time
                    // execution is activated, so if you deactivate and reactivate execution, a new
                    // job will be created and started.
                    job = std::make_shared<creos::Job>(
                        createJobFunc(client, drone_state->GetPosition()));

                    // Start the job execution
                    job->start();
                    job_state = JobState::kJobRunning;
                    break;
                }
                case JobState::kJobRunning:
                {
                    status = job->getJobStatus();
                    reportJobStatus(status, finished_tasks, waypoint_index);
                    break;
                }
                }
            }
            else
            {
                if(job_state == JobState::kJobRunning)
                {
                    job->stop();
                    break;
                }
            }

            spin(update_frequency_hz);
        }

        if(status.status == creos_messages::ExecutionStatus::kSucceeded)
        {
            std::cout << std::endl;
            std::cout << "Example completed successfully!" << std::endl;
        }
        else
        {
            std::cout << std::endl;
            std::cout << "Example did not complete successfully." << std::endl;
        }
    }
    catch(const creos::JobException &e)
    {
        spdlog::error("Job execution occurred with SDK error: {}", e.what());
        return 1;
    }
    catch(const std::exception &e)
    {
        spdlog::error("Job execution occurred with error: {}", e.what());
        return 1;
    }

    return 0;
}
