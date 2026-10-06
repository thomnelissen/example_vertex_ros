# Helix example

This example will make the Vertex fly a helix: the [circle example](../circle_example/README.md), climbing and descending while it circles. It showcases how to use the SDK to fly a pattern with a drone that is already airborne. It will use its current position and heading to start flying. The radius, speed, climb and heading behaviour can be changed using CLI arguments.

> **Pre-condition:** The drone must be hovering stably in the air in **SDK mode** before activating this example.

## Structure

The code is split up into multiple parts:

- [RemoteController](../common/include/common/remote_controller_interface.hpp): Interprets the raw controller data and triggers an action callback.
- [DroneState](../common/include/common/drone_state_interface.hpp): Collects the state and position of the drone.
- [HelixReferences](src/helix_references.hpp): Class that calculates the setpoints for the drone to fly a helix.
- [Main](src/main.cpp): Main sets up all the classes and runs the main loop.

## Helix geometry

Seen from above, the helix is the circle of the circle example: the circle centre is placed directly in front of the drone (in the direction it is facing) at a distance equal to the configured radius, and the drone starts at the rear-most point of the circle and flies **clockwise**.

Over the first `--turns` turns the height goes up smoothly by `--climb`, over the next `--turns` turns back down to the start height, and so on.

With `--frontal` the nose turns along the path. The circle centre then lies `radius` to the **right** of the drone instead, so the drone starts flying straight ahead and does not have to turn on the spot first. Otherwise the heading stays the initial heading.

## HelixReferences

This class is the circle example's `CircleReferences` with a height. Every call to `GetNewStateReference` returns the next setpoint: the position, with the velocity and acceleration as feedforward. The speed law in time is the circle example's.

## Main

The main function sets up all the classes and runs the main loop. It will take care of the CLI arguments and will subscribe to the correct API's and register the correct callbacks.

The execution of the example can be toggled using the D button (SF switch when using Jeti Controller). Once the execution is active, the drone must already be flying in **SDK mode** (referred to as `user` control mode in the API). If the drone is not airborne when the execution is activated, the execution will stop immediately and the reason will be reported.

### Taking over manual control

To take over control manually at any time, switch out of **SDK** mode by pressing the **mode switch button**. This causes the drone to stop following the SDK setpoints immediately, and the helix execution is stopped automatically. To restart the helix from the new position, switch back to SDK mode and press **example activation button**.

### Starting and stopping the example

| Action | Effect |
|--------|--------|
| Press **example activation button** (execution inactive) | Activates the helix. The helix starts from the drone's current position and heading. |
| Press **example activation button** (execution active) | Stops the helix. The drone holds its current position. Press **example activation button** again to restart from the current position. |
| Switch out of **SDK** mode | Immediately stops the helix execution and returns manual control. Switch back to SDK mode and press **example activation button** to restart. |

## Usage

The example can be executed by running the following command:

```bash
./build/bin/helix_example
```

The following CLI arguments can be used:

- `--verbose`: Enable verbose output.
- `--radius`: Radius of the helix in meters. Default is 2 meters.
- `--speed`: Speed of the drone in meters per second. Default is 0.5 m/s.
- `--accel`: Maximum acceleration of the drone in metres per second squared. Default is 1.0 m/s^2.
- `--climb`: Height to climb, and descend again, in meters. Default is 1 meter.
- `--turns`: Turns to climb in, and the same to descend in. Default is 2.
- `--frontal`: Turn the nose along the path instead of holding the initial heading.
- `--frequency`: Update frequency in Hz. Default is 100 Hz.
- `--jeti`: Use Jeti controller. Default is Herelink.

> **Note:** The valid range for `radius`, `speed`, and `accel` is determined by the drone's parameter configuration and its flight limits. See the [flight controller parameters](https://avular-robotics.github.io/vertex_one_user_documentation/latest/flight_controller/parameters.html) for the limits and defaults.

Example:

```bash
./build/bin/helix_example --radius 3 --speed 1 --accel 1 --climb 2 --turns 2
```

The example can be stopped by pressing `Ctrl + C`.
