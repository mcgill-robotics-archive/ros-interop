# AUVSI SUAS Interopability ROS Client

[master]: http://dev.mcgillrobotics.com/buildStatus/icon?job=ros-interop_master
[master url]: http://dev.mcgillrobotics.com/job/ros-interop_master
[![master]][master url]

This ROS package provides a client to communicate with the
[AUVSI SUAS Interopability server](https://github.com/auvsi-suas/interop).

*This package has only been tested on ROS Indigo and ROS Jade on Ubuntu 14.04.
Use at your own risk.*

## Setting up

You must clone this repository as `interop` into your catkin workspace:

```bash
git clone https://github.com/mcgill-robotics/ros-interop.git interop
```

## Dependencies

Before proceeding, make sure to install all dependencies by running:

```bash
rosdep update
rosdep install interop
```

## Compiling

You **must** compile this package before being able to run it. You can do so
by running:

```bash
catkin_make
```

from the root of your `catkin` workspace.

### Testing

You can run the ROS tests to make sure the package is indeed functional by
running the following from the root of your `catkin` workspace:

```bash
catkin_make run_tests
```

## Running

To run, simply launch the package with:

```bash
roslaunch interop interop.launch base_url:=<base_url> username:=<username> password:=<password>
```

## Nodes

This package has the following nodes available:

### `obstacles`

This by default publishes both moving and stationary obstacles at 20 Hz to the
following topics:

-   `~moving`: Moving obstacles, `visualization_msgs/MarkerArray`.
-   `~stationary`: Stationary obstacles, `visualization_msgs/MarkerArray`.

#### Visualizing

Stationary and moving obstacles can be visualized in `rviz` as `MarkerArray`s
relative to the `odom` frame in UTM coordinates. Note that the marker's
positions are in UTM coordinates, so you will likely not be able to see the
obstacles in `rviz` (they will be *very* far away) without a static
transformation to cancel this out. For example, you could have a static
transformation `map -> odom` of your drone's starting position or your ground
station's position in UTM coordinates, and then visualize the obstacles
relative to the `map` frame.

If set up properly, it should look like this:

<img width="1680" alt="rviz"
 src="https://cloud.githubusercontent.com/assets/723610/15096449/d68b0ca2-14ac-11e6-82dc-8513809f7510.png">

where the cylinder is a stationary obstacle, and the sphere is a moving
obstacle.


### `mission_info`

This by default publishes mission information at 1 Hz to the following topics:

-   `~flyzones`: Flight boundaries, `FlyZoneArray`.
-   `~search_grid`: Search grid area, `geometry_msgs/PolygonStamped`.
-   `~waypoints`: List of waypoints, `visualization_msgs/Marker`.
-   `~air_drop_pos`: Air drop position, `geometry_msgs/PointStamped`.
-   `~off_axis_targ`: Off axis target position, `geometry_msgs/PointStamped`.
-   `~emergent_targ_loc`: Emergent target last known location, 
                          `geometry_msgs/PointStamped`

This also provides the following services to change missions:

-   `~get_active_mission`: Change the mission being published to the current 
                           active mission, `std_srvs/Trigger`
-   `~get_mission_by_id` : Change the mission being published to the mission of
                           the given id, `GetMissionByID`

### `telemetry`

This by default subscribes to telemetry data on the following topics, and
uploads them to the interoperability server:

-   `/mavros/global_position/global`: GPS and altitude data,
    `sensor_msgs/NavSatFix`.
-   `/mavros/global_position/compass_hdg`: Current heading in degrees relative
    to true north, `std_msgs/Float64`.

### `targets`

This by default serves ROS services to interact with the interoperability
targets API with the following:

#### Targets

-   `~add`: Adds new target, `AddTarget`.
-   `~get`: Retrieves specific target, `GetTarget`.
-   `~update`: Updates specific target with new characteristics `UpdateTarget`.
-   `~delete`: Deletes specific target, `DeleteTarget`.
-   `~all`: Gets all submitted targets, `GetAllTargets`.

#### Thumbnails

-   `~image/set`: Sets or updates target image thumbnail, `SetTargetImage`.
-   `~image/get`: Retrieves target image thumbnail, `GetTargetImage`.
-   `~image/delete`: Deletes target image thumbnail, `DeleteTargetImage`.

## Arguments

The following are the run-time ROS launch arguments available:

#### Interop server, credentials, and request parameters

-   `base_url`: AUVSI SUAS interop server url, default: `http://interop:80`.
-   `username`: AUVSI SUAS interop server username, default: `testadmin`.
-   `password`: AUVSI SUAS interop server password, default: `testpass`.
-   `timeout`: Timeout for each request in seconds, default: `1.0`.

#### Subscribed topics

-   `navsat_topic`: `sensor_msgs/NavSatFix` feed of the drone's GPS position to
    transmit to the server in the telemetry message,
    default: `/mavros/global_position/global`.
-   `compass_topic`: `std_msgs/Float64` feed of the drone's heading in degrees
    to transmit to the server in the telemetry message,
    default: `/mavros/global_position/compass_hdg`.

#### Published topics

-   `flyzones_topic`: `FlyZoneArray` flight boundaries,
    default: `~mission_info/flyzones`.
-   `search_grid_topic`: `geometry_msgs/PolygonStamped` search grid polygon,
    default: `~mission_info/search_grid`.
-   `waypoints_topic`: `visualization_msgs/Marker` list of waypoints,
    default: `~mission_info/waypoints`.
-   `air_drop_topic`: `geometry_msgs/PointStamped` position of the air drop target,
    default: `~mission_info/air_drop_loc`.
-   `emergent_targ_topc`: `geometry_msgs/PointStamped` last known position of the emergent target,
    default: `~mission_info/emergent_targ_loc`.
-   `off_axis_targ_topic`: `geometry_msgs/PointStamped` position of the off axis target,
    default: `~mission_info/off_axis_targ`.
-   `moving_topic`: `visualization_msgs/MarkerArray` feed of the moving
    obstacles, default: `~obstacles/moving`.
-   `stationary_topic`: `visualization_msgs/MarkerArray` feed of the stationary
    obstacles, default: `~obstacles/stationary`.

#### Publication periods

-   `obstacles_period`: Period to request and publish obstacles at in seconds,
    default: `0.05` (i.e., 20 Hz).
-   `mission_info_period`: Period to publish mission information at
    in seconds, default: `1` (i.e., 1 Hz).

#### Frame IDs

-   `obstacles_frame`: Frame ID of the obstacles' `MarkerArray` messages,
    default: `odom`.
-   `missions_frame`: Frame ID for the mission messages, default: `odom`.

#### Mission IDs

-   `mission_id`: ID for the first mission to access. Negative numbers 
                  will retrieve active mission instead
                  , default: `-1`.

#### Synchronization settings

_Advanced: shouldn't need to be modified._ This tweaks how incoming messages
are synchronized in order to be properly paired. For more information, see
[message filters](http://wiki.ros.org/message_filters/ApproximateTime).

-   `sync_queue_size`: Message synchronization queue size, default: `2`.
-   `max_sync_delay`: Maximum message synchronization delay in seconds,
    default: `1`.

## Known issues

- Obstacles flicker in `rviz`.

## Contributing

Contributions are welcome. Simply open an issue or pull request on the matter,
and it will be accepted as long as it does not complicate the code base too
much.

As for style guides, we follow the ROS Python Style Guide for ROS-specifics and
the Google Python Style Guide for everything else.

Finally, we have a strict 80 character line limit and four spaces per
indentation.

## License

See [LICENSE](LICENSE).
