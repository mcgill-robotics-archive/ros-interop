# AUVSI SUAS Interopability ROS Client

[master]: https://dev.mcgillrobotics.com/buildStatus/icon?job=ros-interop_master
[master url]: https://dev.mcgillrobotics.com/job/ros-interop_master
[![master]][master url]

This ROS package provides a client to communicate with the
[AUVSI SUAS Interopability server](https://github.com/auvsi-suas/interop).

*This package has been tested on ROS Kinetic Kame on Ubuntu 16.04.*

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

or, if you prefer:

```bash
export INTEROP_HOST=<base_url>
export INTEROP_USERNAME=<username>
export INTEROP PASSWORD=<password>
roslaunch interop interop.launch
```

## Nodes

This package has the following nodes available:

### `obstacles`

This by default publishes both moving and stationary obstacles at 20 Hz to the
following topics:

-   `~moving`: Moving obstacles, `GeoSphereArrayStamped`.
-   `~stationary`: Stationary obstacles, `GeoCylinderArrayStamped`.

### `mission_info`

This by default publishes mission information at 1 Hz to the following topics:

-   `~flyzones`: Flight boundaries, `FlyZoneArray`.
-   `~search_grid`: Search grid area, `GeoPolygonStamped`.
-   `~waypoints`: List of waypoints, `WayPoints`.
-   `~air_drop_pos`: Air drop position, `geographic_msgs/GeoPointStamped`.
-   `~off_axis_targ`: Off axis target position, `geographic_msgs/GeoPointStamped`.
-   `~emergent_targ_loc`: Emergent target last known location,
                          `geographic_msgs/GeoPointStamped`.
-   `~home`: Home position, `geographic_msgs/GeoPointStamped`.

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
-   `~image/compressed/set`: Sets or updates target image thumbnail with a `CompressedImage`, `SetTargetCompressedImage`.
-   `~image/compressed/get`: Retrieves target image thumbnail as a `CompressedImage`, `GetTargetCompressedImage`.

#### Syncing

-   `~clear`: Clears all local and remote targets, `Trigger`.
-   `~reload`: Reloads all remote targets, `Trigger`.

## Arguments

The following are the run-time ROS launch arguments available:

#### Interop server, credentials, and request parameters

-   `base_url`: AUVSI SUAS interop server url, default: `$INTEROP_HOST` if set, or `http://interop:80`.
-   `username`: AUVSI SUAS interop server username, default: `$INTEROP_USERNAME` if set, or `testadmin`.
-   `password`: AUVSI SUAS interop server password, default: `$INTEROP_PASSWORD` if set, or `testpass`.
-   `timeout`: Timeout for each request in seconds, default: `1.0`.
-   `verify`: Whether to verify SSL cerificates for HTTPS requests, default: `true`.

#### Local object file directory

-   `targets_root`: The parent of all timestamped directories containing object files, default: `$INTEROP_OBJECTS_ROOT` if set, or `~/object_files/`.
-   `interop_update_period`: Duration between attempts to sync the object files of the current run to the interop server, default: `10.0` (i.e. 10.0 s).
-   `clear_targets`: Automatically clear all remote targets on start up (**use with caution**), default: `false`.

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
-   `search_grid_topic`: `GeoPolygonStamped` search grid polygon,
    default: `~mission_info/search_grid`.
-   `waypoints_topic`: `WayPoints` list of waypoints,
    default: `~mission_info/waypoints`.
-   `air_drop_topic`: `geographic_msgs/GeoPointStamped` position of the air drop target,
    default: `~mission_info/air_drop_loc`.
-   `emergent_targ_topic`: `geographic_msgs/GeoPointStamped` last known position of the emergent target,
    default: `~mission_info/emergent_targ_loc`.
-   `off_axis_targ_topic`: `geographic_msgs/GeoPointStamped` position of the off axis target,
    default: `~mission_info/off_axis_targ`.
-   `moving_topic`: `GeoSphereArrayStamped` feed of the moving
    obstacles, default: `~obstacles/moving`.
-   `stationary_topic`: `GeoSphereArrayStamped` feed of the stationary
    obstacles, default: `~obstacles/stationary`.

#### Publication periods

-   `obstacles_period`: Period to request and publish obstacles at in seconds,
    default: `0.05` (i.e., 20 Hz).
-   `mission_info_period`: Period to publish mission information at
    in seconds, default: `1` (i.e., 1 Hz).

#### Frame IDs

-   `obstacles_frame`: Frame ID of the obstacles' `MarkerArray` messages,
    default: `earth`.
-   `missions_frame`: Frame ID for the mission messages, default: `earth`.

#### Mission IDs

-   `mission_id`: ID for the first mission to access. Negative numbers
                  will retrieve active mission instead
                  , default: `-1`.

#### Synchronization settings

_Advanced: shouldn't need to be modified._ This tweaks how incoming messages
are synchronized in order to be properly paired. For more information, see
[message filters](https://wiki.ros.org/message_filters/ApproximateTime).

-   `sync_queue_size`: Message synchronization queue size, default: `2`.
-   `max_sync_delay`: Maximum message synchronization delay in seconds,
    default: `1`.

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
