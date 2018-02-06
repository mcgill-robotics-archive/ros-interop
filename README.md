# AUVSI SUAS Interoperability ROS Client

[master]: https://dev.mcgillrobotics.com/buildStatus/icon?job=ros-interop/master
[master url]: https://dev.mcgillrobotics.com/blue/organizations/jenkins/ros-interop
[![master]][master url]

This ROS package provides a client to communicate with the
[AUVSI SUAS Interoperability server](https://github.com/auvsi-suas/interop).

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
export INTEROP_USERNAME=<username>
export INTEROP_PASSWORD=<password>
roslaunch interop interop.launch base_url:=<base_url>
```

or, if you prefer to avoid setting the `base_url` every time:

```bash
export INTEROP_HOST=<base_url>
export INTEROP_USERNAME=<username>
export INTEROP_PASSWORD=<password>
roslaunch interop interop.launch
```

There are no arguments for credentials for security concerns as otherwise the
credentials will be stored in plain text in the ROS parameter server. That can
easily leak to other computers in the same network if your network security is
misconfigured.

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
-   `~air_drop`: Air drop position, `geographic_msgs/GeoPointStamped`.
-   `~off_axis_obj`: Off axis object position, `geographic_msgs/GeoPointStamped`.
-   `~emergent_obj`: Emergent object last known position,
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

### `objects`

This by default serves ROS services to interact with the interoperability
odlc API. It also generates all the necessary object files in the Object File
Format described in the competition rules.

The object files are stored in a timestamped directory within the
directory defined by the `objects_root` ROS launch argument. You may also set
this argument with the `$INTEROP_OBJECTS_ROOT` environment variable as follows:

```bash
export INTEROP_OBJECTS_ROOT=/path/to/object_files
```

#### Objects

-   `~add`: Adds new object, `AddObject`.
-   `~get`: Retrieves specific object, `GetObject`.
-   `~update`: Updates specific object with new characteristics `UpdateObject`.
-   `~delete`: Deletes specific object, `DeleteObject`.
-   `~all`: Gets all submitted objects, `GetAllObjects`.

#### Thumbnails

-   `~image/set`: Sets or updates object image thumbnail, `SetObjectImage`.
-   `~image/get`: Retrieves object image thumbnail, `GetObjectImage`.
-   `~image/delete`: Deletes object image thumbnail, `DeleteObjectImage`.
-   `~image/compressed/set`: Sets or updates object image thumbnail with a `CompressedImage`, `SetObjectCompressedImage`.
-   `~image/compressed/get`: Retrieves object image thumbnail as a `CompressedImage`, `GetObjectCompressedImage`.

#### Syncing

-   `~clear`: Clears all local and remote objects, `Trigger`.
-   `~reload`: Reloads all remote objects, `Trigger`.

## Arguments

The following are the run-time ROS launch arguments available:

#### Interop server and request parameters

-   `base_url`: AUVSI SUAS interop server url, default: `$INTEROP_HOST` if set.
-   `timeout`: Timeout for each request in seconds, default: `1.0`.
-   `verify`: Whether to verify SSL certificates for HTTPS requests, default: `true`.

#### Local object file directory

-   `objects_root`: The parent of all timestamped directories containing object files, default: `$INTEROP_OBJECTS_ROOT` if set, or `~/object_files/`.
-   `interop_update_period`: Duration between attempts to sync the object files of the current run to the interop server, default: `10.0` (i.e. 10.0 s).

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
-   `air_drop_topic`: `geographic_msgs/GeoPointStamped` position of the air drop object,
    default: `~mission_info/air_drop`.
-   `emergent_obj_topic`: `geographic_msgs/GeoPointStamped` last known position of the emergent object,
    default: `~mission_info/emergent_obj`.
-   `off_axis_obj_topic`: `geographic_msgs/GeoPointStamped` position of the off axis object,
    default: `~mission_info/off_axis_obj`.
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
                  will retrieve active mission instead, default: `-1`.

#### Synchronization settings

_Advanced: shouldn't need to be modified._ This tweaks how incoming messages
are synchronized in order to be properly paired. For more information, see
[message filters](https://wiki.ros.org/message_filters/ApproximateTime).

-   `sync_queue_size`: Message synchronization queue size, default: `2`.
-   `max_sync_delay`: Maximum message synchronization delay in seconds,
    default: `1`.

#### Miscellaneous

-   `ns`: Namespace for all `interop` nodes. Can be used to launch several
    `interop` instances if wanted, default: `interop`.

## Debugging

To debug any issues you may have, you can enable `debug` logging for the node
in question to get more information about what is happening. You can do so with
`rqt_logger_level`, by running the following:

```bash
rosrun rqt_logger_level rqt_logger_level
```

and selecting the desired log level for the node's `rosout` logger.

## Contributing

Contributions are welcome. Simply open an issue or pull request on the matter,
and it will be accepted as long as it does not complicate the code base too
much.

As for style guides, we follow the ROS Python Style Guide for ROS-specifics and
the Google Python Style Guide for everything else.

### Linting

We use [YAPF](https://github.com/google/yapf) for all Python formatting needs.
You can auto-format your changes with the following command:

```bash
yapf --recursive --in-place --parallel .
```

We also use [catkin_lint](https://github.com/fkie/catkin_lint) for all `catkin`
specifics. You can lint your changes as follows:

```bash
catkin lint --explain -W2 .
```

## License

See [LICENSE](LICENSE).
