# AUVSI SUAS Interopability ROS Client

This ROS package provides a client to communicate with the
[AUVSI SUAS Interopability server](https://github.com/auvsi-suas/interop).
*This package has been tested on ROS Indigo and ROS Jade.*

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
relative to the `odom` frame in UTM coordinates.

### `server_info`

This by default publishes server information at 20 Hz to the following topics:

-   `~message`: Message data, `std_msgs/String`.
-   `~message_timestamp`: Message timestamp, `std_msgs/Time`.
-   `~time`: Server time, `std_msgs/Time`.

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

-   `msg_topic`: `std_msgs/String` feed of the server message,
    default: `~server_info/message`.
-   `timestamp_topic`: `std_msgs/Time` feed of the server message timestamp,
    default: `~server_info/message_timestamp`.
-   `server_time_topic`: `std_msgs/Time` feed of the server's current time,
    default: `~server_info/time`.
-   `moving_topic`: `visualization_msgs/MarkerArray` feed of the moving
    obstacles, default: `~obstacles/moving`.
-   `stationary_topic`: `visualization_msgs/MarkerArray` feed of the stationary
    obstacles, default: `~obstacles/stationary`.

#### Publication periods

-   `obstacles_period`: Period to request and publish obstacles at in seconds,
    default: `0.05` (i.e., 20 Hz).
-   `server_info_period`: Period to request and publish server information at
    in seconds, default: `0.05` (i.e., 20 Hz).

#### Frame IDs

-   `obstacles_frame`: Frame ID of the obstacles' `MarkerArray` messages,
    default: `odom`.

#### Synchronization settings

_Advanced: shouldn't need to be modified._ This tweaks how incoming messages
are synchronized in order to be properly paired. For more information, see
[message filters](http://wiki.ros.org/message_filters/ApproximateTime).

-   `sync_queue_size`: Message synchronization queue size, default: `2`.
-   `max_sync_delay`: Maximum message synchronization delay in seconds,
    default: `1`.

