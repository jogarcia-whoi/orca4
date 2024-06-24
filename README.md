# Orca4 ![ROS2 CI](https://github.com/clydemcqueen/orca4/actions/workflows/build_test.yml/badge.svg?branch=main)

Orca4 is a set of [ROS2](http://www.ros.org/) packages that provide basic AUV (Autonomous Underwater
Vehicle) functionality for the [BlueRobotics BlueROV2](https://www.bluerobotics.com).

Orca4 uses [ArduSub](http://www.ardusub.com/) as the flight controller and
[mavros](https://github.com/mavlink/mavros) as the GCS.

Orca4 runs in [Gazebo Garden](https://gazebosim.org/home) using the standard buoyancy, hydrodynamics and thruster
plugins. The connection between ArduSub and Gazebo is provided by [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo).

## Diagrams

```mermaid
flowchart LR
    %% Existing Nodes
    N1([mavros_node]):::mavros
    N2([manager]):::orca_base
    N3([base_controller]):::orca_base
    N4([orb_slam2_stereo]):::orb_slam2_ros
    N5([camera_info_publisher]):::orca_base
    N6([image_bridge]):::ros_gz_image
    N7([parameter_bridge]):::ros_gz_bridge

    %% Nav2 Nodes
    N8([bt_navigator]):::nav2
    N9([controller_server]):::nav2
    N10([planner_server]):::nav2
    N11([behavior_server]):::nav2
    N12([waypoint_follower]):::nav2

    %% Existing Topics
    T1>mavros/vision_pose/pose]
    T2>mavros/local_position/pose]
    T3>mavros/rc/override]
    T4>mavros/setpoint_position/global]
    T5>camera_pose]
    T6>cmd_vel]
    T7>odom]
    T8>motion]
    T9>stereo_left]
    T10>stereo_right]
    T11>stereo_left/camera_info]
    T12>stereo_right/camera_info]
    T13>model/orca4/odometry]

    %% New Nav2 Topics
    T14>goal_pose]
    T15>plan]

    %% Existing Connections
    N1 -- "Vision-based pose" --> T1
    N1 -- "Local position" --> T2
    N1 -- "RC commands" --> T3
    N1 -- "Global setpoints" --> T4
    N2 -- "Camera pose" --> T5
    N2 -- "Velocity commands" --> T6
    N3 -- "Vision pose" --> T1
    N3 -- "RC overrides" --> T3
    N3 -- "Global setpoints" --> T4
    N3 -- "Odometry" --> T7
    N3 -- "Motion data" --> T8
    N3 -- "Velocity commands" --> T6
    N4 -- "Camera pose" --> T5
    N5 -- "Left camera info" --> T11
    N5 -- "Right camera info" --> T12
    N6 -- "Left image data" --> T9
    N6 -- "Right image data" --> T10
    N7 -- "Model odometry" --> T13

    T2 -- "Local position" --> N2
    T2 -- "Local position" --> N3
    T5 -- "Camera pose" --> N3
    T6 -- "Velocity commands" --> N3
    T9 -- "Left image data" --> N4
    T10 -- "Right image data" --> N4
    T11 -- "Left camera info" --> N4
    T12 -- "Right camera info" --> N4
    T13 -- "Model odometry" --> N3

    %% Nav2 Connections
    N8 -- "Navigation commands" --> T14
    N9 -- "Velocity commands" --> T6
    N10 -- "Path plan" --> T15
    N11 -- "Behavior commands" --> N8
    N12 -- "Waypoint commands" --> N8

    T14 -- "Goal pose" --> N10
    T15 -- "Planned path" --> N9
    T7 -- "Odometry" --> N8
    T7 -- "Odometry" --> N9

    %% Node color classes
    classDef mavros fill:#f9d71c
    classDef orca_base fill:#66c2a5
    classDef orb_slam2_ros fill:#fc8d62
    classDef ros_gz_image fill:#8da0cb
    classDef ros_gz_bridge fill:#e78ac3
    classDef nav2 fill:#e55d87

    %% Legend
    subgraph Legend
        L1([MAVROS]):::mavros
        L2([Orca Base]):::orca_base
        L3([ORB SLAM2]):::orb_slam2_ros
        L4([ROS Gazebo Image]):::ros_gz_image
        L5([ROS Gazebo Bridge]):::ros_gz_bridge
        L6([Navigation2]):::nav2
    end
```


## Sensors

The BlueROV2 provides the following interesting sensors:
* An [external barometer](https://bluerobotics.com/product-category/sensors-sonars-cameras/sensors/) provides depth
* An IMU provides attitude

Orca4 adds a simulated down-facing stereo camera and [ORB_SLAM2](https://github.com/clydemcqueen/orb_slam_2_ros/tree/orca4_galactic)
to generate a 3D pose as long as the camera has a good view of the seafloor.
The pose is sent to ArduSub and fused with the other sensor information.

If there is no view of the seafloor a synthetic pose is generated based on the last good pose and a simple motion model.

See [orca_base](orca_base/README.md) for details.

## Navigation

Orca4 uses the [Navigation2](https://navigation.ros.org/index.html) framework for mission
planning and navigation. Several simple Nav2 plugins are provided to work in a 3D environment:
* straight_line_planner_3d
* pure_pursuit_3d
* progress_checker_3d
* goal_checker_3d 

See [orca_nav2](orca_nav2/README.md) for details.

## Installation

See the [Dockerfile](docker/Dockerfile) for installation details.

Install these packages:
* [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
* [Gazebo Garden 7.1.0](https://gazebosim.org/docs/garden/install)
* [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo)
* [ArduSub](https://ardupilot.org/dev/docs/building-setup-linux.html)

Build ArduSub for SITL:
~~~
cd ~/ardupilot
./waf configure --board sitl
./waf sub
~~~

Populate the workspace:
~~~
mkdir -p ~/colcon_ws/src
cd colcon_ws/src
git clone https://github.com/clydemcqueen/orca4
vcs import < orca4/workspace.repos
~~~


Build ros_gz for Garden, not Fortress:
~~~
export GZ_VERSION=garden
~~~

Get dependencies, ignoring Gazebo Garden rosdep keys:
~~~
rosdep update
rosdep install -y --from-paths . --ignore-src --skip-keys="gz-transport12 gz-sim7 gz-math7 gz-msgs9"
~~~

MAVROS depends on GeographicLib, and GeographicLib needs some datasets:
~~~
sudo mavros/mavros/scripts/install_geographiclib_datasets.sh
~~~

Build the workspace:
~~~
cd ~/colcon_ws
colcon build
~~~

## Simulation

In a terminal run:
~~~
source src/orca4/setup.bash
ros2 launch orca_bringup sim_launch.py
~~~

This will bring up all of the components, including the Gazebo UI.
The surface of the water is at Z=0 and the sub will be sitting at the surface.
The world contains a sandy seafloor 10 meters below the surface.

![GAZEBO GUI](images/gazebo.png)

You should see ArduSub establish a connection to the ardupilot_gazebo plugin:
~~~
[ardusub-2] JSON received:
[ardusub-2] 	timestamp
[ardusub-2] 	imu: gyro
[ardusub-2] 	imu: accel_body
[ardusub-2] 	position
[ardusub-2] 	quaternion
[ardusub-2] 	velocity
~~~

At this point SLAM is not running because the seafloor is too far away,
but the sub can still move using dead-reckoning.
The [base_controller](orca_base/src/base_controller.cpp) node will send default camera poses to ArduSub
to warm up the EKF and the [manager](orca_base/src/manager.cpp) node will request attitude information at 20Hz.
Initialization completes when there is a good pose from the EKF:

~~~
[mavros_node-8] [INFO] [mavros.imu/handle_attitude_quaternion]: IMU: Attitude quaternion IMU detected!
[manager-9] [INFO] [manager/operator()]: EKF is running
[base_controller-10] [INFO] [base_controller/change_state]: EKF is running, state => RUN_NO_MAP
[base_controller-10] [INFO] [base_controller/UnderwaterMotion]: initialize odometry to {{-2.52304e-05, -3.28182e-05, -0.228547}, {0, 0, -5.00936e-05}}
~~~

Execute a mission in a second terminal:
~~~
source src/orca4/setup.bash
ros2 run orca_bringup mission_runner.py
~~~ 

![RVIZ2_GUI](images/rviz2.png)

The default mission will dive to -7m and move in a large rectangle.
At -6m the cameras will pick up a view of the seafloor at and ORB_SLAM2 will start:
~~~
[orb_slam2_ros_stereo-13] New map created with 571 points
[base_controller-10] [INFO] [base_controller/change_state]: map created, state => RUN_LOCALIZED
~~~


You should notice a loop closure sometime during the 2nd run around the rectangle. The adjustment is very small.

~~~
[orb_slam2_ros_stereo-13] Loop detected!
[orb_slam2_ros_stereo-13] Local Mapping STOP
[orb_slam2_ros_stereo-13] Local Mapping RELEASE
[orb_slam2_ros_stereo-13] Starting Global Bundle Adjustment
[orb_slam2_ros_stereo-13] Global Bundle Adjustment finished
[orb_slam2_ros_stereo-13] Updating map ...
[orb_slam2_ros_stereo-13] Local Mapping STOP
[orb_slam2_ros_stereo-13] Local Mapping RELEASE
[orb_slam2_ros_stereo-13] Map updated!
~~~

## Packages

* [`orca_base` Base controller, localization, frames](orca_base)
* [`orca_bringup` Launch files](orca_bringup)
* [`orca_description` SDF files](orca_description)
* [`orca_msgs` Custom messages](orca_msgs)
* [`orca_nav2` Nav2 plugins](orca_nav2)
* [`orca_shared` Dynamics model, shared utilities](orca_shared)
