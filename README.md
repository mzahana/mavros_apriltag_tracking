# mavros_apriltag_tracking
This package allows a PX4-powered drone to track a moving target (an AprilTag in this case) which is detected by a camera. Tag detection is done using `apriltag_ros` package. The target's position and velocity estimation is done using an implementaion of a Kalman filter based on a constant velocity model. The drone is controllerd using an implementation of a position controller which sends velocity commands to PX4. The detection and control are done with respect to the drone frame. So, errors caused by drifts in the local fixed frame can be avoided.

**Kindly, give this repo a STAR if it helps you in your work**

Video:

[![mavros_apriltag_tracking](https://img.youtube.com/vi/5bqOWKYBr0k/0.jpg)](https://youtu.be/5bqOWKYBr0k "mavros_apriltag_tracking")

# Installation
This package is tested on Ubuntu 18 + ROS Melodic

* Clone this package into your `~/catkin_ws/src` folder
    ```sh
    cd ~/catkin_ws/src
    git clone https://github.com/mzahana/mavros_apriltag_tracking.git
    ```
* Use the `install/setup,sh` script to do the installation
    ```sh
    cd ~/catkin_ws/src/mavros_apriltag_tracking/install
    ./setup.sh
    ```
* Copy the modified `typhoon_h480` model to `/home/arrow/Firmware/Tools/sitl_gazebo/models`
    ```sh
    cp -R ~/catkin_ws/src/mavros_apriltag_tracking/models/typhoon_h480 ~/Firmware/Tools/sitl_gazebo/models/husky.urdf.xacro 
    ```
* Copy the modified Husky simulation model. **You need sudo for this one**
```sh
sudo cp ~/catkin_ws/src/mavros_apriltag_tracking/models/custom_husky/husky.urdf.xacro $(catkin_find husky_description/urdf)/
```
# Simulation
* Use the `launch/tracker.launch` file to run the simulation
    ```sh
    roslaunch mavros_apriltag_tracking tracker.launch
    ```
* Bring the drone above and close to the tag by publishing one message to the `/setpoint/local_pos` 
    ```sh
    rostopic pub --once /setpoint/local_pos geometry_msgs/Point "x: 0.0
    y: 0.0
    z: 3.0"
    ```
* Make the camera face downward (pitch rotation of -90 degrees)
    ```sh
    rostopic pub --once /mavros/mount_control/command mavros_msgs/MountControl "header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
    mode: 2
    pitch: -90.0
    roll: 0.0
    yaw: 0.0
    altitude: 0.0
    latitude: 0.0
    longitude: 0.0"
    ```
    **NOTE**: The camera won't move until the drone is armed, but the command will be saved.
* Arm the drone and activate the OFFBOARD mode for the setpoint to take effect
    ```sh
    rosservice call /arm_and_offboard "{}"
    ```

You should see the drone taking off. Then, once the tag on the Husky UGV is detected, the drone will be automatically controlled to hover above the tag. The dron will keep tracking the tag on top of the Husky while it's moving.

To move the Husky around, publish a linear velocity (along forward direction of the Husky's `x-axis`), and a rotational velocity about the Husky's `z-axis` as follows.
```sh
rostopic pub -r 10 /husky_velocity_controller/cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.4"
```

Look at the `launch/tracker.launch` file and change parameters as needed.

# Nodes
There are 2 ROS nodes in this package.
* **`mavros_offboard_controller.py`**: Mainly implements a position controller and sends velocity commands to PX4. This controller accepts two types of position setpoints.
    * Position setpoints in local fixed frame. Corresponding topic is `/setpoint/local_pos`
    * Position setpoints relative to the drone horizontal body frame. This one is used for the tag tracking as the tag is detected relative to the camera. Corresponding topic is `/setpoint/relative_pos`
* **`apriltag_setpoint_publisher.py`**: This node takes the pose of the detected tag and computes relative positino setpoints which are published to `/setpoint/relative_pos` topic.
* **`kf_tracker_node`**: Performs Kalman filtering based on a constant velocity model for object tracking. The input to this node is a PoseStamped topic of the object pose measurements (only the 3D position is consumed). The output of the filter (`kf/estimate` topic) is an estimate of the 3D position of the tracked object in the same frame as the input measrements.

# Tuning
The position controller may require tuning for the PI gains. There are two services to help you tune those gains in runtime, for both horizontal and verical controllers.

To tune the PI gains for the horizontal controller,
```sh
rosservice call /horizontal_controller/pid_gains "p: 1.0
i: 0.01
d: 0.0"
```
for the verical controller,
```sh
rosservice call /vertical_controller/pid_gains "p: 1.0
i: 0.01
d: 0.0"
```

Error signals are also published to the following topics. This helps you to plot the error response while tuning the controllers.

```sh
# Velocity errors help you to tune the velocity controller on PX4 side
/analysis/body_vel_err
/analysis/local_vel_err

# Position errors help you to tune the position controllers implemented in this package
/analysis/local_pos_err
/analysis/relative_pos_err
```

You can use `rqt_plot` or `plotjuggler` to plot signals in realtime.