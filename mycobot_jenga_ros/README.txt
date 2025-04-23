README.txt

MyCobot 280 ROS2 Inverse Kinematics Control
===========================================

Description
-----------
This project uses a ROS2 (Humble) node to control a MyCobot 280 robotic arm. 
It listens for 6-DOF pose messages (position + orientation) from a vision system 
and computes inverse kinematics (IK) using the Gauss-Newton method with finite-difference Jacobian approximation. 
It then publishes joint angle commands to move the arm to the desired pose.

The code includes:
- FK model for the MyCobot 280 arm
- Numerical Jacobian estimation
- Gauss-Newton IK solver with joint limit handling
- ROS2 node to listen for block detections and publish joint states

Dependencies
------------
- ROS2 Humble (Ubuntu 22.04 recommended)
- numpy
- scipy
- geometry_msgs
- sensor_msgs
- custom message type: `DetectedBlockArray` (from `mycobot_interfaces` package)
- MyCobot 280 robot (with joint interface)

Make sure your workspace contains:
- `robot_control` package (this project)
- `mycobot_interfaces` with `DetectedBlockArray` message

Build Instructions
------------------
1. Clone this repo and place it inside your ROS2 workspace:
    cd ~/ros2_ws/src git clone <this-repo>
    
2. Make sure your workspace includes any dependencies and build:
    cd ~/ros2_ws colcon build source install/setup.bash
    

Execution
---------
1. Start your vision node that publishes to `/detected_blocks` (must be `DetectedBlockArray` messages).
2. Run the IK node:
    ros2 run robot_control robot_control_node
    

Behavior
--------
When a block is detected, the robot will:
- Read its 3D pose
- Convert the quaternion to a rotation matrix
- Construct a target 4x4 transform
- Compute IK solution using Gauss-Newton
- Publish joint angles to the `/joint_commands` topic

Notes
-----
- FK and IK are approximated for the MyCobot 280 based on available geometry; joint limits are enforced in software.
- You can tune `max_step`, `epsilon`, or initial angles in `inverse_kinematics_mycobot()` for improved convergence.


Testing Vision Input
---------------------
To run the vision node as an isolated node (no robotics), follow these steps:

---

1. Open a terminal and launch the camera node:

```bash
    cd ~/ros2_ws
    source install/setup.bash
    ros2 launch realsense2_camera rs_launch.py camera_name:=mycamera
```

Leave this running in its own terminal.

---



2. Open a second terminal and run the vision node:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run vision_node vision_node
```

---

Tip:

If you ever get an error about a device being **busy**, make sure no other script (e.g., OpenCV RealSense, or `realsense-viewer`) is running and using the camera before launching the ROS node.

---

If you see an error like:
```
    xioctl(VIDIOC_S_FMT) failed, errno=16 Last Error: Device or resource busy
```

It means another process is using the RealSense camera. To resolve this:

---

Step 1: Check for conflicting processes

In your terminal, run:

```bash
    ps aux | grep -i realsense
```

Look for anything like:

- `realsense-viewer`
- Python/OpenCV scripts using `pyrealsense2`
- Old `ros2 run` or `ros2 launch` processes

---

Step 2: Kill the conflicting processes

Use the PID (second column) from the previous command:

```bash
    kill -9 <PID>
```

Example:

```bash
    kill -9 12345
```

Repeat for all conflicting processes.

---

Step 3: Replug camera (if needed)

If errors persist:

1. Unplug the RealSense USB cable  
2. Wait 2 seconds  
3. Plug it back in

---

Step 4: Relaunch the ROS2 node

```bash
    ros2 launch realsense2_camera rs_launch.py camera_name:=mycamera
```

---



