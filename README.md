# Autonomous Waypoint Navigation and Localization Using SLAM

This repository provides all the necessary resources for building and operating a UGV (Unmanned Ground Vehicle) capable of autonomous waypoint navigation and localization using SLAM (Simultaneous Localization and Mapping). The system integrates ROS-based components, including RTAB-Map for SLAM, `move_base` for path planning, and real-time motor control via Arduino for differential drive robots.



## Features

- **SLAM and Localization:** Utilize RTAB-Map for building and localizing within maps.
- **Waypoint Navigation:** Autonomous navigation using `move_base` and adaptive Monte Carlo Localization (AMCL).
- **Arduino Motor Control:** Controls differential drive motors via velocity commands from the ROS `cmd_vel` topic.
- **Map Server Integration:** Reuse prebuilt maps for efficient localization and navigation.



## Setup Instructions

### Quick Steps

1. **Clone the Repository:**
   ```bash
   git clone https://github.com/JasonM22345/ugv_navigation_with_slam.git
   ```

2. **Copy Packages to Your Catkin Workspace:**
   ```bash
   cp -r ugv_navigation_with_slam/move_base ~/catkin_ws/src
   cp -r ugv_navigation_with_slam/navigation_slam ~/catkin_ws/src
   ```

3. **Build the Workspace:**
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

4. **Install Additional Dependencies:**
   ```bash
   sudo apt install -y ros-$(rosversion -d)-rosserial ros-$(rosversion -d)-realsense2-camera ros-$(rosversion -d)-rtabmap
   ```

5. **(Optional) Download Preconfigured RViz File:**
   ```bash
   wget https://raw.githubusercontent.com/JasonM22345/ugv_navigation_with_slam/main/final_rviz.rviz
   ```

For detailed setup steps, refer to the [Setup Instructions](#setup-instructions).



## Usage Instructions

### Steps to Run the System

1. **Start ROS Core:**
   ```bash
   roscore
   ```

2. **Launch Core Nodes:**
   ```bash
   roslaunch navigation_slam final_no_rtab.launch
   ```

3. **Launch RTAB-Map:**
   ```bash
   roslaunch navigation_slam rtabmap_standalone.launch
   ```

   - Create the SLAM map by moving the robot manually or using teleoperation.
   - Once the map is created, switch RTAB-Map mode from **navigation** to **localization**.

4. **Upload Arduino Code and Start Serial Communication:**
   - Upload the Arduino code and run:
     ```bash
     rosrun rosserial_python serial_node.py /dev/ttyACM0
     ```

5. **Launch Move Base:**
   ```bash
   roslaunch move_base move_base.launch
   ```

6. **Start RViz:**
   ```bash
   rviz
   ```

7. **Set Waypoints in RViz:**
   Use **2D Nav Goals** to set waypoints for autonomous navigation.

For more details, refer to the [Usage Instructions](#usage-instructions).



## Creating and Using a Map

1. **Save the SLAM Map:**
   - Save the RTAB-Map database (`rtabmap.db`) after map creation.

2. **Export a 2D Map:**
   - Use `rtabmap-databaseViewer` to export the 2D map as a `.pgm` file, along with its `.yaml` metadata file.

3. **Use the Map with Map Server:**
   - Start the map server:
     ```bash
     rosrun map_server map_server rtabmap.yaml
     ```

4. **Navigate Using the Prebuilt Map:**
   - Use RViz to visualize the `/map` topic and set navigation goals.



## Arduino Code

### Simple Arduino Code (`differential_drive_pwm.ino`)

This code provides basic motor control for a differential drive robot. It subscribes to the ROS `cmd_vel` topic and adjusts motor speeds based on linear and angular velocity commands.

- **Motor Control:** Defines pins and functions to control motor direction and speed.
- **ROS Communication:** Subscribes to `cmd_vel` to process velocity commands.
- **Functions:**
  - `setup`: Initializes motor pins and ROS communication.
  - `loop`: Updates motor speeds based on incoming commands.

This version simplifies robot control by assuming default parameters for wheel radius and robot geometry.

### Advanced Arduino Code (`differential_drive_pwm-v2.ino`)

This code refines motor control by incorporating precise kinematics and robot parameters:

- Explicitly accounts for wheel radius, distance between wheels, and gear reduction ratio.
- Ensures accurate velocity calculations and precise movement.

In contrast to the simple version, which assumes unit parameters for calculations, this version uses real-world measurements for better performance and reliability.



## Repository

[GitHub Repository](https://github.com/JasonM22345/ugv_navigation_with_slam.git)

This repository contains all the required packages, configuration files, and Arduino code for the project.
