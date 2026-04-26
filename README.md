# UAV City Navigation: SE(3) Control & Minimum Snap Trajectory

This repository contains a fully consolidated ROS package for simulating autonomous Unmanned Aerial Vehicle (UAV) flight in a complex urban environment. It integrates a custom Gazebo city model, differential flatness-based SE(3) geometric control, and Minimum Snap trajectory generation.

## 🧠 Principle & Method

This package bridges high-level path planning with low-level rotor control:

1.  **Minimum Snap Trajectory Generation:**
    * **Method:** Generates smooth, continuous, piecewise polynomial trajectories between user-defined waypoints. By minimizing the "snap" (the fourth derivative of position), the planner ensures that the generated path perfectly respects the dynamic constraints of a quadrotor, resulting in smooth thrust and body-rate commands.

2.  **Nonlinear SE(3) Geometric Control:**
    * **Method:** A cascade control architecture that tracks the generated trajectory. It utilizes differential flatness to map desired position, velocity, acceleration, and jerk into target attitudes (quaternions) and normalized thrust. The attitude loop then computes body-rate commands sent directly to the PX4 flight controller.

3.  **Simulation Environment:**
    * **Method:** The drone (Iris) is spawned natively at the global origin (0,0,0) to maintain mathematical consistency, while a large-scale .dae city mesh (city_osm_roundabout) is offset physically in Gazebo and overlaid visually as a transparent marker in RViz for collision-aware planning.

## 📦 Dependencies

This package is designed for ROS Noetic (Ubuntu 20.04) and requires a functional PX4 SITL environment.

### Core ROS Packages
Ensure you have the following standard ROS dependencies installed:

    sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs
    sudo apt-get install ros-noetic-tf2-ros ros-noetic-dynamic-reconfigure

### System Libraries
* Eigen3: Required for matrix operations and SE(3) math.
    
    sudo apt-get install libeigen-dev

### Custom Message Packages
This controller relies on custom tracking messages. Ensure these are available in your workspace (or build them as dependencies if hosted separately):
* quadrotor_msgs
* controller_msgs

## 🚀 Launching the Simulation

To avoid opening multiple terminals, this repository uses a master bash script to launch Gazebo, the controller, the trajectory planner, and the RViz visualizer simultaneously.

1. Make sure the script is executable:
   
   chmod +x launch_sim.sh
   chmod +x scripts/rviz_city_marker.py

2. Run the master launch script:
   
   ./launch_sim.sh

Note: The script assumes your workspace is named ~/catkin_ws_mine. Update the WS_PATH variable in the script if your workspace is named differently.

## 🎮 Interacting with RViz (Setting Waypoints)

Once the simulation is running, the UAV will take off to a hover state. You can command it to fly through the city using RViz.

### 1. Visualizing the City
The bash script automatically runs rviz_city_marker.py, which projects a transparent 3D mesh of the Gazebo city into your RViz environment. This allows you to see the buildings and plan waypoints safely down the streets.

### 2. Setting X and Y Coordinates
To generate a trajectory, you will feed waypoints to the waypoint_generator node using the RViz GUI:
1.  Look at the top toolbar in RViz.
2.  Click the 2D Nav Goal button (Keyboard shortcut: g).
3.  Click anywhere on the grid/street to drop a waypoint. The direction you drag your mouse will determine the yaw (heading) of that waypoint.
4.  Repeat this process to drop multiple sequential waypoints through the city streets.

### 3. Setting the Z Coordinate (Altitude)
By default, the "2D Nav Goal" tool only selects X and Y coordinates on the ground plane, setting Z = 0.
* Automatic Z-Height: The waypoint_generator node intercepts your 2D clicks and automatically assigns a safe flight altitude based on the planning/initpos_z or default tracking parameters defined in the launch file.
* Execution: Once the waypoints are placed, the trajectory_generator_node will instantly solve the polynomial matrix and draw the curved flight path in RViz. The geometric controller will then command the PX4 to fly the drone along that path.

## 📂 Repository Structure

* /cfg: Dynamic reconfigure files for tuning controller gains mid-flight.
* /include & /src: Core C++ source code for the geometric controller and trajectory generator.
* /launch: Launch files for combining nodes and configuring default parameters.
* /models & /worlds: Gazebo simulation assets (SDFs, DAE meshes, and .world files).
* /rviz: Pre-configured RViz layouts for trajectory visualization.
* /scripts: Python nodes (e.g., the RViz city mesh publisher).
