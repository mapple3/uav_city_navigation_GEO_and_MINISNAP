# UAV City Navigation: SE(3) Control & Minimum Snap Trajectory

This repository contains a fully consolidated ROS package for simulating autonomous Unmanned Aerial Vehicle (UAV) flight in a complex urban environment. It integrates a custom Gazebo city model, differential flatness-based SE(3) geometric control, and Minimum Snap trajectory generation.

## 🧠 Principle & Method

This package bridges high-level path planning with low-level rotor control:

1.  **Minimum Snap Trajectory Generation:** * **Method:** Generates smooth, continuous, piecewise polynomial trajectories between user-defined waypoints. By minimizing the "snap" (the fourth derivative of position), the planner ensures that the generated path perfectly respects the dynamic constraints of a quadrotor, resulting in smooth thrust and body-rate commands.
2.  **Nonlinear SE(3) Geometric Control:** * **Method:** A cascade control architecture that tracks the generated trajectory. It utilizes differential flatness to map desired position, velocity, acceleration, and jerk into target attitudes (quaternions) and normalized thrust. The attitude loop then computes body-rate commands sent directly to the PX4 flight controller.
3.  **Simulation Environment:** * **Method:** The drone (Iris) is spawned natively at the global origin `(0,0,0)` to maintain mathematical consistency, while a large-scale `.dae` city mesh (`city_osm_roundabout`) is offset physically in Gazebo and overlaid visually as a transparent marker in RViz for collision-aware planning.

## 📦 Dependencies

This package is designed for **ROS Noetic** (Ubuntu 20.04) and requires a functional **PX4 SITL** environment.

### Core ROS Packages
Ensure you have the following standard ROS dependencies installed:
```bash
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs
sudo apt-get install ros-noetic-tf2-ros ros-noetic-dynamic-reconfigure
