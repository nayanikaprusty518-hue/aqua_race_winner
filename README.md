# AquaRace 2026: Team Aqua Pilot
**Maintainer:** Nayanika Prusty  

---

## 1. Project Overview
This repository contains the `aqua_race_winner` ROS 2 package. The system is an autonomous "brain" for an Underwater Autonomous Vehicle (AUV) designed to navigate a 3D gate course using real-time sensor feedback and a custom PID control architecture.

## 2. Dependencies & Requirements
These dependencies are declared in `package.xml` and must be present for a successful build:
* **OS:** Ubuntu 22.04 LTS (Jammy Jellyfish)
* **Middleware:** ROS 2 Humble Hawksbill
* **ROS 2 Libraries:**
    * `rclpy` (Python Client API)
    * `geometry_msgs` (Twist commands for movement)
    * `sensor_msgs` (IMU and FluidPressure data)

## 3. Technical Approach & Algorithms
### **Control Architecture**
* **PID Controller:** A custom Python class implements Proportional, Integral, and Derivative logic. 
* **Heading (Yaw):** The Derivative ($K_d$) term is specifically tuned to eliminate "wobble" and handle the unbalanced physics of **Model Alpha**.
* **Depth (Heave):** Uses feedback from the `/depth` topic (FluidPressure) to maintain vertical stability through course transitions.

### **Mission Logic**
* **Finite State Machine (FSM):** The controller uses an automated waypoint list. 
* **Autonomous Switching:** Once the AUV's heading error falls below 0.05 radians of the current gate, the FSM triggers a target update to the next waypoint in the sequence, ensuring "Swift Navigation."

## 4. Installation & Build Instructions
Run these commands in your workspace terminal:

```bash
# Navigate to workspace
cd ~/ros2_ws

# Build the package
colcon build --packages-select aqua_race_winner

# Source the setup files
source install/setup.bash
```

## 5. Execution
To launch the autonomous pilot and begin the mission, execute:

```bash
ros2 run aqua_race_winner auv_pilot
```

## 6. Package Structure
* `controller_node.py`: Main autonomous logic containing the PID class and AUV node.
* `package.xml`: Build dependencies and package metadata.
* `setup.py`: Entry point configuration for the `auv_pilot` command.
* `resource/`: ROS 2 index resources.
