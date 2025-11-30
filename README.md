# Robot Safety Control System

## Team Members
- dkandl2s
- dbalaj2s

## Overview

This package contains two different implementations for controlling a robot safely. Both implementations monitor the battery level and check for obstacles, then decide whether the robot should move forward, rotate, or stop.

1. **SMACH-based Implementation**: Uses a state machine to manage robot states based on sensor inputs.
2. **Behavior Tree Implementation**: Uses a behavior tree to prioritize actions based on sensor inputs.

## Installation

Install the required dependencies:

```bash
# Install SMACH packages
sudo apt-get install ros-humble-executive-smach ros-humble-smach-ros
# Install py_trees ROS package
sudo apt-get install ros-humble-py-trees-ros
# Install py_trees Python packages
pip3 install py-trees py-trees-ros-interfaces 
```

## Build Instructions

```bash
cd ~/ros2_ws
colcon build --packages-select serassignment1
source install/setup.bash
```

Note: You need to source the workspace in every new terminal.

## Execution Instructions

### Launch Gazebo Simulation

First, launch the Gazebo simulation environment with the robot:

```bash
ros2 launch robile_gazebo gazebo_4_wheel.launch.py
```


### Run the Safety Node

In a new terminal, source the workspace and run one of the safety nodes:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run serassignment1 smach_node
```

Or run the behavior tree:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run serassignment1 bt_node
```

### Using Teleop for Manual Control

If the robot detects a collision and stops, you can manually move it away from the obstacle using keyboard teleop. Open a new terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use the keyboard controls to move the robot:
- w: move forward
- x: move backward
- a: rotate left
- d: rotate right
- s: stop

Once you move the robot away from the obstacle (distance greater than 0.4m for SMACH or 0.25m for behavior tree), the safety node will detect the clear path and automatically resume normal operation.

## Code Explanation

### SMACH Node (smach_node.py)

This implementation uses a state machine with four states. The robot transitions between states based on sensor data.

**State Classes:**

1. **MonitorBatteryAndCollision**: This is the initial state. It subscribes to battery level and laser scan topics. It checks if the battery is below 40% or if an obstacle is closer than 0.4 meters. Based on these checks, it returns one of three outcomes: "battery_low", "collision_detected", or "normal".

2. **MoveForward**: This state publishes velocity commands to move the robot forward at 0.5 m/s. It continuously monitors the battery and distance sensors while moving. If the battery drops below 40%, it returns "battery_low". If an obstacle comes within 0.4 meters, it returns "collision_detected".

3. **RotateBase**: When the battery is low, the robot enters this state and rotates in place at 1.0 rad/s. This simulates the robot looking for a charging station or waiting for the battery to recover. It keeps rotating until the battery level reaches 40% again, then returns "battery_ok".

4. **StopMotion**: This is the emergency state. It publishes zero velocity to stop the robot completely. The robot stays stopped until the obstacle is moved away (distance becomes greater than 0.4 meters), then returns "safe_again".


**State Machine Structure:**
**Root State Machine**
- **MonitorBatteryAndCollision**
   - "battery_low" -> RotateBase
   - "collision_detected" -> StopMotion
   - "normal" -> MoveForward
- **MoveForward**
   - "battery_low" -> RotateBase
   - "collision_detected" -> StopMotion
   - continues in MoveForward
- **RotateBase**
   - "battery_ok" -> MoveForward
- **StopMotion**
   - "safe_again" -> MoveForward            


### Behavior Tree Node (bt_node.py)

This implementation uses a hierarchical tree structure where behaviors are organized by priority. The tree is evaluated every 100 milliseconds.

**Custom Behavior Classes:**

1. **BatteryStatus2bb**: Subscribes to the battery level topic and writes the value to a shared blackboard. If the battery is below 30%, it sets a flag called "battery_low_warning" to True.

2. **LaserScan2bb**: Subscribes to the laser scan topic and finds the minimum distance to any obstacle. If this distance is less than 0.25 meters, it sets a flag called "collision_imminent" to True.

3. **IsCollisionImminent**: A condition check that reads the "collision_imminent" flag from the blackboard. Returns SUCCESS if True, FAILURE if False.

4. **IsBatteryLow**: A condition check that reads the "battery_low_warning" flag from the blackboard. Returns SUCCESS if True, FAILURE if False.

5. **StopMotion**: Publishes zero velocity to stop the robot. Returns RUNNING status to keep executing.

6. **Rotate**: Publishes angular velocity of 1.0 rad/s to rotate the robot. Returns RUNNING status.

7. **MoveForward**: Publishes linear velocity of 0.3 m/s to move forward. Returns RUNNING status.

**Tree Structure:**
**Root (Parallel)**
   - **Topics2BB (Sequence)**
     - BatteryStatus2bb
     - LaserScan2bb
   - **Priorities (Selector)**
     - IsCollisionImminent -> StopMotion
     - IsBatteryLow -> Rotate
     - MoveForward

The tree works as follows: The Parallel node runs both children simultaneously. The Topics2BB sequence continuously updates sensor data on the blackboard. The Priorities selector evaluates conditions from top to bottom and executes the first one that succeeds. If a collision is imminent, the robot stops. Otherwise, if the battery is low, the robot rotates. If neither condition is true, the robot moves forward.

## Topics

The nodes subscribe to:
- `/battery_level` (std_msgs/Float32): Current battery percentage
- `/scan` (sensor_msgs/LaserScan): Laser range data for obstacle detection

The nodes publish to:
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands for the robot

## Testing

### Testing in Gazebo

The best way to test is using the Gazebo simulation as described in the Execution Instructions section. The robot will naturally encounter obstacles in the simulated world and the laser scanner will detect them.

### Testing with Manual Topic Publishing

If you want to test without Gazebo, you can manually publish test data. Open a second terminal and publish:

```bash
ros2 topic pub /battery_level std_msgs/Float32 "data: 60"
ros2 topic pub /battery_level std_msgs/Float32 "data: 20"
```

### Monitoring Robot Behavior

Monitor the velocity commands being sent to the robot:
```bash
ros2 topic echo /cmd_vel
```

Check current battery level:
```bash
ros2 topic echo /battery_level
```

Check laser scan data:
```bash
ros2 topic echo /scan
```

## Verification Results

The `results/` folder contains screenshots from the implementation, including:
    -Gazebo simulation images showing the robot’s behavior
    -Terminal outputs from both the SMACH and behavior tree nodes
    -Verification of state transitions and behavior tree execution

And also added the FSM diagram in the results folder as a reference for the SMACH implementation.

## Configuration

SMACH Node:
- Battery threshold: 40%
- Collision distance: 0.4 meters
- Forward speed: 0.5 m/s
- Rotation speed: 1.0 rad/s

Behavior Tree Node:
- Battery threshold: 30%
- Collision distance: 0.25 meters
- Forward speed: 0.3 m/s
- Rotation speed: 1.0 rad/s

## Troubleshooting

If you get "No executable found":
```bash
cd ~/ros2_ws
colcon build --packages-select serassignment1
source install/setup.bash
```

Verify the executables are installed:
```bash
ros2 pkg executables serassignment1
```


















