---
sidebar_position: 3
---

# ROS2 - Robot Operating System 2

## Introduction to ROS2

ROS2 (Robot Operating System 2) is the next generation of ROS, designed for production robotics applications. It provides:

- Hardware abstraction
- Device drivers
- Package management
- Inter-process communication

## Key Concepts

### Nodes

Individual components that perform specific tasks:

```python
# Simple ROS2 node example
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Node initialized!')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Topics

Asynchronous message passing via publish/subscribe:

```python
# Publisher
self.publisher = self.create_publisher(String, 'topic_name', 10)
self.publisher.publish(String(data='Hello ROS2!'))

# Subscriber
def callback(self, msg):
    self.get_logger().info(f'Heard: {msg.data}')
self.subscription = self.create_subscription(
    String, 'topic_name', callback, 10)
```

### Services

Synchronous request/response pattern:

```python
# Service Server
from srv import AddTwoInts

def add_callback(self, request, response):
    response.sum = request.a + request.b
    return response

self.service = self.create_service(
    AddTwoInts, 'add_two_ints', add_callback)
```

### Actions

For long-running tasks with feedback:

```python
# Action Server
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose

def execute_callback(self, goal_handle):
    # Execute the action
    for i in range(100):
        # Do work...
        feedback = NavigateToPose.Feedback()
        goal_handle.publish_feedback(feedback)
    result = NavigateToPose.Result()
    return result
```

## ROS2 Architecture

### DDS (Data Distribution Service)

ROS2 uses DDS for communication, providing:
- Real-time performance
- Quality of Service (QoS) settings
- Built-in discovery

### Quality of Service (QoS)

QoS policies include:
- **Reliability**: Reliable vs. Best Effort
- **Durability**: Transient Local vs. Volatile
- **Deadline**: Time constraints
- **Liveliness**: Communication liveness

## ROS2 Tools

| Tool | Purpose |
|------|---------|
| `ros2 run` | Launch a node |
| `ros2 launch` | Launch multiple nodes |
| `ros2 topic list` | List active topics |
| `ros2 topic echo` | View topic messages |
| `ros2 node list` | List active nodes |
| `rviz2` | 3D visualization |
| `gazebo` | Physics simulation |

## Workspace Structure

```
my_ros2_ws/
├── src/
│   ├── my_package/
│   │   ├── src/
│   │   ├── include/
│   │   ├── launch/
│   │   ├── config/
│   │   ├── package.xml
│   │   └── setup.py
│   └── another_package/
├── build/
├── install/
└── log/
```

## Creating a Package

```bash
# Create a new package
ros2 pkg create --build-type ament_python my_package

# Build the workspace
colcon build

# Source the setup
source install/setup.bash
```

## URDF - Robot Description

The Unified Robot Description Format (URDF) describes robot geometry:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </visual>
  </link>
  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </visual>
  </link>
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

## ROS2 Navigation Stack

Navigation components:

```python
# Nav2 stack integration
from nav2_simple_commander.robot_navigator import BasicNavigator

navigator = BasicNavigator()
navigator.waitUntilNav2Active()

# Navigate to a pose
initial_pose = Navigator.getPoseStamped([x, y, theta])
navigator.goToPose(initial_pose)

while not navigator.isTaskComplete():
    feedback = navigator.getFeedback()
```

## Best Practices

1. Use lifecycle nodes for robust state management
2. Implement error handling and recovery
3. Use parameters for configuration
4. Follow naming conventions (`/namespace/node_name`)
5. Test with `ros2 test`

## Exercises

1. Create a publisher/subscriber pair
2. Build a simple service client
3. Load and visualize a URDF in RViz
4. Implement a custom action server

## Summary

ROS2 provides a robust framework for building robotics applications. Its modular architecture, DDS-based communication, and rich ecosystem make it ideal for production robotics systems.
