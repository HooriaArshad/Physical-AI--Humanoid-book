---
sidebar_position: 3
title: "Chapter 3: ROS 2 Fundamentals"
---

# Chapter 3: ROS 2 Fundamentals

## What is ROS 2?

ROS 2 is an open-source middleware framework for building robot applications. Key concepts include nodes, topics, services, and actions for distributed robot control.

## Core Concepts

### Nodes
Modular processes that perform computation. Each node is independent and communicates with others.

### Topics  
Publish/subscribe pattern for asynchronous data streams (sensor readings, commands).

### Services
Request/response pattern for synchronous operations (queries, triggers).

### Actions
Long-running tasks with feedback (navigation, manipulation).

## Example: Simple Publisher

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2'
        self.publisher_.publish(msg)
```

## Common Tools

- `ros2 topic list` - List all topics
- `ros2 node list` - List all nodes  
- `ros2 bag record` - Record data
- `ros2 launch` - Start multiple nodes

## Summary

ROS 2 provides modular architecture for robot software. In the next chapter, we explore digital twin simulation.
