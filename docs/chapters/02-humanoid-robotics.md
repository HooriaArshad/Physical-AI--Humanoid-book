---
sidebar_position: 2
title: "Chapter 2: Basics of Humanoid Robotics"
---

# Chapter 2: Basics of Humanoid Robotics

## Why Humanoid Robots?

Humanoid robots are designed to mimic human morphology and movement. This human-like form offers several advantages:

1. **Environment Compatibility**: Operate in spaces designed for humans
2. **Tool Use**: Manipulate objects designed for human hands
3. **Social Interaction**: Natural communication with people
4. **Intuitive Control**: Leverage human motion data for training

## Robot Morphology

### Degrees of Freedom (DoF)

A **degree of freedom** is an independent direction of movement. Humans have approximately 244 DoF, but humanoid robots typically implement 20-50 DoF for practical reasons.

#### Common DoF Distribution

| Body Part | Typical DoF | Purpose |
|-----------|-------------|---------|
| Head | 2-3 | Pan, tilt, (roll) for perception |
| Torso | 1-3 | Bending, twisting |
| Each Arm | 7 | Shoulder (3), elbow (1), wrist (3) |
| Each Hand | 1-15 | Gripper (1) to dexterous (15+) |
| Each Leg | 6 | Hip (3), knee (1), ankle (2) |

**Example**: Boston Dynamics' Atlas has ~28 hydraulic joints.

### Kinematic Chain

A robot's structure can be represented as a kinematic chain:

```
Base (Pelvis)
├── Left Leg (6 DoF)
│   ├── Hip (3 DoF)
│   ├── Knee (1 DoF)
│   └── Ankle (2 DoF)
├── Right Leg (6 DoF)
├── Torso (3 DoF)
├── Left Arm (7 DoF)
│   └── Hand (1-15 DoF)
├── Right Arm (7 DoF)
│   └── Hand (1-15 DoF)
└── Head (2-3 DoF)
```

## Actuation Technologies

### 1. Electric Motors

**Advantages**:
- Precise position control
- Efficient for small to medium loads
- Easy to program

**Disadvantages**:
- Lower power-to-weight ratio than hydraulics
- Require gearing for high torque

**Example**: Tesla Optimus uses electric actuators

### 2. Hydraulic Actuators

**Advantages**:
- High force output
- Good power-to-weight ratio
- Fast response

**Disadvantages**:
- Require pumps and fluid management
- Potential leaks
- Noisy

**Example**: Boston Dynamics' Atlas

### 3. Pneumatic Actuators

**Advantages**:
- Compliant (soft, safe)
- Lightweight
- Inexpensive

**Disadvantages**:
- Difficult to control precisely
- Require compressed air supply
- Lower force than hydraulics

### 4. Series Elastic Actuators (SEA)

Combine motors with springs for:
- Shock absorption
- Force measurement
- Compliant interaction

Used in collaborative robots that work safely near humans.

## Locomotion

### Bipedal Walking

Walking on two legs is mechanically unstable but offers advantages:
- Navigate stairs and uneven terrain
- Free upper body for manipulation
- Match human environments

#### Static vs Dynamic Walking

**Static Walking**: Center of mass always within support polygon
- Slow but stable
- Used in early humanoid robots

**Dynamic Walking**: Uses momentum, briefly unstable
- Faster, more efficient
- Requires sophisticated control
- Natural human gait

#### Zero Moment Point (ZMP)

A key concept for bipedal balance:
- Point where net moment from ground reaction forces is zero
- Must stay within foot support region for stability
- Used for gait planning

```python
# Simplified ZMP calculation
def calculate_zmp(forces, moments):
    """
    forces: [fx, fy, fz] ground reaction forces
    moments: [mx, my, mz] moments
    """
    # ZMP in x-y plane
    zmp_x = -moments[1] / forces[2]
    zmp_y = moments[0] / forces[2]
    return (zmp_x, zmp_y)
```

### Balance Control

Humanoids maintain balance through:
1. **Ankle Strategy**: Small adjustments at ankle joints
2. **Hip Strategy**: Larger movements at hip
3. **Stepping Strategy**: Take a step to recover balance

## Manipulation

### Hand Design

#### 1. Parallel Gripper (1-2 DoF)
- Simple open/close mechanism
- Reliable for many tasks
- Limited dexterity

#### 2. Three-Finger Gripper (3-9 DoF)
- Better grasp variety
- More dexterity than parallel
- Easier to control than full hand

#### 3. Dexterous Hand (15-20 DoF)
- Human-like manipulation
- Complex control
- Expensive

### Grasp Planning

Common grasp types:
- **Power Grasp**: Whole hand wraps around object (robust)
- **Precision Grasp**: Fingertips pinch object (delicate)
- **Hook Grasp**: Fingers curl to hang object

## Sensing

### Proprioceptive Sensors

Measure robot's own state:
- **Joint Encoders**: Position and velocity
- **Force/Torque Sensors**: Interaction forces
- **IMU (Inertial Measurement Unit)**: Orientation and acceleration

### Exteroceptive Sensors

Perceive the environment:
- **Cameras**: RGB, depth (stereo, ToF)
- **Lidar**: 2D/3D mapping
- **Tactile Sensors**: Contact detection

### Sensor Fusion

Combining multiple sensors improves perception:
```
Vision + IMU → Better pose estimation
Force sensors + encoders → Compliant control
Depth + RGB → 3D object recognition
```

## Control Architecture

### Hierarchical Control

```
┌─────────────────────┐
│  Task Planning      │  ← High-level goals
├─────────────────────┤
│  Motion Planning    │  ← Generate trajectories
├─────────────────────┤
│  Inverse Kinematics │  ← Joint angles from position
├─────────────────────┤
│  Joint Control      │  ← PID/torque control
├─────────────────────┤
│  Actuators          │  ← Physical motors
└─────────────────────┘
```

### Inverse Kinematics (IK)

Given desired end-effector position, compute joint angles:

```python
# Simplified IK example (2D, 2-link arm)
import numpy as np

def inverse_kinematics_2d(x, y, l1, l2):
    """
    x, y: Target position
    l1, l2: Link lengths
    Returns: (theta1, theta2) joint angles
    """
    # Law of cosines
    d = np.sqrt(x**2 + y**2)
    cos_theta2 = (d**2 - l1**2 - l2**2) / (2 * l1 * l2)
    theta2 = np.arccos(cos_theta2)
    
    # Use geometry
    theta1 = np.arctan2(y, x) - np.arctan2(l2 * np.sin(theta2), 
                                             l1 + l2 * np.cos(theta2))
    return theta1, theta2
```

## Notable Humanoid Robots

### 1. Atlas (Boston Dynamics)
- 28 hydraulic joints
- 1.5m tall, 89kg
- Known for parkour, backflips

### 2. Tesla Optimus
- Electric actuators
- 1.73m tall, 57kg
- Focus on manufacturing tasks

### 3. Figure 01
- Electric actuators
- Full-body AI control
- Designed for warehouse work

### 4. Digit (Agility Robotics)
- Bipedal locomotion
- Package delivery focus
- Deployed in logistics

## Challenges in Humanoid Robotics

### 1. Mechanical Complexity
- Many joints require coordination
- Wear and maintenance
- Weight and power constraints

### 2. Control Complexity
- High-dimensional state space
- Real-time balance and manipulation
- Handling contact forces

### 3. Power Supply
- Batteries add weight
- Limited runtime (typically 1-2 hours)
- Tethered robots limit mobility

### 4. Cost
- High-precision actuators expensive
- Sensors and compute add up
- Maintenance and repairs costly

## Future Directions

- **AI-Driven Control**: End-to-end learning replacing classical control
- **Improved Actuators**: Lighter, more powerful motors
- **Better Sensing**: Tactile skin, better vision
- **Modularity**: Standardized components for easier assembly
- **Affordability**: Reducing costs for wider adoption

## Summary

Humanoid robots combine mechanical design, actuation, sensing, and control to create human-like physical agents. While challenging to build and control, their human-like form enables operation in human environments and interaction with human tools and spaces.

In the next chapter, we'll explore **ROS 2**, the Robot Operating System that provides the software infrastructure for controlling complex robots like humanoids.

## Further Reading

- Humanoid Robotics: A Reference (Kajita et al.)
- Introduction to Humanoid Robotics (Yoshida)
- Modern Robotics: Mechanics, Planning, and Control (Lynch & Park)
