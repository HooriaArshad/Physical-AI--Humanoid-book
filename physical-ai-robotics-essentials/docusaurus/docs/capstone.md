---
sidebar_position: 6
---

# Capstone Project

## Project Overview

In this capstone, you'll integrate everything you've learned to build a complete robotic system with:

- A humanoid robot simulation
- ROS2 for communication
- Digital twin for monitoring
- VLA model for intelligent control

## Project: Intelligent Object Manipulation Robot

### Objective

Build a robot that can:
1. Receive natural language instructions
2. Perceive the environment through vision
3. Plan and execute manipulation tasks
4. Adapt to novel objects and scenarios

### System Architecture

```
┌────────────────────────────────────────────────────────────────┐
│                    Capstone System Architecture                │
├────────────────────────────────────────────────────────────────┤
│                                                                │
│   ┌─────────────┐                                             │
│   │  User       │                                             │
│   │  Input      │                                             │
│   │  ("Pick up  │                                             │
│   │   the cup") │                                             │
│   └──────┬──────┘                                             │
│          │                                                    │
│          ▼                                                    │
│   ┌─────────────┐      ┌──────────────────────────────────┐  │
│   │  VLA Model  │─────►│  Task Planner                    │  │
│   │  (RT-2/OCTO)│      │  (Generate motion primitives)   │  │
│   └─────────────┘      └───────────────┬──────────────────┘  │
│                                        │                     │
│                                        ▼                     │
│   ┌────────────────────────────────────────────────────────┐ │
│   │                    ROS2 Control Layer                  │ │
│   │  ┌───────────┐  ┌───────────┐  ┌───────────────────┐   │ │
│   │  │ Arm Ctrl  │  │ Grip Ctrl │  │ Balance Controller │   │ │
│   │  └─────┬─────┘  └─────┬─────┘  └─────────┬─────────┘   │ │
│   │        └──────────────┼───────────────────┘             │ │
│   └───────────────────────┼───────────────────────────────────┘ │
│                           ▼                                     │
│   ┌────────────────────────────────────────────────────────┐  │
│   │              Robot Simulation (PyBullet)               │  │
│   │        ┌─────────────────────────────────────┐         │  │
│   │        │         Humanoid Robot              │         │  │
│   │        │  - 7-DOF arms                       │         │  │
│   │        │  - Gripper                          │         │  │
│   │        │  - Camera                           │         │  │
│   │        └─────────────────────────────────────┘         │  │
│   └────────────────────────────────────────────────────────┘  │
│                           │                                     │
│                           ▼                                     │
│   ┌────────────────────────────────────────────────────────┐  │
│   │                   Digital Twin                          │  │
│   │  - Real-time state visualization                       │  │
│   │  - Trajectory recording                                │  │
│   │  - Performance analytics                               │  │
│   └────────────────────────────────────────────────────────┘  │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

## Prerequisites

Before starting, ensure you have:

- [ ] ROS2 Humble or Iron installed
- [ ] Python 3.10+
- [ ] PyBullet or Isaac Sim
- [ ] CUDA-capable GPU (for VLA inference)
- [ ] 16GB+ RAM

## Project Structure

```
capstone_project/
├── src/
│   ├── vla_policy/
│   │   ├── __init__.py
│   │   ├── model.py           # VLA model wrapper
│   │   ├── processor.py       # Input preprocessing
│   │   └── inference.py       # Inference loop
│   │
│   ├── robot_control/
│   │   ├── __init__.py
│   │   ├── arm_controller.py  # Arm motion control
│   │   ├── gripper.py         # Gripper control
│   │   └── balance.py         # Balance controller
│   │
│   ├── perception/
│   │   ├── __init__.py
│   │   ├── camera.py          # Camera interface
│   │   ├── detection.py       # Object detection
│   │   └── segmentation.py    # Instance segmentation
│   │
│   └── task_planner/
│       ├── __init__.py
│       ├── planner.py         # High-level planning
│       ├── primitives.py      # Motion primitives
│       └── verifier.py        # Task verification
│
├── config/
│   ├── robot.yaml            # Robot parameters
│   ├── tasks.yaml            # Task definitions
│   └── camera.yaml           # Camera config
│
├── scripts/
│   ├── run_simulation.py     # Main simulation script
│   ├── collect_data.py       # Data collection
│   └── evaluate.py           # Evaluation script
│
├── tests/
│   ├── test_control.py
│   ├── test_perception.py
│   └── test_integration.py
│
├── data/
│   ├── models/               # Saved models
│   ├── episodes/             # Collected episodes
│   └── logs/                 # Execution logs
│
├── README.md
├── requirements.txt
└── setup.py
```

## Step-by-Step Implementation

### Phase 1: Setup and Environment

**Step 1.1: Create Workspace**

```bash
mkdir -p capstone_ws/src
cd capstone_ws
colcon init
```

**Step 1.2: Install Dependencies**

```bash
# In requirements.txt
torch>=2.0
transformers>=4.30
pybullet>=3.2
numpy>=1.24
opencv-python>=4.8
pyyaml>=6.0
asyncio
```

```bash
pip install -r requirements.txt
```

**Step 1.3: Test Simulation**

```python
# scripts/test_simulation.py
import pybullet as p

def test_simulation():
    # Connect
    p.connect(p.DIRECT)

    # Load robot
    robot_id = p.loadURDF("robot.urdf")

    # Verify
    assert robot_id >= 0, "Robot failed to load"
    print("Simulation ready!")

    p.disconnect()

if __name__ == "__main__":
    test_simulation()
```

### Phase 2: Robot Control

**Step 2.1: Arm Controller**

```python
# src/robot_control/arm_controller.py
import numpy as np
import pybullet as p

class ArmController:
    def __init__(self, robot_id, joint_indices):
        self.robot_id = robot_id
        self.joint_indices = joint_indices
        self.num_joints = len(joint_indices)

        # Control gains
        self.kp = 0.1
        self.kd = 0.01

    def set_joint_target(self, target_positions, duration=1.0):
        """Set target joint positions with smooth trajectory"""
        for i in range(int(duration * 240)):  # 240Hz control
            current = self.get_joint_positions()
            error = np.array(target_positions) - current
            velocity = error * self.kp - self.get_joint_velocities() * self.kd

            # Apply control
            p.setJointMotorControlArray(
                self.robot_id,
                self.joint_indices,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=velocity.tolist()
            )
            p.stepSimulation()

    def get_joint_positions(self):
        states = p.getJointStates(self.robot_id, self.joint_indices)
        return np.array([s[0] for s in states])

    def get_joint_velocities(self):
        states = p.getJointStates(self.robot_id, self.joint_indices)
        return np.array([s[1] for s in states])

    def get_ee_pose(self):
        """Get end-effector pose using FK"""
        # Simplified FK for 7-DOF arm
        # In practice, use robot-specific FK library
        return p.getLinkState(self.robot_id, 7)[0:2]
```

**Step 2.2: Gripper Controller**

```python
# src/robot_control/gripper.py
class GripperController:
    def __init__(self, robot_id, gripper_indices):
        self.robot_id = robot_id
        self.gripper_indices = gripper_indices
        self.open_width = 0.08  # meters
        self.closed_width = 0.0

    def open(self):
        self._set_width(self.open_width)

    def close(self, force=50):
        self._set_width(self.closed_width, max_effort=force)

    def _set_width(self, width, max_effort=30):
        """Set gripper to target width"""
        for joint in self.gripper_indices:
            p.setJointMotorControl2(
                self.robot_id, joint,
                controlMode=p.POSITION_CONTROL,
                targetPosition=width,
                maxVelocity=0.1,
                force=max_effort
            )
```

### Phase 3: VLA Integration

**Step 3.1: Model Wrapper**

```python
# src/vla_policy/model.py
import torch
from transformers import AutoModelForActionPrediction, AutoProcessor

class VLAPolicy:
    def __init__(self, model_path="openvla/openvla-7b"):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        self.model = AutoModelForActionPrediction.from_pretrained(
            model_path,
            torch_dtype=torch.bfloat16,
            low_cpu_mem_usage=True
        ).to(self.device)

        self.processor = AutoProcessor.from_pretrained(
            model_path,
            trust_remote_code=True
        )

    def predict(self, image, instruction):
        """Predict action from image and instruction"""
        # Preprocess
        inputs = self.processor(
            instruction,
            image,
            return_tensors="pt",
            do_rescale=False
        ).to(self.device)

        # Inference
        with torch.no_grad():
            action = self.model(**inputs)

        # Postprocess
        action_tokens = action.action_tokens[0]
        return self._tokens_to_joints(action_tokens)

    def _tokens_to_joints(self, tokens):
        """Convert discrete tokens to continuous joint commands"""
        # Map token indices to joint positions
        # Implementation depends on model
        return tokens
```

### Phase 4: Task Planning

**Step 4.1: Motion Primitives**

```python
# src/task_planner/primitives.py
import numpy as np

class MotionPrimitives:
    def __init__(self, arm_controller):
        self.arm = arm_controller

    def reach(self, target_pose, approach_height=0.1):
        """Reach to a target pose with approach"""
        # Move above target
        approach_pose = [target_pose[0], target_pose[1],
                        target_pose[2] + approach_height]
        self._move_to(approach_pose)

        # Move down to target
        self._move_to(target_pose)

    def grasp(self, object_pose):
        """Grasp at object position"""
        self.reach(object_pose)
        self.arm.gripper.close()

    def place(self, target_location):
        """Place object at target location"""
        # Lift
        lift_pose = [target_location[0], target_location[1],
                    target_location[2] + 0.2]
        self._move_to(lift_pose)

        # Move to target
        self._move_to(target_location)

        # Release
        self.arm.gripper.open()

    def _move_to(self, pose):
        """Move end-effector to pose (requires IK)"""
        # Convert pose to joint angles using IK
        joint_targets = self._ik_solver(pose)
        self.arm.set_joint_target(joint_targets)
```

### Phase 5: Main Integration

**Step 5.1: Main Script**

```python
# scripts/run_simulation.py
import pybullet as p
import cv2
from src.vla_policy.model import VLAPolicy
from src.robot_control.arm_controller import ArmController
from src.task_planner.planner import TaskPlanner

class CapstoneSystem:
    def __init__(self):
        # Initialize simulation
        p.connect(p.GUI)
        p.setGravity(0, 0, -9.81)

        # Load environment
        p.loadURDF("plane.urdf")
        self.robot_id = p.loadURDF("humanoid_robot.urdf")

        # Initialize components
        self.arm = ArmController(self.robot_id, joint_indices=[0,1,2,3,4,5,6])
        self.vla = VLAPolicy()
        self.planner = TaskPlanner(self.arm)

    def run(self, instruction):
        """Execute instruction using VLA + planning"""
        # Capture image
        image = self._capture_image()

        # Get VLA action prediction
        action = self.vla.predict(image, instruction)

        # Plan and execute
        trajectory = self.planner.plan(action)

        for step in trajectory:
            self.arm.set_joint_target(step)
            p.stepSimulation()

    def _capture_image(self):
        """Capture image from robot camera"""
        width, height, rgb_img, depth, seg = p.getCameraImage(
            width=224, height=224,
            viewMatrix=p.computeViewMatrix([0,0,1], [0,0,0]),
            projectionMatrix=p.computeProjectionMatrixFOV(60, 1, 0.1, 10)
        )
        return rgb_img

    def shutdown(self):
        p.disconnect()

if __name__ == "__main__":
    system = CapstoneSystem()

    try:
        while True:
            instruction = input("Enter instruction: ")
            system.run(instruction)
    finally:
        system.shutdown()
```

## Evaluation

### Success Metrics

| Task | Success Criteria |
|------|------------------|
| Pick up object | Object lifted 10cm |
| Place object | Object placed in target zone |
| Stack objects | Two objects stacked stably |
| Novel object | Success on unseen object |

### Test Scenarios

1. **Basic Manipulation**
   - "Pick up the red block"
   - "Place the cup on the table"

2. **Complex Tasks**
   - "Stack the blue box on the red box"
   - "Put the object in the bin"

3. **Generalization**
   - Novel objects not seen during training
   - New environments

## Bonus Challenges

1. **Add force control** for delicate manipulation
2. **Implement teleoperation** for data collection
3. **Train a custom policy** on your data
4. **Add multi-modal perception** (depth, tactile)
5. **Deploy on real hardware** (if available)

## Submission Requirements

- [ ] Working simulation with all components
- [ ] Documentation of system architecture
- [ ] Video demonstration of task execution
- [ ] Evaluation results on test scenarios
- [ ] Reflection on challenges and improvements

## Resources

- OpenVLA: https://github.com/openvla/openvla
- PyBullet: https://pybullet.org
- ROS2: https://docs.ros.org/en/humble/
- DROID Dataset: https://droid-dataset.github.io

## Summary

This capstone integrates Physical AI concepts into a complete robotic system. You'll experience the full pipeline from perception through VLA inference to physical execution, preparing you for real-world robotics development.
