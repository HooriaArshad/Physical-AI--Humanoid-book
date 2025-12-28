---
sidebar_position: 5
---

# VLA Models - Vision-Language-Action

## Introduction to VLA

Vision-Language-Action (VLA) models combine:
- **Vision**: Understanding visual input from cameras
- **Language**: Processing natural language instructions
- **Action**: Generating robot control commands

This enables robots to follow natural language instructions while interpreting visual scenes.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    VLA Model Architecture                   │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   ┌─────────┐    ┌─────────────┐    ┌─────────────────┐    │
│   │  Image  │───►│  Vision     │───►│  Fusion Layer   │    │
│   │  Input  │    │  Encoder    │    │  (Cross-attn)   │    │
│   └─────────┘    └─────────────┘    └────────┬────────┘    │
│                                               │             │
│   ┌─────────┐    ┌─────────────┐             │             │
│   │  Text   │───►│  Language   │─────────────┘             │
│   │ Command │    │  Encoder    │                          │
│   └─────────┘    └─────────────┘                          │
│                                               │             │
│                       ┌──────────────────────────▼─────┐   │
│                       │     Action Decoder           │   │
│                       │  (End-to-end policy)         │   │
│                       └──────────────┬───────────────┘   │
│                                      │                    │
│                         ┌────────────▼────────────┐      │
│                         │   Robot Actions         │      │
│                         │   (7-DOF trajectories)  │      │
│                         └─────────────────────────┘      │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## Popular VLA Models

| Model | Organization | Key Features |
|-------|--------------|--------------|
| **RT-2** | Google DeepMind | Web-scale pretraining, generalization |
| **RT-X** | Google DeepMind | Multi-embodiment training |
| **OpenVLA** | Stanford/NYU | Open-source, 7B parameters |
| **Octo** | Columbia/CMU | General-purpose policy |
| **GR-2** | NVIDIA | Video prediction + action |

## RT-2 Overview

RT-2 (Robotics Transformer 2) is a pioneering VLA model:

### Key Innovations

1. **Vision-Language Pretraining**: Trained on web-scale image-text data
2. **Fine-tuning for Robotics**: Adapted for robot control
3. **Co-finetuning**: Combined web and robot data
4. **Chain-of-Thought Reasoning**: Can reason about tasks

### Architecture

```python
# Conceptual RT-2 architecture
class RT2Model(nn.Module):
    def __init__(self):
        super().__init__()
        # Vision encoder (ViT)
        self.vision_encoder = ViT_Large()
        # Language encoder (PaLM)
        self.language_encoder = PaLM_Encoder()
        # Action head (auto-regressive)
        self.action_decoder = TransformerDecoder(
            d_model=1024,
            nhead=16,
            num_layers=6
        )

    def forward(self, image, text, actions=None):
        # Encode vision and language
        vision_emb = self.vision_encoder(image)
        language_emb = self.language_encoder(text)

        # Fuse representations
        fused = self.fusion_layer(vision_emb, language_emb)

        # Generate or predict actions
        if actions is not None:
            loss = self.action_decoder(fused, actions)
            return loss
        else:
            actions = self.action_decoder.generate(fused)
            return actions
```

## OpenVLA - Open Source Alternative

OpenVLA is a 7B parameter VLA model trained on:

- 970K robot episodes
- Open-source vision-language data

### Usage

```python
import torch
from transformers import AutoModelForActionPrediction, AutoProcessor

# Load model
model = AutoModelForActionPrediction.from_pretrained(
    "openvla/openvla-7b",
    torch_dtype=torch.bfloat16,
    low_cpu_mem_usage=True
)
processor = AutoProcessor.from_pretrained(
    "openvla/openvla-7b",
    trust_remote_code=True
)

# Inference
image = load_image("robot_view.jpg")
prompt = "put the apple on the plate"

inputs = processor(
    prompt,
    image,
    return_tensors="pt",
    do_rescale=False
)

with torch.no_grad():
    action = model(**inputs)
```

## Data Collection

### Collecting Robot Data

```python
class DataCollector:
    def __init__(self, save_path):
        self.save_path = save_path
        self.episodes = []

    def record_episode(self, teleop_data):
        episode = {
            'observations': [],
            'actions': [],
            'language': teleop_data['instruction']
        }

        for timestep in teleop_data['timesteps']:
            obs = {
                'image': timestep['image'],
                'depth': timestep['depth'],
                'joint_state': timestep['joint_positions'],
                'language_instruction': teleop_data['instruction']
            }
            action = {
                'joint_delta': timestep['joint_delta'],
                'gripper_action': timestep['gripper']
            }
            episode['observations'].append(obs)
            episode['actions'].append(action)

        self.episodes.append(episode)

    def save(self):
        torch.save(self.episodes, self.save_path)
```

### Data Format (DROID)

```python
# DROID dataset format
episode = {
    "episode_id": "episode_001",
    "language_instruction": "pick up the red block",
    "observations": [
        {
            "timestamp": 0.0,
            "image": {"rgb": "...", "depth": "..."},
            "state": {"joint_positions": [...], "ee_pose": ...},
        }
    ],
    "actions": [
        {
            "delta_joint_positions": [...],
            "gripper_command": 0 or 1,
        }
    ],
    "success": True
}
```

## Training VLA Models

### Training Pipeline

```python
# Conceptual training loop
def train_vla(model, dataloader, optimizer):
    model.train()
    total_loss = 0

    for batch in dataloader:
        images = batch['image']
        instructions = batch['language']
        actions = batch['action']

        # Forward pass
        loss = model(images, instructions, actions)
        total_loss += loss.item()

        # Backward pass
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

    return total_loss / len(dataloader)
```

### Training Tips

1. **Learning Rate**: 1e-5 to 1e-4 for fine-tuning
2. **Batch Size**: 8-32 depending on GPU memory
3. **Gradient Clipping**: 1.0 for stability
4. **Warmup**: Linear warmup for first 500 steps
5. **Mixed Precision**: Use bf16 for efficiency

## Inference and Deployment

### Real-Time Inference

```python
import asyncio
from asyncio import Queue

class VLAInference:
    def __init__(self, model, device="cuda"):
        self.model = model.to(device)
        self.device = device
        self.action_queue = Queue()

    async def process_frame(self, image, instruction):
        # Preprocess
        inputs = self.preprocess(image, instruction)

        # Inference
        with torch.no_grad():
            action = self.model(inputs)

        # Postprocess
        joint_command = self.postprocess(action)
        return joint_command

    async def run_loop(self, camera_stream):
        while True:
            image = await camera_stream.get_frame()
            instruction = await self.get_instruction()

            action = await self.process_frame(image, instruction)
            await self.robot.execute(action)
```

### Latency Optimization

| Technique | Benefit |
|-----------|---------|
| **Model Quantization** | 4x memory reduction |
| **TensorRT** | 2-5x speedup |
| **Action Chunking** | Reduce inference frequency |
| **Async Processing** | Hide latency |

## Evaluation Metrics

| Metric | Description |
|--------|-------------|
| **Success Rate** | Percentage of completed tasks |
| **Goal Condition Success** | Did robot achieve goal? |
| **Plan Success** | Did robot follow instructions? |
| **Efficiency** | Time to completion |
| **Safety** | Collision avoidance |

## Challenges and Limitations

1. **Sim-to-Real Gap**: Differences between simulation and real world
2. **Generalization**: Performance on novel objects/scenes
3. **Safety**: Reliable operation in all conditions
4. **Compute**: Large models require powerful hardware
5. **Data**: High-quality robot data is expensive to collect

## Future Directions

- **Foundation Models**: General-purpose robot policies
- **Multi-Modal Learning**: Touch, audio, and more
- **Continuous Learning**: Adapting to new tasks
- **Safety Verification**: Formal guarantees

## Exercises

1. Fine-tune OpenVLA on a small dataset
2. Implement a data collection interface
3. Evaluate model generalization on novel objects
4. Optimize inference latency with quantization

## Summary

VLA models represent a major advance in robot learning, enabling natural language control of robots that can understand visual scenes. By combining vision, language, and action, these models can generalize to new tasks and environments in ways that were previously impossible.
