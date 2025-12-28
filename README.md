# 🤖 Physical AI & Humanoid Robotics — Essentials

<div align="center">

![Physical AI Banner](https://img.shields.io/badge/Physical%20AI-Robotics-blue?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Active-success?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)

**A comprehensive textbook and interactive platform for learning Physical AI and Humanoid Robotics**

Created by **[Hooria Arshad](https://github.com/HooriaArshad)**

[📖 Read Online](https://hooriaarshad.github.io/docasuros-book/) | [🚀 Get Started](#-quick-start) | [📚 Documentation](#-project-structure)

</div>

---

## 📋 Table of Contents

- [About](#-about)
- [Features](#-features)
- [Project Structure](#-project-structure)
- [Quick Start](#-quick-start)
- [Development](#-development)
- [Deployment](#-deployment)
- [Technologies](#-technologies)
- [Contributing](#-contributing)
- [Author](#-author)
- [License](#-license)

---

## 🌟 About

This project is an **interactive textbook** that introduces students and developers to the world of **Physical AI** and **Humanoid Robotics**. It combines theoretical knowledge with practical implementations, covering everything from ROS 2 fundamentals to Vision-Language-Action systems.

### What You'll Learn

- 🤖 **Physical AI**: AI systems that interact with the physical world
- 🦾 **Humanoid Robotics**: Robot design, actuation, and control
- 🔧 **ROS 2**: Robot Operating System for modern robotics
- 🖥️ **Digital Twin Simulation**: Gazebo and NVIDIA Isaac Sim
- 👁️ **Vision-Language-Action (VLA)**: AI models controlling robots
- 🏗️ **Capstone Projects**: End-to-end AI-robot pipelines

---

## 📖 Book Chapters

### Preface: The Future is Embodied
Physical AI marks the convergence of artificial intelligence and robotics — where digital intelligence steps into the physical world.

**Reading Time**: ~5 minutes

---

### Chapter 1: Introduction to Physical AI
Learn about AI systems that interact with the physical world through embodied agents like robots.

**Key Topics**:
- What is Physical AI and why it matters
- Core components: Perception, Reasoning, Actuation
- The Physical AI Stack (from AI/ML to hardware)
- Challenges: Sim-to-Real gap, Safety, Generalization
- Recent breakthroughs: VLA models, Foundation Models, End-to-End Learning

**Reading Time**: ~20 minutes

---

### Chapter 2: Basics of Humanoid Robotics
Explore humanoid robot design, actuation, sensing, and control systems.

**Key Topics**:
- Why humanoid robots? (Environment compatibility, tool use)
- Robot morphology and Degrees of Freedom (DoF)
- Actuation technologies: Electric, Hydraulic, Pneumatic, Series Elastic
- Bipedal locomotion: Static vs Dynamic walking, Zero Moment Point (ZMP)
- Hand design and grasp planning
- Notable robots: Atlas, Tesla Optimus, Figure 01, Digit

**Reading Time**: ~25 minutes

---

### Chapter 3: ROS 2 Fundamentals
Master the Robot Operating System 2, the middleware framework for modern robotics.

**Key Topics**:
- ROS 2 core concepts: Nodes, Topics, Services, Actions
- Publish/subscribe pattern for sensor data
- Request/response for synchronous operations
- Creating publishers and subscribers in Python
- Common ROS 2 tools and commands

**Reading Time**: ~30 minutes

---

### Chapter 4: Digital Twin Simulation
Learn how to test and train robots in virtual environments before real-world deployment.

**Key Topics**:
- What is a Digital Twin and why simulation matters
- **Gazebo Simulator**: Open-source, ROS 2 integrated
  - Realistic physics, sensor simulation, plugin system
- **NVIDIA Isaac Sim**: Photorealistic, AI-powered
  - RTX rendering, PhysX physics, domain randomization
- Sim-to-Real transfer techniques
- Bridging the simulation-reality gap

**Reading Time**: ~25 minutes

---

### Chapter 5: Vision-Language-Action Systems
Discover how AI models understand instructions and control robots through natural language.

**Key Topics**:
- What are VLA systems? (Vision + Language + Action)
- Architecture: Vision encoder → Language model → Action decoder
- Key models: RT-1, RT-2, PaLM-E, Open X-Embodiment
- Training approaches: Behavior cloning, RL, Pre-training + Fine-tuning
- Challenges: Data efficiency, long-horizon tasks, safety
- Practical deployment examples

**Reading Time**: ~25 minutes

---

### Chapter 6: Capstone Project
Build an end-to-end AI-robot pipeline that integrates all concepts from previous chapters.

**Project Goal**: Create a system where a robot arm picks and places objects based on natural language instructions.

**Components**:
- Perception: RGB-D camera + object detection (YOLOv8)
- Planning: MoveIt 2 for motion planning + grasp planning
- Control: ROS 2 nodes + joint trajectory controller
- Simulation first: Test in Gazebo with domain randomization

**Reading Time**: ~20 minutes

---

**Total Reading Time**: 145-170 minutes

---

## ✨ Features

### 📖 Interactive Textbook (Docusaurus)
- 6 comprehensive chapters covering Physical AI & Robotics
- Reading time: ~145-170 minutes
- Search functionality and responsive design
- Dark/Light mode support

### 💬 AI-Powered Chatbot (Backend)
- Real-time Q&A about course content
- Context-aware responses using OpenAI
- WebSocket-based communication
- Vercel-ready deployment

### ⚛️ Modern Frontend (React + Vite)
- Fast development with Vite
- TypeScript for type safety
- Responsive UI with modern styling
- Component-based architecture

---

## 📁 Project Structure

```
Physical-AI--Humanoid-book/
├── docs/                    # 📖 Textbook content (Markdown)
│   ├── preface.md
│   ├── chapters/
│   │   ├── physical-ai.md
│   │   ├── humanoid-robotics.md
│   │   ├── ros2-fundamentals.md
│   │   ├── digital-twin.md
│   │   ├── vla-systems.md
│   │   └── capstone.md
│   └── index.md
│
├── frontend/                # ⚛️ React + Vite frontend
│   ├── src/
│   │   ├── App.tsx
│   │   ├── main.tsx
│   │   └── components/
│   ├── package.json
│   └── vite.config.ts
│
├── backend/                 # 🤖 AI Chatbot backend
│   ├── api/
│   │   └── index.js        # Express + WebSocket server
│   ├── package.json
│   └── vercel.json
│
├── src/                     # 🎨 Docusaurus theme customization
│   ├── css/
│   └── components/
│
├── static/                  # 📦 Static assets
│   └── img/
│
├── docusaurus.config.js     # ⚙️ Docusaurus configuration
├── sidebars.js             # 📑 Sidebar structure
└── package.json            # 📦 Root dependencies
```

---

## 🚀 Quick Start

### Prerequisites

- Node.js (v18 or higher)
- npm or yarn
- Git

### 1️⃣ Clone the Repository

```bash
git clone https://github.com/HooriaArshad/Physical-AI--Humanoid-book.git
cd Physical-AI--Humanoid-book
```

### 2️⃣ Install Dependencies

**For Docusaurus (Textbook):**
```bash
npm install
```

**For Frontend:**
```bash
cd frontend
npm install
```

**For Backend:**
```bash
cd backend
npm install
```

### 3️⃣ Run Development Servers

**Docusaurus (Port 3000):**
```bash
npm start
```

**Frontend (Port 3000):**
```bash
cd frontend
npm run dev
```

**Backend (Port 3001):**
```bash
cd backend
npm run dev
```

---

## 💻 Development

### Docusaurus Development

```bash
# Start development server
npm start

# Build for production
npm run build

# Serve production build locally
npm run serve

# Clear cache
npm run clear
```

### Frontend Development

```bash
cd frontend

# Start Vite dev server
npm run dev

# Build for production
npm run build

# Preview production build
npm run preview

# Lint code
npm run lint
```

### Backend Development

```bash
cd backend

# Start development server with hot reload
npm run dev

# Start production server
npm start

# Run tests (if configured)
npm test
```

---

## 🌐 Deployment

### Docusaurus (GitHub Pages)

```bash
# Build and deploy to GitHub Pages
npm run deploy
```

**Live URL**: https://hooriaarshad.github.io/docasuros-book/

### Backend (Vercel)

```bash
cd backend
vercel deploy
```

### Frontend (Netlify/Vercel)

```bash
cd frontend
npm run build
# Deploy the 'dist' folder
```

---

## 🛠️ Technologies

### Textbook
- **[Docusaurus](https://docusaurus.io/)** - Documentation framework
- **React** - UI library
- **MDX** - Markdown with JSX

### Frontend
- **[React 18](https://react.dev/)** - UI library
- **[Vite](https://vitejs.dev/)** - Build tool
- **[TypeScript](https://www.typescriptlang.org/)** - Type safety
- **CSS3** - Styling

### Backend
- **[Node.js](https://nodejs.org/)** - Runtime
- **[Express](https://expressjs.com/)** - Web framework
- **[Socket.io](https://socket.io/)** - WebSocket library
- **[OpenAI API](https://openai.com/)** - AI-powered responses
- **[Vercel](https://vercel.com/)** - Serverless deployment

---

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

### Steps to Contribute

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

---

## 👩‍💻 Author

**Hooria Arshad**

Passionate about Physical AI, Humanoid Robotics, and building educational resources.

<div align="center">

[![GitHub](https://img.shields.io/badge/GitHub-181717?style=for-the-badge&logo=github)](https://github.com/HooriaArshad)
[![LinkedIn](https://img.shields.io/badge/LinkedIn-0077B5?style=for-the-badge&logo=linkedin)](https://www.linkedin.com/in/hooria-arshad-hoor-ba32b5301/)
[![Instagram](https://img.shields.io/badge/Instagram-E4405F?style=for-the-badge&logo=instagram)](https://www.instagram.com/call_me_hoora_/)

</div>

---

## 📄 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

- ROS 2 Community
- NVIDIA Isaac Sim Team
- Docusaurus Team
- Open Source Community

---

<div align="center">

**⭐ Star this repository if you find it helpful!**

Made with ❤️ by [Hooria Arshad](https://github.com/HooriaArshad)

</div>
