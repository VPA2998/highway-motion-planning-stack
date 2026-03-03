# 🛣️ Highway Motion Planning Stack

**Autonomous highway driving simulation with Lanelet mapping, A* routing, FSM behavior planning, IDM longitudinal control, and Frenet lattice trajectory generation.**

![Python](https://img.shields.io/badge/Python-3.8%2B-blue)
![License](https://img.shields.io/badge/License-MIT-green)

---

## 📋 Overview

This project implements a complete motion planning stack for autonomous highway driving in a single Jupyter notebook. It demonstrates:

- **Map & Routing**: Lanelet-style straight highway representation with A* global route planning
- **Vehicle Dynamics**: Dynamic bicycle model for ego vehicle, kinematic models for traffic
- **Behavior Planning**: Route-aware Finite State Machine (FSM) with states: CRUISE, FOLLOW, STOP, LANE_CHANGE_LEFT, LANE_CHANGE_RIGHT
- **Longitudinal Control**: Intelligent Driver Model (IDM) for car-following behavior
- **Local Planning**: Frenet lattice planner using quintic polynomials for lateral trajectories
- **Collision Detection**: Oriented rectangle representation with Separating Axis Theorem (SAT)
- **Visualization**: Gradio interactive app, GIF animation generation, CSV trajectory logging

---

## 📁 Project Structure

```
highway-motion-planning-stack/
├── notebooks/
│ └── 01_highway_motion_planning_full_stack.ipynb # Main implementation
├── src/ # Future: modular Python packages
├── data/ # Map data, scenario configs
├── outputs/ # Generated GIFs, CSV logs
├── docs/ # Documentation, diagrams
├── .gitignore
├── requirements.txt
└── README.md
```
---

## 🚀 Quick Start

### 1. Clone the repository

```bash
git clone https://github.com/VPA2998/highway-motion-planning-stack.git
cd highway-motion-planning-stack
```
### 2. Create virtual environment

```bash
python3 -m venv .venv
source .venv/bin/activate  # On WSL/Linux
# or: .venv\Scripts\activate  # On Windows
```

### 3. Install dependencies

```bash
pip install -r requirements.txt
```

### 4. Launch Jupyter

```bash
jupyter notebook
```

### 5. Open and run the notebook

- Navigate to [notebooks](notebooks/Project3_Motion_Planning.ipynb) and run all cells.
---

## 🎯 Key Features

### Lanelet Map Representation

- Straight 3-lane highway with configurable lanelet connections

- A* search on lanelet graph for optimal lane sequence

### Finite State Machine (FSM)
#### **Route-aware behavior states:**

- CRUISE: Maintain desired speed in current lane

- FOLLOW: Car-following with leading vehicle (IDM)

- STOP: Emergency stop when collision risk detected

- LANE_CHANGE_LEFT/RIGHT: Execute safe lane changes

### IDM Longitudinal Control
- Realistic car-following behavior

- Parameters: desired speed, time gap, min gap, acceleration, comfortable deceleration

### Frenet Lattice Planner
- Quintic polynomial trajectories in Frenet frame

- Lateral offset and velocity profiling

- Cost-based trajectory selection (smoothness, safety, efficiency)

### Collision Checking
- Vehicle represented as oriented rectangles

- Separating Axis Theorem (SAT) for precise collision detection

- Real-time safety verification


## 📊 Outputs

The notebook generates:

- GIF Animation: Visualization of ego vehicle and traffic over time

- CSV Log: Timestamped trajectory data (t, x, y, lane_index, speed, behavior)

- Gradio App: Interactive scenario exploration (optional)


