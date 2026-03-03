# 🚀 Quick Start Guide

Get the **Highway Motion Planning Stack** running in under 5 minutes!

---

## ⚡ Prerequisites

- **Python 3.8+** (tested on 3.10)
- **pip** (Python package manager)
- **Git**

---

## 📥 Step 1: Clone the Repository

```bash
git clone https://github.com/VPA2998/highway-motion-planning-stack.git
cd highway-motion-planning-stack
```
## 🛠️ Step 2: Create Virtual Environment

### On Linux/WSL/macOS:
```bash
python3 -m venv .venv
source .venv/bin/activate
```
### On Windows (PowerShell):
```powershell
python -m venv .venv
.venv\Scripts\Activate
```

- You should see `(.venv)` in your terminal prompt.

## 📦 Step 3: Install Dependencies
```bash
pip install -r requirements.txt
```
-> **Installed packages:**

- `numpy` – Numerical computing

- `matplotlib` – Visualization

- `networkx` – Graph algorithms (A* routing)

- `gradio` – Interactive web UI

- `pillow` – GIF generation

- `jupyter` – Notebook environment

- `scipy` – Scientific utilities

## 🎬 Step 4: Run the Simulation

### Option A: Jupyter Notebook (Recommended)
```bash
jupyter notebook
```

This will open a browser window. Navigate to:

```text
notebooks/01_highway_motion_planning_full_stack.ipynb
```
> - Click "Kernel" → "Restart & Run All" to execute the entire simulation.

**What happens:**

1. Creates a 3-lane highway map (500m)

2. Spawns ego vehicle + 20 traffic vehicles

3. Plans global route using A*

4. Simulates 40 seconds of driving with:

    - FSM behavior decisions

    - IDM longitudinal control

    - Frenet lattice trajectory generation

    - SAT collision avoidance

5. Generates a GIF animation and CSV trajectory log
---
### Option B: Command Line (Future – After Modular Refactoring)
*Currently under development. Once complete:*

```bash
python src/main.py --road-length 500 --num-vehicles 20 --seed 0
```
---

## 📊 Step 5: View Outputs
### GIF Animation
After running the notebook, a GIF is generated in a temporary folder. The notebook cell will print the path, e.g.:

```text
/tmp/tmpXXXXXX/highwaysimdynbicyclerectcoll.gif
```

**To save it permanently:**

```bash
cp /tmp/tmpXXXXXX/highwaysimdynbicyclerectcoll.gif outputs/demo/my_simulation.gif
```
### CSV Trajectory Log

The notebook also exports ego_trajectory.csv with columns:

- `t` – Timestamp (seconds)

- `x, y` – World coordinates (meters)

- `lane_index` – Current lane (0, 1, 2)

- `speed` – Velocity (m/s)

- `behavior` – FSM state (CRUISE, FOLLOW, etc.)


## 🎮 Step 6: Interactive Gradio App (Optional)

In the notebook, scroll to the Gradio App section and run the cell:

```python
demo.launch(share=True)
```

This launches a web-based UI where you can:

- Adjust road length, traffic density, seed

- Change ego start/goal positions

- See real-time GIF + speed plot + trajectory CSV

Access the app via the public URL shown (expires in 1 week) or local URL.

## 🐛 Troubleshooting
`Issue:` ModuleNotFoundError: No module named 'numpy'
`Fix:` Ensure virtual environment is activated:

```bash
source .venv/bin/activate  # Linux/WSL
.venv\Scripts\Activate     # Windows
```
`Issue:` GIF not generating
`Fix:` Check if pillow is installed:

```bash
pip install pillow
```
`Issue:` Gradio share link not working
`Fix:` Update Gradio:

```bash
pip install --upgrade gradio
```
`Issue:` Slow performance
`Fix:` Reduce traffic in simulation parameters:

```python
num_vehicles = 10  # Instead of 20 
```

## 📚 Next Steps
🏗️ Read [`System Architecture`](ARCHITECTURE.md) for module details

📖 Explore individual [`src/`](../src) modules for standalone usage

🔮 Try modifying parameters (road length, traffic density, seed)

🤝 Fork and extend for your own scenarios (merging, exits, construction zones)