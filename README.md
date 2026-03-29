# Pattern-Drawing Robot — MATLAB & CoppeliaSim

A simulated 3-DOF Cartesian gantry robot that draws geometric patterns and even writes text, controlled entirely from MATLAB through the CoppeliaSim ZeroMQ Remote API.

Think of it as a pen plotter — three prismatic joints (X, Y, Z) move a pen-tip end-effector along pre-generated trajectories. The Z-axis lifts the pen between strokes. The robot draws circles, squares, diamonds, and can even write letters.

---

## The control story (the interesting part)

The first attempt used a classical PID controller — the obvious choice for joint position regulation. It failed, and for a non-obvious reason.

CoppeliaSim 4.6 models prismatic joints as **kinematic constraints**, not motorized actuators. There's no velocity interface, no torque input, no internal motor dynamics. Sending PID output as a velocity command either snaps the joint instantly, causes oscillation, or does nothing — depending on timing. Tuning couldn't fix it because the problem was architectural.

The working solution was to stop treating it like a motor control problem and treat it like a **CNC machine** instead. At each timestep, the joint is moved a small bounded step toward the target:

```matlab
dq    = q_desired - q_current;
step  = sign(dq) .* min(abs(dq), maxStep);
q_new = q_current + step;
sim.setJointTargetPosition(hX, q_new(1));
sim.setJointTargetPosition(hY, q_new(2));
sim.setJointTargetPosition(hZ, q_new(3));
```

This is exactly how GRBL and Marlin firmware work on real plotters — feedrate-limited incremental position commands rather than torque control. The result is smooth, deterministic, CNC-like motion with zero tuning.

---

## What's in the repo

| File | What it does |
|---|---|
| `coppeliasim_cartesian_remote.m` | **Core script** — square trajectory, incremental kinematic controller |
| `cartesian_robot_sim.m` | **Enhanced version** — pen up/down signals, joint limits, safety clamps, logging, plots |
| `circle_PID.m` | The failed PID attempt — kept for comparison and learning |
| `draw_diamond.m` | Draws a diamond with full pen-up/pen-down travel moves |
| `draw_name.m` | Writes "ADIL" letter by letter using stroke-based motion |
| `cartesian_3dof_draw.m` | Pure MATLAB dynamics simulation — no CoppeliaSim needed, PD control on a mass-damper model |
| `cartesian_pen_scene.ttt` | CoppeliaSim scene file — open this before running any script |

---

## Requirements

- MATLAB R2021a or later
- CoppeliaSim 4.6 EDU — [download](https://www.coppeliarobotics.com)
- ZeroMQ Remote API for MATLAB (included with CoppeliaSim, add to MATLAB path)

---

## Running it

1. Open `cartesian_pen_scene.ttt` in CoppeliaSim — **don't start the simulation manually**
2. Add the ZeroMQ Remote API folder to your MATLAB path
3. Run whichever script you want — it connects, starts the simulation, draws, and stops automatically

**Recommended starting point:** `cartesian_robot_sim.m` — the most complete version with pen control, safety limits, and logged plots.

**Want to see why PID failed?** Run `circle_PID.m` and watch what happens.

---

## Key parameters

```matlab
dt       = 0.02;    % control timestep (50 Hz)
T        = 12;      % trajectory duration (s)
maxStep  = 0.003;   % max joint displacement per step — controls drawing speed
Zdown    = 0.10;    % pen-on-surface height (m)
Zup      = 0.25;    % pen-lifted height (m)
```

---

## Trajectories implemented

- **Square** — four linear segments with corner Z-lifts
- **Circle** — parametric (cos/sin) with uniform angular velocity
- **Diamond** — four diagonal segments with pen-up travel moves between strokes
- **"ADIL"** — each letter drawn stroke-by-stroke with pen-up travel between characters
- **Letter A dynamics** — piecewise stroke trajectory in the standalone MATLAB model

Adding new shapes just means generating a new (x, y, z) waypoint sequence — the controller doesn't care what shape it's following.

---

## Standalone MATLAB simulation (`cartesian_3dof_draw.m`)

No CoppeliaSim needed for this one. Runs a full dynamics simulation in MATLAB — mass-damper model per axis, PD control, Euler integration, and plots. Switch between `'circle'` and `'letterA'` at the top.

```
RMSE  X: ~0.001 m    Y: ~0.001 m    Z: ~0.0003 m
```

---

## Hardware equivalent

The incremental controller maps directly to stepper motor firmware:

```
Δq (meters)  →  Δq × steps/mm  (step pulses to driver)
```

A physical build would use NEMA 17 steppers, GT2 belt drives, and GRBL or Marlin on an Arduino or STM32. The control philosophy is identical — the simulation was designed with this transition in mind.

---

## About

Built as a micro-project for the Kinematics, Dynamics and Control of Robots course (221TIA003) at the College of Engineering Trivandrum, M.Tech Robotics and Automation. Supervised by Prof. Merlin Mon Mathew.

**Author:** Adil Naz Muhammed | [LinkedIn](https://www.linkedin.com/in/adil-naz-muhammed) | [Email](mailto:adilnazmuhammed.mec@gmail.com)
