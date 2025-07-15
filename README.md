# Control Barrier Function for Tactical Conflict Resolution

## Overview

This simulation models **Tactical Conflict Resolution** for a grid-based urban airspace where aircraft (represented as 2D point-mass vehicles) travel along orthogonal corridors. The core logic implements **Control Barrier Functions (CBFs)** to modulate vehicle speeds and maintain **safe separation** at intersections. The control is **centralised and ground-based**, making it ideal for **UAS Traffic Management** systems operating in structured low-altitude corridors.

---

## Key Concepts and Notation

Let:

- $i, j \in [1, \ldots, N ]$ index the vehicles  
- $\mathbf{p}_i(t) \in \mathbb{R}^2$: position of vehicle *i* at time *t*  
- $s_i(t) \in \mathbb{R}$: scalar path parameter for vehicle *i*  
- $v_i(t) = \dot{s}_i(t)$: scalar speed of vehicle *i*  
- $\mathcal{L}_{ij} = \|\mathbf{p}_i(t) - \mathbf{p}_j(t)\|^2 - d^2$: safety constraint, with $d > 0$ as the **loss-of-separation** threshold

A **Control Barrier Function (CBF)** is defined for each conflicting pair $(i, j)$ as:

```math
h_{ij}(t) = \|\mathbf{p}_i(t) - \mathbf{p}_j(t)\|^2 - d^2
```

To ensure safety, we enforce the CBF condition:

```math
\dot{h}_{ij}(t) + \alpha h_{ij}(t) \geq 0
```

where $\alpha > 0$ is a tunable CBF gain.

The goal is to find an admissible control (speed) $v_i(t)$ such that **all pairwise CBF constraints are satisfied**.

---

## Script Structure

### `main()`

Initialises the simulation, runs each timestep, and visualises the result.

### `Simulation` class

Encapsulates simulation logic:
- Detects potential conflicts
- Computes feasible speed intervals $[LB_i, UB_i]$
- Applies clipped speed control:  
  $v_i(t) = \text{clip}(v_{\text{nominal}}, LB_i, UB_i)$


### `ControlBarrier`

Holds the LoS threshold and the gain $\alpha$ used in the inequality constraint.

### `Traffic`

Tracks scalar path positions, previous velocities, and full position histories.

### `Airspace`

Defines a structured grid of flight paths. Horizontal and vertical vehicles are assigned based on their index.

---

## Conflict Detection Logic

Only **orthogonal vehicles** may conflict (horizontal vs vertical).

Each vehicle computes its **arrival time** at the intersection:

$t_i = \frac{s^\text{int}_i - s_i}{v_i}, \quad t_j = \frac{s^\text{int}_j - s_j}{v_j}$

A conflict is resolved **only if** $t_i > t_j$, assigning priority to vehicle *j*.

The CBF condition then yields a linear constraint on $v_i$, used to adjust its bounds for safety.

---

## Visualisation

The plot includes:
- Corridor boundaries as dashed lines
- Real-time vehicle positions
- Vehicle markers with unique colours
- An animated playback of the simulation

---

## How to Run

```bash
python3 controlBarrierFunction.py
```

Ensure dependencies are installed:

```bash
pip install numpy matplotlib
```

---

## Academic Reference

This implementation demonstrates a **discrete-time, single-integrator CBF method** for tactical deconfliction in urban airspace corridors. It is suitable for fast-time simulation and evaluation of **centralised ground-based safety nets** in UTM.

> Frémond, R. (2025). *Centralised Tactical Conflict Resolution Using Control Barrier Functions in Grid-Based Airspace*. Internal Project Note.
>

**Keywords**: Control Barrier Function, Tactical Conflict Resolution, Unmanned Aircraft System Traffic Management (UTM), Separation Assurance, Grid-Based Airspace.

---
