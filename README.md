# Control Barrier Function for Tactical Conflict Resolution

## Overview

This simulation models **Tactical Conflict Resolution (TCR)** for a grid-based urban airspace where aircraft (represented as 2D point-mass vehicles) travel along orthogonal corridors. The core logic implements **Control Barrier Functions (CBFs)** to modulate vehicle speeds and maintain **safe separation** at intersections. The control is **centralised and ground-based**, making it ideal for **UAS Traffic Management (UTM)** systems operating in structured low-altitude corridors.

---

## Key Concepts and Notation

Let:

- \( i, j \in \{1, \ldots, N\} \) index the vehicles
- \( \mathbf{p}_i(t) \in \mathbb{R}^2 \): position of vehicle *i* at time *t*
- \( s_i(t) \in \mathbb{R} \): scalar path parameter for vehicle *i*
- \( v_i(t) = \dot{s}_i(t) \): scalar speed of vehicle *i*
- \( \mathcal{L}_{ij} = \|\mathbf{p}_i(t) - \mathbf{p}_j(t)\|^2 - d^2 \): safety constraint, with \( d > 0 \) as the **loss-of-separation (LoS)** threshold

A **Control Barrier Function (CBF)** is defined for each conflicting pair \( (i, j) \) as:

```math
h_{ij}(t) = \|\mathbf{p}_i(t) - \mathbf{p}_j(t)\|^2 - d^2
