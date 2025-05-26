# Autonomous Tractor Parking Simulation using Nonlinear and PI Control
This repository contains the code and simulation models developed as part of my research on autonomous parking for Amiga farming tractors. The project applies nonlinear control theory (using Lyapunov stability principles) and Proportional-Integral (PI) feedback controllers in both MATLAB and CoppeliaSim environments to achieve precise trajectory tracking and parking maneuvers for differential-drive vehicles.

## Project Summary
- **Objective**: Enable autonomous parking of a differential-drive robotic tractor using a combination of nonlinear control and PI feedback.
- **Control Strategy**: Lyapunov-based stability theory for nonlinear trajectory tracking; PI controllers for low-level velocity and turning rate regulation.
- **Simulation Tools**: 
  - **MATLAB** for theoretical modeling and trajectory visualization.
  - **CoppeliaSim (V-REP)** for simulating physical robot behavior and logging real-time data.

---

## Control Theory
This project leverages a control strategy based on Lyapunov stability analysis, detailed in the paper:

The nonlinear controller used is based on Lyapunov stability analysis. The Lyapunov candidate function is:

$` V = \frac{1}{2} \lambda e^2 + \frac{1}{2} (\alpha^2 + h\theta^2) `$; $`\;`$ $`\lambda, h > 0`$

From this, the control laws are derived as:
- **Velocity Reference**: $` u\_{REF} = \gamma * \cos(\alpha) * e`$
- **Angular Velocity Reference**: $` \omega\_{REF} = k\alpha + \gamma\cos(\alpha)(\frac{\sin(\alpha)}{\alpha})(\alpha + h\theta)`$
