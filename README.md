# Quarter-Car Suspension Dynamics & Optimization

A Python-based computational model of a 2-DOF quarter-car suspension system used to investigate how suspension stiffness and damping affect vehicle dynamic response to road disturbances.

## Engineering Question

How can suspension stiffness and damping ratio be optimized to reduce vibration while maintaining controlled suspension travel when subjected to road disturbances?

## Project Overview

This project models a quarter-car suspension system consisting of a sprung mass, unsprung mass, suspension spring and damper, and tire stiffness.

The simulation evaluates the response of the system to a modeled road bump and performs a parametric sweep across 25 combinations of suspension stiffness and damping ratio.

### Design Variables

* Suspension stiffness, `k₁`: 15,000–25,000 N/m
* Damping ratio, `ζ₁`: 0.20–0.40

### Fixed Parameters

* Sprung mass: 400 kg
* Unsprung mass: 30 kg
* Tire stiffness: 200,000 N/m
* Bump height: 0.2 m
* Bump length: 0.8 m
* Vehicle speed: 0.85 m/s

## Simulation Method

The suspension dynamics are represented using a state-space formulation and solved numerically using `scipy.integrate.odeint`.

The model evaluates:

1. Maximum body displacement
2. RMS body acceleration
3. Settling time
4. Maximum suspension travel

## Results

The parameter sweep demonstrates a multi-objective tradeoff between ride isolation and suspension control.

Lower suspension stiffness generally reduces body displacement and acceleration, while higher stiffness reduces suspension travel and generally improves recovery time. Increasing damping improved all four measured metrics within the tested parameter range.

No single configuration minimized every metric simultaneously, demonstrating the tradeoff between ride comfort and suspension control.

## Key Findings

### Ride Comfort

The configuration with:

* `k₁ = 15,000 N/m`
* `ζ₁ = 0.40`

produced the lowest maximum body displacement and RMS body acceleration among the tested configurations.

### Suspension Control

The configuration with:

* `k₁ = 25,000 N/m`
* `ζ₁ = 0.40`

produced the lowest settling time and maximum suspension travel among the tested configurations.

## Engineering Interpretation

The results show that suspension design is a multi-objective problem. A configuration that improves ride isolation may not simultaneously minimize suspension travel or recovery time.

The preferred suspension characteristics therefore depend on the intended vehicle application and its performance priorities.

## Future Work

Future development will extend the computational model toward physical validation using a bench-scale suspension test platform and Arduino-based instrumentation.

Planned extensions include:

* Physical acceleration and displacement measurements
* Comparison between simulated and experimental results
* Additional terrain profiles
* Repeated and irregular road disturbances
* Expanded parameter studies
* Application to robotics and off-road vehicle systems

## Tools

* Python
* NumPy
* SciPy
* Matplotlib
