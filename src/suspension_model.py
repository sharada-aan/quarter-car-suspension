# ============================================================
# QUARTER-CAR SUSPENSION DYNAMICS MODEL
# ============================================================
#
# This model represents one corner of a vehicle using a
# two-degree-of-freedom quarter-car suspension system.
#
# Sprung mass (m1):
# Components supported by the suspension, such as a portion
# of the vehicle body and chassis.
#
# Unsprung mass (m2):
# Components below the suspension, such as the wheel, tire,
# and portions of the suspension assembly.
#
# State vector:
#
# x = [y1, y1_dot, y2, y2_dot]
#
# where:
# y1      = sprung mass displacement
# y1_dot  = sprung mass velocity
# y2      = unsprung mass displacement
# y2_dot  = unsprung mass velocity
#
# The system is represented in state-space form:
#
# x_dot = A*x + F
#
# The primary design variables are:
#
# k1      = suspension spring stiffness
# zeta1   = suspension damping ratio
#
# The suspension damping coefficient is calculated using:
#
# c1 = 2*zeta1*sqrt(k1*m1)
# ============================================================

import numpy as np
import matplotlib.pyplot as plt
from numpy import cos, pi
from scipy.integrate import odeint

# ============================================================
# VEHICLE PARAMETERS
# ============================================================

m1 = 400          # Sprung mass (kg)
m2 = 30           # Unsprung mass (kg)

# k1 = 20e3         # Suspension spring stiffness (N/m)
k2 = 200e3        # Tire stiffness (N/m)

# zeta1 = 0.30      # Suspension damping ratio

# Calculate suspension damping coefficient
# c1 = 2 * zeta1 * np.sqrt(k1 * m1)

c2 = 0             # Tire damping coefficient (Ns/m)

# ============================================================
# ROAD / TEST PARAMETERS
# ============================================================

a = 0.2           # Bump height (m)
v = 0.85          # Vehicle speed (m/s)
d = 0.8           # Bump length (m)

ts = 0             # Time at which bump begins (s)

# Angular frequency of road input
w = 2 * pi * (v / d)

# ============================================================
# SIMULATION TIME
# ============================================================

t_step = 0.01

# Keep the full 20-second simulation so that the system has
# enough time to settle.
T = np.arange(0, 20 + t_step, t_step)

# ============================================================
# STATE-SPACE MATRIX
# ============================================================

# A = np.array([
#     [0.0,       1.0,          0.0,                0.0],
#
#     [-k1/m1,   -c1/m1,       k1/m1,              c1/m1],
#
#     [0.0,       0.0,          0.0,                1.0],
#
#     [k1/m2,     c1/m2,   -(k1 + k2)/m2,
#                          -(c1 + c2)/m2]
# ])

# ============================================================
# DIFFERENTIAL EQUATION
# ============================================================

def quarter_car(x, t, k1, zeta1):
    c1 = 2 * zeta1 * np.sqrt(k1 * m1)

    A = np.array([
        [0.0,       1.0,          0.0,                0.0],

        [-k1/m1,   -c1/m1,       k1/m1,              c1/m1],

        [0.0,       0.0,          0.0,                1.0],

        [k1/m2,     c1/m2,   -(k1 + k2)/m2,
                             -(c1 + c2)/m2]
    ])

    # Apply the road bump only while the vehicle is on it.
    if ts <= t <= (ts + d / v):

        road_displacement = (
            0.5 *
            a *
            (1 - cos(w * (t - ts)))
        )

        # Road force transmitted through tire stiffness
        F_road = (k2 / m2) * road_displacement

    else:

        F_road = 0.0

    F = np.array([
        0.0,
        0.0,
        0.0,
        F_road
    ])

    return A.dot(x) + F

# ============================================================
# INITIAL CONDITIONS
# ============================================================

# Vehicle starts at equilibrium with zero displacement
# and zero velocity.

x0 = np.array([
    0.0,
    0.0,
    0.0,
    0.0
])

# ============================================================
# PARAMETRIC SWEEP
# ============================================================

k1_values = np.linspace(15e3, 25e3, 5)
zeta1_values = np.linspace(0.2, 0.4, 5)

results = []

for k1 in k1_values:
    for zeta1 in zeta1_values:
        # ============================================================
        # SOLVE DIFFERENTIAL EQUATIONS
        # ============================================================

        solution = odeint(
            quarter_car,
            x0,
            T,
            args=(k1, zeta1)
        )

        # ============================================================
        # EXTRACT STATES
        # ============================================================

        # Sprung mass displacement
        d1 = solution[:, 0]

        # Sprung mass velocity
        v1 = solution[:, 1]

        # Unsprung mass displacement
        d2 = solution[:, 2]

        # Unsprung mass velocity
        v2 = solution[:, 3]

        # ============================================================
        # SUSPENSION TRAVEL
        # ============================================================

        # Relative displacement between sprung and unsprung masses

        suspension_travel = d1 - d2

        # ============================================================
        # SPRUNG MASS ACCELERATION
        # ============================================================

        c1 = 2 * zeta1 * np.sqrt(k1 * m1)

        body_acceleration = (
            -c1 * v1
            + c1 * v2
            - k1 * d1
            + k1 * d2
        ) / m1

        # ============================================================
        # METRIC 1:
        # MAXIMUM BODY DISPLACEMENT
        # ============================================================

        max_body_displacement = np.max(
            np.abs(d1)
        )

        # ============================================================
        # METRIC 2:
        # RMS BODY ACCELERATION
        # ============================================================
        #
        # Instead of calculating RMS acceleration over the entire
        # 20-second simulation, calculate it over the first 5 seconds.
        #
        # This prevents the long period after the vehicle has settled
        # from diluting the vibration measurement.
        # ============================================================

        rms_window = 5.0

        rms_indices = T <= rms_window

        rms_body_acceleration = np.sqrt(
            np.mean(
                body_acceleration[rms_indices] ** 2
            )
        )

        # ============================================================
        # METRIC 3:
        # SETTLING TIME
        # ============================================================
        #
        # Settling time is defined as the FIRST time after which the
        # body displacement remains within a specified tolerance band
        # for the remainder of the simulation.
        #
        # Here we use a 2% tolerance based on maximum body displacement.
        # ============================================================

        threshold = 0.02 * max_body_displacement

        # Find all points where the displacement is outside
        # the tolerance band.

        outside_band = np.abs(d1) > threshold

        if np.any(outside_band):

            # Find the LAST time the response is outside the band.
            last_outside_index = np.where(outside_band)[0][-1]

            # Settling time is therefore the time immediately after
            # the final excursion outside the tolerance band.

            if last_outside_index < len(T) - 1:
                settling_time = T[last_outside_index + 1]
            else:
                settling_time = np.nan

        else:

            settling_time = 0.0

        # ============================================================
        # METRIC 4:
        # MAXIMUM SUSPENSION TRAVEL
        # ============================================================

        max_suspension_travel = np.max(
            np.abs(suspension_travel)
        )

        results.append({
            'k1': k1,
            'zeta1': zeta1,
            'max_body_displacement': max_body_displacement,
            'rms_body_acceleration': rms_body_acceleration,
            'settling_time': settling_time,
            'max_suspension_travel': max_suspension_travel
        })

# ============================================================
# PRINT RESULTS
# ============================================================

print("\nEngineering Performance Metrics")
print("--------------------------------")

print(f"{'k1 (N/m)':<10} {'zeta1':<10} {'Max Body Disp (m)':<20} {'RMS Accel (m/s²)':<20} {'Settling Time (s)':<20} {'Max Suspension Travel (m)':<25}")
print("-" * 100)

for result in results:
    print(f"{result['k1']:<10.2f} {result['zeta1']:<10.2f} {result['max_body_displacement']:<20.6f} {result['rms_body_acceleration']:<20.6f} {result['settling_time']:<20.3f} {result['max_suspension_travel']:<25.6f}")

# ============================================================
# PLOT
# ============================================================

# Organize results by zeta1
zeta1_groups = {}
for result in results:
    zeta1 = result['zeta1']
    if zeta1 not in zeta1_groups:
        zeta1_groups[zeta1] = []
    zeta1_groups[zeta1].append(result)

# Create subplots
fig, axs = plt.subplots(2, 2, figsize=(12, 10))
fig.suptitle('Quarter-Car Suspension Performance Metrics')

# Plot 1A: Maximum Body Displacement vs k1
axs[0, 0].set_title('1A: Maximum Body Displacement vs k1')
axs[0, 0].set_xlabel('k1 (N/m)')
axs[0, 0].set_ylabel('Maximum Body Displacement (m)')
for zeta1, group in zeta1_groups.items():
    k1_values = [result['k1'] for result in group]
    max_body_displacement = [result['max_body_displacement'] for result in group]
    axs[0, 0].plot(k1_values, max_body_displacement, label=f'ζ={zeta1:.2f}')
axs[0, 0].legend()
axs[0, 0].grid(True)

# Plot 1B: RMS Acceleration vs k1
axs[0, 1].set_title('1B: RMS Acceleration vs k1')
axs[0, 1].set_xlabel('k1 (N/m)')
axs[0, 1].set_ylabel('RMS Acceleration (m/s²)')
for zeta1, group in zeta1_groups.items():
    k1_values = [result['k1'] for result in group]
    rms_body_acceleration = [result['rms_body_acceleration'] for result in group]
    axs[0, 1].plot(k1_values, rms_body_acceleration, label=f'ζ={zeta1:.2f}')
axs[0, 1].legend()
axs[0, 1].grid(True)

# Plot 1C: Settling Time vs k1
axs[1, 0].set_title('1C: Settling Time vs k1')
axs[1, 0].set_xlabel('k1 (N/m)')
axs[1, 0].set_ylabel('Settling Time (s)')
for zeta1, group in zeta1_groups.items():
    k1_values = [result['k1'] for result in group]
    settling_time = [result['settling_time'] for result in group]
    axs[1, 0].plot(k1_values, settling_time, label=f'ζ={zeta1:.2f}')
axs[1, 0].legend()
axs[1, 0].grid(True)

# Plot 1D: Maximum Suspension Travel vs k1
axs[1, 1].set_title('1D: Maximum Suspension Travel vs k1')
axs[1, 1].set_xlabel('k1 (N/m)')
axs[1, 1].set_ylabel('Maximum Suspension Travel (m)')
for zeta1, group in zeta1_groups.items():
    k1_values = [result['k1'] for result in group]
    max_suspension_travel = [result['max_suspension_travel'] for result in group]
    axs[1, 1].plot(k1_values, max_suspension_travel, label=f'ζ={zeta1:.2f}')
axs[1, 1].legend()
axs[1, 1].grid(True)

plt.tight_layout()
plt.show()