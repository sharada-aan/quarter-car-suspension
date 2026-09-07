# Suspension Dynamics Modeling, Testing & Optimization

## 1. Engineering Question

**How can suspension stiffness ($k_1$) and damping ratio ($\zeta_1$) be optimized to reduce vibration and maintain controlled suspension travel when subjected to road disturbances?**

This project develops a computational quarter-car suspension model to investigate how suspension stiffness and damping affect vehicle response to a controlled road disturbance. A parametric sweep is used to evaluate multiple suspension configurations, followed by multi-objective Pareto analysis to identify tradeoffs between ride comfort and suspension control.

---

## 2. Project Metrics & Objectives

The suspension configurations are evaluated using four performance metrics.

| Metric                    | Definition                                                          | Engineering Objective |
| ------------------------- | ------------------------------------------------------------------- | --------------------- |
| Maximum body displacement | $\max\left(\lvert y_1\rvert\right)$                                 | Minimize              |
| RMS body acceleration     | RMS acceleration over the first 5 seconds                           | Minimize              |
| Settling time             | Time required for body displacement to remain within a $\pm2%$ band | Minimize              |
| Maximum suspension travel | $\max\left(\lvert y_1-y_2\rvert\right)$                             | Minimize              |

### Primary Optimization Objectives

The project focuses primarily on two competing objectives:

1. **Ride comfort:** Minimize RMS body acceleration.
2. **Suspension control:** Minimize maximum suspension travel.

These objectives are not necessarily minimized by the same suspension configuration, creating a multi-objective engineering tradeoff.

---

## 3. System Definition & Variables

### 3.1 Quarter-Car Model

The system represents one corner of a vehicle using a two-degree-of-freedom quarter-car suspension model.

* $m_1$ = sprung mass
* $m_2$ = unsprung mass
* $k_1$ = suspension spring stiffness
* $c_1$ = suspension damping coefficient
* $k_2$ = tire stiffness
* $c_2$ = tire damping
* $y_1$ = sprung mass displacement
* $y_2$ = unsprung mass displacement
* $y_r$ = road displacement

The suspension damping coefficient is determined from the damping ratio:

$$
c_1 = 2\zeta_1\sqrt{k_1m_1}
$$

where $\zeta_1$ is the suspension damping ratio.

### 3.2 Design Variables

The two primary design variables are:

* Suspension stiffness, $k_1$
* Suspension damping ratio, $\zeta_1$

The parameter sweep evaluates:

$$
k_1 \in \{15{,}000,\ 17{,}500,\ 20{,}000,\ 22{,}500,\ 25{,}000\}\ \text{N/m}
$$

and

$$
\zeta_1 \in \{0.20,\ 0.25,\ 0.30,\ 0.35,\ 0.40\}
$$

This produces:

$$
5\times5=25
$$

total suspension configurations.

### 3.3 Fixed Parameters

| Parameter             |           Value |
| --------------------- | --------------: |
| Sprung mass, $m_1$    |        $400$ kg |
| Unsprung mass, $m_2$  |         $30$ kg |
| Tire stiffness, $k_2$ | $200{,}000$ N/m |
| Tire damping, $c_2$   |       $0$ N·s/m |
| Bump height, $a$      |         $0.2$ m |
| Vehicle speed, $v$    |      $0.85$ m/s |
| Bump length, $d$      |         $0.8$ m |
| Simulation duration   |          $20$ s |
| Time step             |        $0.01$ s |

The road disturbance frequency is defined as:

$$
\omega = 2\pi\frac{v}{d}
$$

---

## 4. Mathematical & Computational Modeling

### 4.1 State-Space Representation

The system state vector is defined as:

$$
x =
\begin{bmatrix}
y_1 \\
\dot{y}_1 \\
y_2 \\
\dot{y}_2
\end{bmatrix}
$$

The system is represented in state-space form as:

$$
\dot{x}=Ax+F_r(t)
$$

where $A$ is the system matrix and $F_r(t)$ represents the forcing caused by the road disturbance.

The system matrix is:

$$
A=
\begin{bmatrix}
0 & 1 & 0 & 0 \\
-\frac{k_1}{m_1} & -\frac{c_1}{m_1} &
\frac{k_1}{m_1} & \frac{c_1}{m_1} \\
0 & 0 & 0 & 1 \\
\frac{k_1}{m_2} & \frac{c_1}{m_2} &
-\frac{k_1+k_2}{m_2} &
-\frac{c_1+c_2}{m_2}
\end{bmatrix}
$$

### 4.2 Road Disturbance

The vehicle is subjected to a controlled half-cosine road bump.

During the bump:

$$
y_r(t)=
\frac{a}{2}
\left[
1-\cos\left(\omega(t-t_s)\right)
\right]
$$

for

$$
t_s \leq t \leq t_s+\frac{d}{v}
$$

Outside this interval:

$$
y_r(t)=0
$$

The corresponding road forcing is applied through the tire stiffness:

$$
F_r(t)=
\begin{bmatrix}
0\\
0\\
0\\
\frac{k_2}{m_2}y_r(t)
\end{bmatrix}
$$

### 4.3 Numerical Integration

The initial implementation used explicit Euler integration. For the selected system and timestep, this produced unstable and physically unrealistic results.

The simulation was subsequently refactored to use `scipy.integrate.odeint`, which uses the LSODA numerical solver with adaptive step-size control and automatic method selection.

The revised implementation produced physically consistent responses across the tested parameter range.

### 4.4 Performance Metrics

#### Maximum Body Displacement

Maximum body displacement is calculated as:

$$
D_{\max}=\max\left(\lvert y_1(t)\rvert\right)
$$

This measures the maximum vertical movement of the vehicle body.

#### RMS Body Acceleration

Body acceleration is calculated from the suspension forces:

$$
\ddot{y}_1=
\frac{
-c_1\dot{y}_1
+c_1\dot{y}_2
-k_1y_1
+k_1y_2
}{m_1}
$$

RMS acceleration is then calculated over the first 5 seconds:

$$
a_{\mathrm{RMS}}
=
\sqrt{
\frac{1}{N}
\sum_{i=1}^{N}
\ddot{y}_1(t_i)^2
}
$$

Lower RMS acceleration corresponds to lower vibration transmitted to the vehicle body.

#### Settling Time

A tolerance band is defined as:

$$
\epsilon=0.02D_{\max}
$$

The settling time is determined by finding the final time at which:

$$
\lvert y_1(t)\rvert>\epsilon
$$

The following timestep is taken as the settling time.

#### Maximum Suspension Travel

Suspension travel is:

$$
\Delta y=y_1-y_2
$$

and the maximum suspension travel is:

$$
S_{\max}=\max\left(\lvert y_1-y_2\rvert\right)
$$

Lower suspension travel indicates that the suspension experiences less relative movement between the sprung and unsprung masses.

---

## 5. Parametric Sweep Results

A total of **25 suspension configurations** were simulated by varying $k_1$ and $\zeta_1$ while keeping all other system parameters fixed.

### 5.1 Complete Results

| $k_1$ (N/m) | $\zeta_1$ | Max Body Displacement (m) | RMS Acceleration (m/s²) | Settling Time (s) | Max Suspension Travel (m) |
| ----------: | --------: | ------------------------: | ----------------------: | ----------------: | ------------------------: |
|      15,000 |      0.20 |                  0.279147 |                2.290177 |              4.07 |                  0.167146 |
|      15,000 |      0.25 |                  0.270411 |                2.060626 |              3.51 |                  0.153311 |
|      15,000 |      0.30 |                  0.263209 |                1.906259 |              3.00 |                  0.141344 |
|      15,000 |      0.35 |                  0.257228 |                1.798623 |              2.51 |                  0.130784 |
|      15,000 |      0.40 |              **0.252233** |            **1.721743** |              2.45 |                  0.121521 |
|      17,500 |      0.20 |                  0.282529 |                2.495595 |              3.83 |                  0.155695 |
|      17,500 |      0.25 |                  0.273792 |                2.235643 |              3.31 |                  0.142712 |
|      17,500 |      0.30 |                  0.266520 |                2.058611 |              2.83 |                  0.131472 |
|      17,500 |      0.35 |                  0.260413 |                1.933181 |              2.38 |                  0.121588 |
|      17,500 |      0.40 |                  0.255228 |                1.841840 |              2.33 |                  0.112857 |
|      20,000 |      0.20 |                  0.284409 |                2.664350 |              3.64 |                  0.145166 |
|      20,000 |      0.25 |                  0.275756 |                2.379464 |              3.15 |                  0.132989 |
|      20,000 |      0.30 |                  0.268522 |                2.183767 |              2.70 |                  0.122467 |
|      20,000 |      0.35 |                  0.262409 |                2.043619 |              2.27 |                  0.113273 |
|      20,000 |      0.40 |                  0.257176 |                1.940275 |              2.23 |                  0.105163 |
|      22,500 |      0.20 |                  0.285461 |                2.801035 |              3.48 |                  0.135542 |
|      22,500 |      0.25 |                  0.276866 |                2.496369 |              3.02 |                  0.124240 |
|      22,500 |      0.30 |                  0.269690 |                2.285734 |              2.59 |                  0.114398 |
|      22,500 |      0.35 |                  0.263623 |                2.133727 |              2.19 |                  0.105778 |
|      22,500 |      0.40 |                  0.258379 |                2.020658 |              2.14 |                  0.098189 |
|      25,000 |      0.20 |                  0.285812 |                2.909847 |              3.67 |                  0.129425 |
|      25,000 |      0.25 |                  0.277370 |                2.590068 |              2.91 |                  0.116230 |
|      25,000 |      0.30 |                  0.270299 |                2.367899 |              2.50 |                  0.107053 |
|      25,000 |      0.35 |                  0.264289 |                2.206637 |              2.11 |                  0.099050 |
|      25,000 |      0.40 |                  0.259037 |                2.085909 |          **2.08** |              **0.091999** |

### 5.2 Performance Metrics

![Performance Metrics](../figures/performance_metrics.png)

The four plots show how suspension stiffness and damping ratio affect body displacement, body acceleration, settling time, and suspension travel across all 25 configurations.

### 5.3 Observed Parameter Trends

Several trends emerge from the parameter sweep.

**Increasing damping ratio improves performance across the tested range.**

For every stiffness value tested, increasing $\zeta_1$ from $0.20$ to $0.40$ reduces:

* maximum body displacement,
* RMS body acceleration,
* settling time, and
* maximum suspension travel.

This indicates that greater damping suppresses the transient response for the specific road disturbance modeled.

**Increasing stiffness creates a tradeoff between ride comfort and suspension control.**

For a fixed damping ratio, increasing $k_1$ generally:

* increases RMS body acceleration,
* slightly increases maximum body displacement,
* reduces maximum suspension travel, and
* generally reduces settling time.

Therefore, higher stiffness provides greater control of relative suspension movement but can transmit more vibration to the vehicle body.

These trends apply specifically to the modeled vehicle parameters, road disturbance, and tested design range.

---

## 6. Engineering Optimization & Conclusion

The parameter sweep demonstrates a **multi-objective tradeoff between ride isolation and suspension control**.

Lower stiffness reduces body displacement and acceleration, while higher stiffness reduces suspension travel and generally improves recovery time. Increasing damping improves all four metrics within the tested parameter range.

### 6.1 Ride Comfort Optimization

The configuration with the lowest RMS body acceleration is:

$$
\boxed{
k_1=15{,}000\ \text{N/m},
\qquad
\zeta_1=0.40
}
$$

This configuration produces:

* RMS body acceleration: **1.7217 m/s²**
* Maximum body displacement: **0.2522 m**
* Maximum suspension travel: **0.1215 m**
* Settling time: **2.45 s**

This configuration represents the **ride-comfort endpoint** of the tested design space.

### 6.2 Suspension Control Optimization

The configuration with the lowest maximum suspension travel is:

$$
\boxed{
k_1=25{,}000\ \text{N/m},
\qquad
\zeta_1=0.40
}
$$

This configuration produces:

* Maximum suspension travel: **0.0920 m**
* Settling time: **2.08 s**
* RMS body acceleration: **2.0859 m/s²**
* Maximum body displacement: **0.2590 m**

This configuration represents the **suspension-control endpoint** of the tested design space.

### 6.3 Pareto Optimization

Because no single configuration minimizes every performance metric, Pareto analysis was used to identify non-dominated configurations with respect to:

1. RMS body acceleration
2. Maximum suspension travel

A configuration is considered Pareto-optimal if no other tested configuration provides both lower RMS acceleration **and** lower suspension travel.

The resulting Pareto-optimal configurations are:

| $k_1$ (N/m) | $\zeta_1$ | RMS Acceleration (m/s²) | Max Suspension Travel (m) |
| ----------: | --------: | ----------------------: | ------------------------: |
|      15,000 |      0.40 |                  1.7217 |                    0.1215 |
|      17,500 |      0.40 |                  1.8418 |                    0.1129 |
|      20,000 |      0.40 |                  1.9403 |                    0.1052 |
|      22,500 |      0.40 |                  2.0207 |                    0.0982 |
|      25,000 |      0.40 |                  2.0859 |                    0.0920 |

![Comfort vs. Suspension Control](../figures/comfort_vs_control.png)

The Pareto front demonstrates the tradeoff clearly:

* Moving toward **lower stiffness** improves ride comfort by reducing RMS body acceleration.
* Moving toward **higher stiffness** reduces suspension travel.
* All Pareto-optimal configurations occur at $\zeta_1=0.40$, indicating that the highest tested damping ratio was favored for both objectives.

The middle configurations provide intermediate tradeoffs between comfort and suspension control.

### 6.4 Engineering Interpretation

The results show that suspension design cannot be reduced to a single universally optimal stiffness value.

Instead, the preferred configuration depends on the engineering priority:

| Priority                  | Preferred Configuration                       |
| ------------------------- | --------------------------------------------- |
| Minimum body vibration    | $k_1=15{,}000$ N/m, $\zeta_1=0.40$            |
| Minimum suspension travel | $k_1=25{,}000$ N/m, $\zeta_1=0.40$            |
| Balanced performance      | One of the intermediate Pareto configurations |

For an engineering application, the final choice would depend on additional requirements such as allowable suspension travel, passenger comfort targets, vehicle packaging constraints, component limitations, and expected terrain.

### 6.5 Limitations

This model is intentionally simplified and does not represent every aspect of a real vehicle suspension system.

Key limitations include:

* The model represents only one quarter of a vehicle.
* The road input is a controlled half-cosine bump rather than measured terrain data.
* Tire damping is set to zero.
* The suspension parameters are evaluated only over the tested design range.
* The model does not include nonlinear suspension behavior.
* No experimental validation has yet been performed.

Therefore, the results should be interpreted as a computational design study rather than a complete vehicle suspension design.

---

## 7. Project Workflow & Next Steps

### Completed

**1. Mathematical Model**

Developed and implemented a two-degree-of-freedom quarter-car suspension model.

**2. Numerical Simulation**

Implemented numerical integration using `scipy.integrate.odeint` and verified the resulting response across the tested parameter range.

**3. Parametric Sweep**

Evaluated 25 combinations of suspension stiffness and damping ratio.

**4. Performance Analysis**

Calculated:

* maximum body displacement,
* RMS body acceleration,
* settling time, and
* maximum suspension travel.

**5. Multi-Objective Optimization**

Identified the Pareto-optimal configurations balancing ride comfort and suspension control.

---

### Future Work

**6. Physical Bench-Scale Suspension Test Platform**

Develop a physical suspension test platform capable of reproducing controlled disturbances.

**7. Arduino Instrumentation**

Integrate Arduino-based sensing and data acquisition to measure physical system response.

Potential measurements include:

* acceleration,
* displacement,
* suspension travel, and
* input disturbance.

**8. Experimental Data Collection**

Run physical experiments using selected baseline configurations from the computational Pareto front.

**9. Model Validation**

Compare experimental measurements against simulation predictions and quantify model error.

**10. More Complex Terrain Models**

Extend the road input beyond a single bump to include:

* repeated bumps,
* washboard terrain, and
* irregular terrain profiles.

**11. Robotics / Off-Road Extension**

Adapt the suspension model toward robotic vehicle applications, where terrain interaction and wheel-ground contact become increasingly important.


### Tools & Technologies

* **Python**
* **NumPy**
* **SciPy**
* **Matplotlib**
* **CSV data processing**
* **State-space modeling**
* **Numerical integration**
* **Parametric analysis**
* **Pareto optimization**

---

## 12. Conclusion

This project developed a computational quarter-car suspension model and used it to investigate how suspension stiffness and damping affect vehicle response to a controlled road disturbance.

The 25-configuration parameter sweep demonstrated that increasing damping ratio improved all four evaluated performance metrics within the tested range. Suspension stiffness produced a clear tradeoff: lower stiffness favored ride comfort, while higher stiffness reduced suspension travel and generally improved recovery time.

Pareto analysis identified five non-dominated configurations, all using the highest tested damping ratio of $\zeta_1=0.40$. The resulting Pareto front provides a range of viable engineering solutions rather than a single universal optimum.

The next stage of the project is to transition from computational analysis to physical experimentation, using an instrumented bench-scale suspension platform to validate the simulation and establish a foundation for future robotics and off-road vehicle applications.