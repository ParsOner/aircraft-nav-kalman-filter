# Aircraft Navigation System: Climb-Cruise-Descent Trajectory Simulation

A MATLAB-based aircraft navigation simulation that models a complete climb–cruise–descent flight profile, simulates five onboard sensors with realistic error models, and fuses GPS + barometric altimeter measurements through a **6-state Kalman Filter** for optimal state estimation.

> 📄 Final project for *Systems for Autonomous Aircraft* — MSc in Autonomous Vehicle Engineering, Università degli Studi di Napoli Federico II (2025–2026)

---

## Overview

This project constructs a realistic aircraft navigation pipeline from scratch:

1. **Trajectory Generation** — Cosine-smoothed climb/cruise/descent profile in the NED frame  
2. **Sensor Simulation** — Five sensor models with bias, drift, and noise  
3. **State Estimation** — 6-state linear Kalman Filter fusing GPS and barometer data  

The simulation runs for 120 seconds across three 40-second flight phases at 80 m/s forward airspeed and 1000 m cruise altitude.

---

## Sensor Models

| Sensor | Measurement | Error Model |
|---|---|---|
| **Accelerometer** | Specific force (ax, ay, az) | Bias random walk + white noise |
| **Gyroscope** | Angular rates (p, q, r) | Bias random walk + white noise |
| **Barometric Altimeter** | Pressure → altitude | Constant bias + Gaussian noise |
| **Magnetometer** | Heading (ψ) via Earth's B-field | Hard-iron bias + noise |
| **GNSS (GPS)** | Position & velocity (3D) | White noise, 2 Hz update rate |

---

## Kalman Filter Design

**State vector (6 states):**

```
x = [x, y, z, vx, vy, vz]ᵀ
```

**Prediction:** Constant-velocity model (`x_{k+1} = x_k + v·dt`)

**Update:** Sequential — GPS update (when available at 2 Hz), then barometer update (every step)

**Tuning parameters:**

| Parameter | Value | Description |
|---|---|---|
| Q (position) | 0.5 m² | Process noise — position |
| Q (velocity) | 0.1 (m/s)² | Process noise — velocity |
| R (GPS pos.) | 4 m² | GPS position noise variance |
| R (GPS vel.) | 0.058 (m/s)² | GPS velocity noise variance |
| R (baro) | 9 m² | Barometer altitude noise variance |

---

## Results

The Kalman Filter provides smooth, accurate estimates compared to raw GPS:

- **Horizontal position (X, Y):** Mean error ≈ 0, consistently within ±1σ bounds  
- **Vertical position (Z):** Small bias (~−0.02 m), slightly more sensitive to noise  
- **Velocity:** Good consistency in Vx/Vy; Vz shows larger errors during climb/descent transitions  

---

## Getting Started

### Requirements

- MATLAB R2020b or later (no additional toolboxes required)

### Run

```matlab
nav_simulation
```

The script generates 8 figures covering trajectory visualization, sensor data, Kalman Filter estimates, and error analysis with covariance bounds.

---

## Project Structure

```
aircraft-nav-kalman-filter/
├── nav_simulation.m              # Main simulation script
├── docs/
│   └── project_presentation.pptx # Project presentation (24 slides)
├── README.md
├── LICENSE
└── .gitignore
```

---

## Future Work

- **Extended Kalman Filter (15 states)** — Add attitude (φ, θ, ψ) and IMU biases
- **IMU integration** — Fuse accelerometer + gyroscope for attitude estimation
- **Magnetometer fusion** — Incorporate heading into the state vector
- **Real flight validation** — Deploy on UAV, compare with RTK-GPS ground truth

---

## References

- Course: *Systems for Autonomous Aircraft* — Lecture 8: Kalman Filtering  
- Project Guide: Prof. Roberto Opromolla  
- University: Università degli Studi di Napoli Federico II — Dipartimento di Ingegneria Industriale

---

## License

This project is released under the [MIT License](LICENSE).
