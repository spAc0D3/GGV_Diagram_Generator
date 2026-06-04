# 2026 GGV Project

GGV (G-G-Velocity) Diagram Generator for FSAE electric, in development. Subsequent development will focus on creating a lap time simulation program.

## Dependencies

- [FSAE-VD-Personal-Scripts](https://github.com/Alphabet1671/FSAE-VD-Personal-Scripts) — `external/FSAE-VD-Personal-Scripts/`
  - Using the UniTire model for tire modeling.

## Specs for Simulation

| File | Description |
| ---- | ----------- |
| `data/Vehicle_BasicParams.csv` | Vehicle basic parameters |
| `data/Vehicle_AeroMap_RideHeight.csv` | Aerodynamic coefficients vs Ride Height |
| `data/Vehicle_AeroMap_Yaw.csv` | Aerodynamic coefficients vs Yaw (Not yet used) |
| `data/Vehicle_Motor_PeakTorque_EMRAX228.csv` | EMRAX 228 Peak Torque |
| `data/Vehicle_Motor_Efficiency_EMRAX228.csv` | EMRAX 228 Efficiency MAP |
| `external/FSAE-VD-Personal-Scripts/` | Tire Model (UniTire), from Alphabet1671 (MIT License) |
