# GGV Diagram Generator for FSAE Electric

GGV (G-G-Velocity) Diagram Generator for FSAE electric, in development. Subsequent development will focus on creating a lap time simulation program.

## Dependencies

Using the UniTire model for tire modeling (`https://github.com/Alphabet1671/FSAE-VD-Personal-Scripts/Yaw Dynamics/Tire Model/`).

## Specs for Simulation

| File | Description |
| ---- | ----------- |
| `data/Vehicle_BasicParams.csv` | Vehicle basic parameters |
| `data/Vehicle_AeroMap_RideHeight.csv` | Aerodynamic coefficients vs Ride Height |
| `data/Vehicle_AeroMap_Yaw.csv` | Aerodynamic coefficients vs Yaw (Not yet used) |
| `data/Vehicle_Motor_Efficiency_EMRAX228.csv` | EMRAX 228 Efficiency MAP |
| `data/Vehicle_Motor_Torque_EMRAX228_Factory.csv` | EMRAX 228 Factory Torque MAP |
| `data/Vehicle_Motor_Torque_EMRAX228_80kW.csv` | EMRAX 228 Torque MAP Under 80kW Limitation |
| `data/Vehicle_Motor_Torque_EMRAX228_25E46.csv` | EMRAX 228 Torque MAP Derived from 25E46 |
