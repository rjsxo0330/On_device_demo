# On_device_Code_lat README

This folder contains the lateral-only on-device SILS controller with EPS torque output.

## 1. File Structure

```text
On_device_Code_lat/
|-- On_device_main_sils.c
|-- README.md
|-- Parameter/
|   |-- car.par
|   |-- Type_def.h
|   |-- code_var.h
|   |-- NumericalOperator.c
|   +-- NumericalOperator.h
|-- Perception/
|   |-- Perception.c
|   |-- Perception.h
|   |-- Input_Mapping/
|   |   |-- Input_Mapping.c
|   |   +-- Input_Mapping.h
|   |-- Make_Vehicle_stat/
|   |   |-- Make_lane/
|   |   |   |-- Make_lane.c
|   |   |   +-- Make_lane.h
|   |   +-- Make_Vehicle_Speed/
|   |       |-- Make_Vehicle_Speed.c
|   |       +-- Make_Vehicle_Speed.h
|   +-- Path_Command/
|       |-- PathCommand.c
|       +-- PathCommand.h
+-- Control/
    |-- Control.c
    |-- Control.h
    +-- Lateral_Controller/
        |-- Lane_Keeping_Control/
        |   |-- LKSControl.c
        |   +-- LKSControl.h
        +-- EPS_Torque_Control/
            |-- SteeringControlBackstepping.c
            +-- SteeringControlBackstepping.h
```

## 2. Flow

```text
On_device_main_sils.c
    -> Read ECAN, ScenarioCAN, MAB input ports
    -> Perception()
        -> Input_Mapping()
        -> Make_Vehicle_Speed()
        -> Make_lane()
        -> PathCommand()
    -> Control()
        -> Control_Gating()
        -> LaneKeepingController()
        -> SteeringControlBackstepping()
    -> Write Gateway CAN torque output
```

## 3. Top-Level Interface

Input ports:

| Port | Width | Source | Purpose |
|---|---:|---|---|
| `u0` | 14 | ECAN | lateral acceleration, yaw rate, steering wheel, rear wheel speed, camera lane coefficients |
| `u1` | 3 | ScenarioCAN | Driver ID, LKS on/off, LKS level |
| `u2` | 13 | MAB | LKS LQR gains and lookahead distance |
| `u3` | 1 | MAB | lateral level |

Output ports:

| Port | Width | Destination | Purpose |
|---|---:|---|---|
| `y0` | 1 | Gateway CAN | EPS torque command |

## 4. Control And Gating

`Control_Gating()` keeps the lateral controller path enabled. LKS on/off is handled by `Logic_LKS_ON_OFF_SEND`.

`LaneKeepingController()` applies level-based LKS LQR gain smoothing through `LevelTransitionArraySmoothing(...)`.

`SteeringControlBackstepping()` converts `Logic_ref_SWA` into `Lateral_Cont_Torque`.

## 5. Unit Policy

Input-side conversions in `On_device_main_sils.c`:

- `ECAN_IMU_YawRt`: deg/s -> rad/s
- `ECAN_SWA_Angle`: deg -> rad
- `ECAN_SWA_Spd`: deg/s -> rad/s
- `ECAN_WHL_SpdRR`: kph -> m/s
- `ECAN_WHL_SpdRL`: kph -> m/s
