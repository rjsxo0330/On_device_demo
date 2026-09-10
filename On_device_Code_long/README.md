# On_device_Code_long README

This folder now contains the longitudinal-only on-device SILS controller.

## 1. File Location Structure

```text
On_device_Code_long/
|-- On_device_main_sils.c                         // S-function entry, port mapping, top-level execution
|-- README.md
|-- Parameter/
|   |-- car.par
|   |-- Type_def.h
|   |-- code_var.h
|   |-- NumericalOperator.c                      // Numerical utilities and level smoothing
|   +-- NumericalOperator.h
|-- Perception/
|   |-- Perception.c                             // Perception integration
|   |-- Perception.h
|   |-- Input_Mapping/
|   |   |-- Input_Mapping.c                      // ECAN/MAB -> Logic mapping
|   |   +-- Input_Mapping.h
|   +-- Make_Vehicle_stat/
|       +-- Make_Vehicle_Speed/
|           |-- Make_Vehicle_Speed.c             // Rear wheel speed average
|           +-- Make_Vehicle_Speed.h
+-- Control/
    |-- Control.c                                // Longitudinal control integration
    |-- Control.h
    +-- Longitudinal_Controller/
        |-- Longi_Con.c                          // Longitudinal controller integration
        |-- Longi_Con.h
        |-- Transition_Logic/
        |   |-- Longi_Transition.c               // CC/ACC transition decision
        |   +-- Longi_Transition.h
        |-- Crusie_Controller/
        |   |-- CruiseControl.c                  // Cruise PI control with gain smoothing
        |   +-- CruiseControl.h
        |-- Adaptive_Crusie_Controller/
        |   |-- AdaptiveCruiseControl.c          // ACC CTG controller
        |   +-- AdaptiveCruiseControl.h
        +-- Safety_Logic/
            |-- Longi_safe_function.c            // Acceleration saturation
            +-- Longi_safe_function.h
```

## 2. Simplified Flow

```text
On_device_main_sils.c
    -> Read ECAN and MAB input ports
    -> Perception()
        -> Input_Mapping()
        -> Make_Vehicle_Speed()
    -> Control()
        -> ALL_LONGI_CON()
            -> Longi_Transition()
            -> CruiseController() or AdaptiveCruiseControl()
            -> Longi_Safty_Function()
    -> Write Gateway CAN longitudinal output
```

## 3. Top-Level Interface

`On_device_main_sils.c` is the Simulink S-function entry point.

Input ports:

| Port | Width | Source | Purpose |
|---|---:|---|---|
| `u0` | 4 | ECAN | Rear wheel speeds, CIPV relative speed and distance |
| `u1` | 6 | MAB | CC PI gains for levels 1 through 3 |
| `u2` | 3 | MAB | longitudinal level, CC target speed, ACC response level |

Output ports:

| Port | Width | Destination | Purpose |
|---|---:|---|---|
| `y0` | 3 | Gateway CAN | acceleration command, upper jerk limit, lower jerk limit |

## 4. Perception Flow

`Perception()` integrates only the longitudinal perception-side logic.

1. `Input_Mapping()` maps raw interface variables to `Logic_*` variables.
2. `Make_Vehicle_Speed()` computes `Logic_wheel_velocity` from rear wheel speeds.

## 5. Control Flow

`Control()` directly runs `ALL_LONGI_CON()` every step.

## 6. Longitudinal Control

`ALL_LONGI_CON()` runs the longitudinal chain:

1. Run `Longi_Transition()`.
2. If `Logic_TR_FLAG == 1`, run `AdaptiveCruiseControl()`.
3. Otherwise run `CruiseController()`.
4. Run `Longi_Safty_Function()` for final acceleration saturation.

`CruiseController()` uses smoothed PI gains:

```c
Logic_CC_Smooth_PI_gain[0]  // smoothed P gain
Logic_CC_Smooth_PI_gain[1]  // smoothed I gain
```

## 7. Unit Policy

Input-side conversions currently applied in `On_device_main_sils.c`:

- `ECAN_WHL_SpdRR`: kph -> m/s
- `ECAN_WHL_SpdRL`: kph -> m/s

MAB target speed input is used as m/s.

## 8. Removed Integrated/Lateral Items

- Removed LKS command input, LKS gain input, EPS observer/backstepping gain input, lane camera coefficient input, steering input, yaw-rate input, and lateral acceleration input from the S-function interface.
- Removed lateral monitor outputs and EPS torque output from the Gateway CAN output port.
- Removed active source files for lane generation, path command generation, LKS control, and EPS torque control.
