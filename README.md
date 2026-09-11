# On_device_demo

On-device SILS controller demo project.

## Project Structure

| Folder | Purpose | Output |
|---|---|---|
| `On_device_Code_long` | Longitudinal-only controller | acceleration request |
| `On_device_Code_lat_EPS_Torque` | Lateral-only controller with EPS torque control | EPS torque request |
| `On_device_Code_lat_SWA` | Lateral-only controller with direct steering wheel angle output | steering wheel angle request |

## Lateral Variants

Both lateral variants use the same input/output shape:

- `u0`: width 14, E-CAN lateral sensors
- `u1`: width 3, ScenarioCAN LKS command
- `u2`: width 13, LKS gains and lookahead distance
- `u3`: width 1, lateral level
- `y0`: width 1, selected lateral command output

`On_device_Code_lat_EPS_Torque` outputs `Lateral_Cont_Torque`.

`On_device_Code_lat_SWA` outputs `Logic_ref_SWA`.
