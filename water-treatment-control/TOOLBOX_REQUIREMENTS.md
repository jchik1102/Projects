# Toolbox Requirements

## Install now

| Product | Status | Why it is used |
|---|---|---|
| MATLAB | Required | Parameters, calculations, scripts, test framework, tables, plots, exports |
| Simulink | Required | Dynamic plant and controller-validation models |
| Control System Toolbox | Required | `tf`, `pidtune`, `pid`, `feedback`, `margin`, `stepinfo`, `c2d`, `isstable`, poles |

## Optional now

| Product | Status | Use |
|---|---|---|
| Simulink Control Design | Optional | Exact linearization, operating points, Model Linearizer |
| Simulink Test | Optional | Test Manager, test harnesses, managed regression reports |

## Install later for OpenPLC integration

| Product | Status | Use |
|---|---|---|
| Industrial Communication Toolbox | Later requirement | Modbus TCP read/write between Simulink/MATLAB and OpenPLC |

MathWorks documents the Simulink Modbus Client blocks as available beginning in R2024b, so R2024b or newer is recommended.
