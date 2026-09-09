# Automated Water Treatment Control Project — Full Control Integration v0.5.2

This package extends the known-good v0.4.5 Simulink/OpenPLC loop with the complete Stage 5 controller/plant commissioning test.

## Architecture boundary

- **MATLAB:** parameters, controller design, scenarios, test execution, metrics, exports, and plots.
- **Simulink:** tanks, pumps, valves, pressure, concentration, sensors, actuator dynamics, and physical feedback.
- **OpenPLC:** modes, batch sequence, PI execution, interlocks, alarms, staging, and fault handling.
- **Ignition later:** HMI, alarms, trends, history, and operator requests.

The operational controller remains in OpenPLC; Simulink remains the physical plant.

## Required MathWorks products

1. MATLAB
2. Simulink
3. Control System Toolbox
4. Industrial Communication Toolbox

The integration was developed for MATLAB R2026a. Simulink Control Design and Simulink Test are optional.

No Simscape, Simscape Fluids, Stateflow, PLC Coder, Optimization Toolbox, System Identification Toolbox, or Fixed-Point Designer is required.

## Standalone engineering tests

Extract the project, set MATLAB Current Folder to the project root, and run:

```matlab
START_HERE
run_all_tests
```

The package includes or builds these models in `02_SIMULINK`:

- `Water_Treatment_Plant.slx`
- `Pressure_Controller_Test.slx`
- `Concentration_Controller_Test.slx`
- `Water_Treatment_Plant_OpenPLC_v0_4.slx`

## Stage 5 full-control integration

First rebuild and upload the v0.5.2 OpenPLC source as described in
`03_OPENPLC/README_FIRST_RUN.md`; Stage 5 contains the nonlinear-validated
concentration PI gains and the corrected 8.0-second XV-201 open-feedback
supervision window. Keep Docker and OpenPLC running, ensure the updated program
is in Run mode, and close any other MATLAB Modbus client. Then run:

```matlab
if bdIsLoaded('Water_Treatment_Plant_OpenPLC_v0_4')
    close_system('Water_Treatment_Plant_OpenPLC_v0_4', 0);
end

clear classes
clear functions

RUN_STAGE5_FULL_CONTROL
```

The paced test takes approximately 260 seconds and checks:

- complete fill/dose/mix/verify/transfer batch execution;
- pressure PI response and two-booster staging;
- concentration PI response and quality-band acceptance;
- lead alternation after batch completion; and
- P-301B takeover after an injected P-301A trip.

Results, raw logs, and the evidence trend are saved under `06_TESTING`.
Stage 5 closes the controller/plant portion of Integration Gate C1. Ignition display and history remain for Stage 6.

## Test groups

- Stages 1–4: MATLAB foundation, controller design, standalone Simulink plant, and controller-model validation.
- Gate G4: first real Simulink/OpenPLC measurement-command-feedback loop with watchdog.
- Stage 5: complete automatic batch, both PLC PI loops, staging, alternation, and standby takeover.

## Useful commands

```matlab
START_HERE
run_all_tests
generate_control_design_plots
BUILD_OPENPLC_INTEGRATION
run_openplc_simulink_demo
RUN_STAGE5_FULL_CONTROL
```

## Environment-specific verification

The Stage 5 source and package consistency are statically checked here, but the live 260-second test must execute on the MATLAB R2026a/OpenPLC installation that owns the running Modbus server. `RUN_STAGE5_FULL_CONTROL` reports bridge or model errors and always attempts to return PLC requests and fault injections to a stopped, cleared state.
