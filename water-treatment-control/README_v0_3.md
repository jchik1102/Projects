# Automated Water Treatment Control Project — v0.3

Version 0.3 retains the validated MATLAB/Simulink v0.2 work and adds the OpenPLC control layer, OpenPLC Runtime v4 Docker configuration, an expanded Modbus register map, MATLAB Modbus tests, and Simulink/Ignition connection instructions.

## Required software now

- MATLAB R2026a
- Simulink
- Control System Toolbox
- **Industrial Communication Toolbox**
- **OpenPLC Editor v4**
- **Docker Desktop for Windows with WSL 2**

Ignition is required when the SCADA stage begins, but it is not required to pass the first OpenPLC/Simulink communication tests.

## Start here

1. Read `03_OPENPLC/README_FIRST_RUN.md`.
2. Start the runtime from `03_OPENPLC/Runtime`.
3. Create the `main` Structured Text POU using the two paste files in `03_OPENPLC/Source`.
4. Upload and start the PLC program.
5. In MATLAB run:

```matlab
START_HERE
check_openplc_requirements
test_openplc_modbus_smoke
run_openplc_acceptance_tests
```

6. Connect the actual Simulink model using `05_INTEGRATION/SIMULINK_OPENPLC_CONNECTION.md`.
7. Connect Ignition using `05_INTEGRATION/IGNITION_OPENPLC_CONNECTION.md`.

## PLC implementation included

- Stopped, automatic, manual, and maintenance modes
- Treatment batch state machine: 0, 10, 20, 30, 40, 50, 60, 70, and 900
- Pressure PI with output bias, clamping anti-windup, staging, de-staging, lead/lag selection, and standby takeover
- Concentration PI with saturation, anti-windup, stable-band timing, retries, and high-high shutdown
- Pump and valve permissives
- Failed-start and failed-open detection
- Communication heartbeat watchdog and safe state
- Runtime counters, batch counters, first-out code, alarm words, and SCADA status bits

The PLC code assumes a 100 ms cyclic task. Do not change the task period without retuning the integral increments and scan-count timers.
