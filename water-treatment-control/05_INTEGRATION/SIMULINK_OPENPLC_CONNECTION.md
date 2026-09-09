# Connect Simulink to OpenPLC over Modbus TCP

## Required MathWorks product

Install **Industrial Communication Toolbox**. MATLAB R2026a then provides the `modbus` interface and the Simulink **Modbus Client Read** and **Modbus Client Write** blocks. No Simulink Control Design, Simulink Test, Simscape, or PLC Coder product is needed for this connection.

## Connection settings

| Setting | Value |
|---|---|
| Transport | TCP/IP |
| Device address | `127.0.0.1` or `localhost` |
| Port | `5020` |
| Server ID | `1` |
| Sample time | `0.10 s` |
| Address convention | 1-based |

## Create the integrated model

1. Build the original plant if necessary:

```matlab
START_HERE
build_water_treatment_plant(true)
```

2. Open `Water_Treatment_Plant.slx` and save a copy as `Water_Treatment_Plant_OpenPLC.slx`.
3. Remove or disconnect the ten `From Workspace` command blocks for the actuator commands. Keep disturbance sources such as raw inflow and network demand.
4. Add one **Modbus Client Read** and one **Modbus Client Write** block from:

```text
Industrial Communication Toolbox / Modbus
```

5. In Model Settings > Modbus, add client `OpenPLC` with TCP/IP, `localhost`, and port `5020`.

## Modbus Client Read table

Configure three rows:

| Name | Address | Register type | Precision | Count |
|---|---:|---|---|---:|
| PLC_Analog_Commands | 101 | Holding Register | uint16 | 6 |
| PLC_Digital_Commands | 51 | Coil | bit | 7 |
| Fault_Injection | 251 | Coil | bit | 4 |

All addresses are entered exactly as shown. The Simulink blocks use 1-based addresses.

Use `unpack_openplc_commands.m` as the reference for decoding:

```matlab
cmd = unpack_openplc_commands(analogRegs,digitalCoils);
```

Connections:

- HR101/102 -> P-101A/B speed inputs; gate with C51/C52.
- HR103 -> P-201 speed input; gate with C53.
- HR104/105 -> P-301A/B speed inputs; gate with C54/C55.
- HR106 -> DP-201 dosing command.
- C56 -> mixer command/effectiveness. For the first integration version, use `double(C56)` as mixer effectiveness and return C6 equal to C56.
- C57 -> XV-201 command.
- C251 -> P-301A trip/unavailable input.
- C252 -> freeze the AIT-201 measured signal while leaving actual concentration unchanged.
- C253 -> force the valve actual position/feedback closed.
- C254 -> freeze the heartbeat rather than resetting the PLC alarm directly.

## Modbus Client Write table

Configure two rows:

| Name | Address | Register type | Precision |
|---|---:|---|---|
| Plant_Analog_Measurements | 1 | Holding Register | uint16 |
| Plant_Digital_Feedback | 1 | Coil | bit |

The first input is a 1-by-9 vector for HR1-HR9. Use `pack_openplc_plant_registers.m` as the exact scaling reference:

```matlab
regs = pack_openplc_plant_registers( ...
    LIT101_pct,LIT201_pct,LIT301_pct, ...
    FIT101_Lps,FIT201_Lps,FIT301_Lps, ...
    AIT201_mgL,PIT301_kPa,heartbeat);
```

The second input is a 1-by-8 feedback vector for C1-C8:

```matlab
feedback = double([ ...
    P101A_RunFb, P101B_RunFb, P201_RunFb, ...
    P301A_RunFb, P301B_RunFb, M201_RunFb, ...
    XV201_OpenFb, XV201_ClosedFb]);
```

Set the Modbus Client Write sample time to `0.10` seconds.

## Heartbeat

Create a counter that increments once per 0.10-second communication update and wraps at 65535. If C254 is true, hold the previous heartbeat value. The PLC declares communication failed when the value does not change for 2 seconds.

## Run close to wall-clock time

PLC timers execute in real wall-clock time while ordinary Simulink simulation may run much faster. Enable Simulation Pacing at 1x or run:

```matlab
set_param('Water_Treatment_Plant_OpenPLC','EnablePacing','on','PacingRate','1');
```

Use Normal simulation mode for initial integration.

## First closed-loop startup order

1. Start Docker Desktop.
2. Start the OpenPLC Runtime container.
3. Open OpenPLC Editor, upload the program, and start the PLC.
4. Start `Water_Treatment_Plant_OpenPLC.slx` so the heartbeat and safe initial measurements begin updating.
5. Confirm `Comm_Healthy = TRUE` in OpenPLC.
6. Use MATLAB Modbus Explorer or Ignition to pulse C104 (Auto Mode) and then C101 (System Start).
7. Observe C51/C52 and HR101/HR102 filling T-201.
8. Keep MATLAB/Simulink running while using OpenPLC online monitoring.

For the automated first-loop proof, use:

```matlab
BUILD_OPENPLC_INTEGRATION
run_openplc_simulink_demo
```

The demo deliberately uses only one MATLAB Modbus client at a time. It first
checks `OpenPLCModbusBridge` directly, then the bridge sends the reset,
automatic-mode, and start pulses while Simulink runs for 30 seconds at 1x
pacing. After the bridge releases its client, the script reads PLC status and
evaluates the logged pump command, physical feedback, and T-201 level change.

## Stage 5 full-control integration

After the first closed-loop demo passes, run:

```matlab
RUN_STAGE5_FULL_CONTROL
```

Rebuild/upload the v0.5.2 OpenPLC source first. The final nonlinear-validated
concentration gains are `Kp=100.0` and `Ki*Ts=0.10`; the old reduced-order
candidate must not be left in the running PLC. Version v0.5.2 also sets the
XV-201 open-feedback timeout to 8.0 seconds, safely beyond the approximately
6.0 seconds needed by the simulated valve to reach its 95% open threshold.

The approximately 260-second test validates a complete automatic batch, both
OpenPLC PI loops, booster staging, lead alternation, and physical standby
takeover. It also logs a 48-value PLC diagnostic vector containing HR301-HR317
and C151-C181. The column definitions are in
`05_INTEGRATION/Stage5_Diagnostic_Map.csv`.

Only the controller/plant portion of Integration Gate C1 is closed in Stage 5.
Ignition display and history are completed and verified in Stage 6.

## Critical rule

Do not connect Ignition buttons directly to C51-C57 or HR101-HR106. Those are PLC-owned actuator outputs. Ignition writes requests C101-C118 and setpoints HR201-HR205; OpenPLC applies permissives and interlocks.
