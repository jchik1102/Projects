# OpenPLC v4 — First Run and Project Setup

## Required installations

1. **OpenPLC Editor v4** for creating, compiling, uploading, and monitoring the IEC 61131-3 project.
2. **Docker Desktop for Windows**, using the WSL 2 backend, for the OpenPLC Runtime v4 container.
3. **Industrial Communication Toolbox** in MATLAB, in addition to the MATLAB, Simulink, and Control System Toolbox products already installed.
4. **Ignition** later for SCADA. It is not required for the first PLC/Simulink closed-loop test.

Do not install or follow an old OpenPLC Runtime v3 tutorial. Runtime v4 is headless and is controlled from Editor v4 over HTTPS port 8443. There is no v3-style browser dashboard.

## Part 1 — Start the runtime

1. Start Docker Desktop and wait until the engine reports that it is running.
2. Open PowerShell in `03_OPENPLC/Runtime`.
3. Run:

```powershell
Set-ExecutionPolicy -Scope Process Bypass
.\start_runtime.ps1
```

Equivalent commands:

```powershell
docker compose pull
docker compose up -d
docker ps --filter name=water-treatment-openplc
docker logs --tail 100 water-treatment-openplc
```

Expected local endpoints:

- OpenPLC Editor management connection: `https://localhost:8443`
- Modbus TCP server: `127.0.0.1:5020`

Check the Modbus port with:

```powershell
Test-NetConnection 127.0.0.1 -Port 5020
```

The Modbus port may not start until an uploaded PLC program is running. If port 5020 is initially closed, continue with the Editor setup.

## Part 2 — Create the OpenPLC project

OpenPLC Editor v4 is still evolving, so some menu labels may differ slightly by build. Use the following structure:

1. Create a new standard IEC 61131-3 project named `Water_Treatment_OpenPLC`.
2. Add a **Program POU** named `main` using **Structured Text**.
3. Open `03_OPENPLC/Source/MAIN_Variables.txt` and paste its contents into the POU declaration/variables section.
4. Open `03_OPENPLC/Source/MAIN_Body.st` and paste its contents into the Structured Text implementation/body section.
5. Under the project resource/configuration, create a cyclic task:
   - Name: `MainTask`
   - Interval: `T#100ms`
   - Priority: default or medium
6. Add an instance of program `main` to `MainTask`.
7. Build/compile the project. Resolve any paste-related formatting error before connecting the runtime.

`Water_Treatment_Main_FULL_REFERENCE.st` contains the same declarations and body as one complete reference file. The two paste files are used because IEC editors commonly separate declarations from implementation.

## Part 3 — Connect Editor to Runtime

1. In OpenPLC Editor, add or configure a runtime connection.
2. Runtime address: `https://localhost:8443`
3. On the first connection, create the first runtime user when prompted.
4. Log in using that account.
5. Build and upload the project.
6. Start the PLC from the Editor.
7. Open online/debug mode and monitor:
   - `Comm_Healthy`
   - `Active_Mode`
   - `Batch_State`
   - `P101A_StartCmd` / `P101B_StartCmd`
   - `Pressure_PI_Output`

At this point `Comm_Healthy` remains false until MATLAB or Simulink increments `Plant_Heartbeat` at HR9.

## Part 4 — First communication test from MATLAB

Install Industrial Communication Toolbox through MATLAB Add-On Explorer, restart MATLAB, set the Current Folder to the v0.5.2 project root, and run:

```matlab
START_HERE
check_openplc_requirements
test_openplc_modbus_smoke
```

Expected result:

```text
OPENPLC MODBUS SMOKE TEST: PASSED
```

Then run the longer PLC logic tests:

```matlab
run_openplc_acceptance_tests
```

The acceptance script forces simulated sensor values over Modbus and verifies normal sequencing, pressure control, standby takeover, and communication-loss safe state.

## Stage 5 controller update and live test

Stage 5 corrects the concentration PI gains after validating the controller
against the nonlinear chemical mass-balance plant. Before running Stage 5,
replace the existing POU declaration and body with the v0.5.2 files, rebuild,
upload, and start the PLC again:

1. Paste `03_OPENPLC/Source/MAIN_Variables.txt` into the `main` declaration.
2. Paste `03_OPENPLC/Source/MAIN_Body.st` into the `main` implementation.
3. Confirm these two constants in the declaration:

```text
CONCENTRATION_KP    = 100.0000
CONCENTRATION_KI_TS = 0.1000
```

4. Build, upload, and start the PLC.
5. In MATLAB, run:

```matlab
RUN_STAGE5_FULL_CONTROL
```

The live test takes approximately 260 seconds and saves its CSV, MAT-file,
and PNG evidence under `06_TESTING`.

The v0.5.2 body allows 8.0 seconds for XV-201 open feedback. The simulated
2.0-second first-order valve reaches the 95% open threshold after about
6.0 seconds, so the former 5.0-second supervision setting rejected a healthy
batch during transfer.

## Stop the runtime

From `03_OPENPLC/Runtime`:

```powershell
.\stop_runtime.ps1
```

The named Docker volume preserves the runtime account and uploaded state between restarts.
