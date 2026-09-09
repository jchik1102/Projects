# Automated Water Treatment Control

Software-only three-tank water-treatment simulation with an OpenPLC controller and Ignition Perspective operator interface.

| Platform | Responsibility |
| --- | --- |
| MATLAB | Parameters, PI design, scenarios, automated checks and results |
| Simulink | Tank levels, flows, concentration, pressure and equipment dynamics |
| OpenPLC | Batch sequencing, PI loops, interlocks, pump staging and fault response |
| Ignition | Operator commands, process screens, 102 OPC tags, alarms and history |

This combines control project **v0.5.2** with SCADA **v0.6.1** and the separate **v0.5.2 Ignition tag export**. The source packages are preserved; the README supplies the combined setup order. It is an educational simulation, not a commissioned physical treatment installation.

## Requirements

The archived project targets Windows, MATLAB **R2026a**, OpenPLC **Editor and Runtime v4**, Docker Desktop with WSL 2, and Ignition **8.3** with Perspective, OPC UA/Modbus, historian and alarm-journal capabilities. Configure the appropriate Ignition license/trial for your installation.

MATLAB requires Simulink, Control System Toolbox and Industrial Communication Toolbox. Simulink Test and Simulink Control Design are optional. No physical PLC or separate Python installation is required; the `.py` SCADA scripts run inside Ignition.

Use the same Windows host for MATLAB, Docker and Ignition for the supplied loopback configuration. The Docker image currently uses `latest`; an exact historical image digest was not supplied, so future runtime builds may require adjustments.

## 1. Get the source and initialize MATLAB

Download this repository or clone it:

```sh
git clone https://github.com/jchik1102/Projects.git
```

Set MATLAB Current Folder to `Projects/water-treatment-control`, then run:

```matlab
START_HERE
run_all_tests
```

This initializes parameters and builds/checks the standalone plant and controller models. The four `.slx` models are also included under `02_SIMULINK`. Generated outputs go under `06_TESTING`.

## 2. Start OpenPLC

Start Docker Desktop. In PowerShell, change to `03_OPENPLC/Runtime` and run:

```powershell
docker compose pull
docker compose up -d
docker logs --tail 100 water-treatment-openplc
```

In OpenPLC Editor v4, create a Structured Text Program POU named `main`:

1. Paste `03_OPENPLC/Source/MAIN_Variables.txt` into its declarations.
2. Paste `03_OPENPLC/Source/MAIN_Body.st` into its implementation.
3. Assign an instance of `main` to a cyclic task with interval `T#100ms`.
4. Build, connect to `https://localhost:8443`, create/sign into the runtime account, upload and start the PLC.

The complete `.st` reference is included alongside the two paste files. Keep the v0.5.2 source: it contains the concentration gains and corrected valve-feedback timeout.

The Modbus endpoint is `127.0.0.1:5020`, unit ID `1`. It may become available only after the PLC program starts. See [the detailed OpenPLC instructions](03_OPENPLC/README_FIRST_RUN.md).

Back in MATLAB:

```matlab
START_HERE
test_openplc_modbus_smoke
run_openplc_acceptance_tests
BUILD_OPENPLC_INTEGRATION
```

Run these sequentially. Do not run the smoke/acceptance scripts concurrently with the live Simulink plant; both write plant inputs.

## 3. Configure Ignition

1. In the Gateway, create a Modbus TCP device named **OpenPLC**, host **127.0.0.1**, port **5020**, unit ID **1**. Use the address convention expected by the supplied tag export (for example `[OpenPLC]1.HR1` represents the first holding register). Keep the built-in OPC server named **Ignition OPC UA Server**.
2. Import `04_IGNITION/Water_Treatment_SCADA_COMPLETE_v0_6_1.zip` as an Ignition project, named **Water_Treatment_SCADA**. This is a project export, not a full Gateway backup.
3. Open the project in Designer. In the **default** tag provider, import `04_IGNITION/Water_Treatment_Ignition_Tags_v0_5_2.csv`. The tag root must be `[default]WaterTreatment`. The project ZIP does not contain these Gateway tags.
4. Create a Core Historian named **WaterTreatmentHistory** and an Internal Alarm Journal named **WaterTreatmentJournal** in the Gateway.
5. In the Designer Script Console, run:

```python
SCADASetup.validate()
SCADASetup.run("WaterTreatmentHistory")
```

Expected validation is `expected: 102`, `existing: 102`, `good: 102`, with empty `missing` and `badQuality` lists. The setup configures 18 Boolean alarms and 57 history tags. It also clears operator requests and simulation fault requests, so run it before the demonstration.

Save the project and launch a **Perspective session** from the Gateway. With the usual local Gateway port, the session address is:

```text
http://localhost:8088/data/perspective/client/Water_Treatment_SCADA/
```

Use the actual Gateway port and project name if you changed them. Available pages include overview, treatment, pumps, pressure, alarms, trends, maintenance, simulation and diagnostics.

The CSV already defines raw-to-engineering scaling. Do not apply a second scaling layer. The HMI writes request bits and permitted setpoints; PLC outputs remain controller-owned.

## 4. Run the integrated demonstration

Keep Docker, the PLC program and Ignition running. Open the Perspective session first, then in MATLAB run:

```matlab
START_HERE
RUN_STAGE5_FULL_CONTROL
```

The supplied demonstration runs for approximately **260 seconds**, with a complete treatment batch, pressure staging and injected lead-pump failure/takeover. It uses an accelerated commissioning profile: treatment-tank volume and dosing capacity are each scaled to 10% while preserving their ratio. It is not the nominal full-size process timing.

Watch the tank levels, batch state and pump commands in the Perspective session. MATLAB saves measurements and results under `06_TESTING` and attempts to stop/clear requests when finished. This command is the recovered full-control commissioning runner; no separate later final-run script was recovered.

For another session, start Docker/PLC and Ignition, open the project in MATLAB, run `START_HERE`, then `RUN_STAGE5_FULL_CONTROL`. The one-time Ignition import/setup does not need repeating.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Modbus timeout/refused | Start the PLC; run `Test-NetConnection 127.0.0.1 -Port 5020`; inspect Docker logs and close competing MATLAB runs. |
| Missing/bad Ignition tags | Import the CSV into `default`; confirm device/server names, unit ID and Modbus addressing. |
| SCADA button script errors | Import v0.6.1, which calls `SCADASetup` directly rather than `project.SCADASetup`. |
| No tank movement | Ignition is the supervisor; start the MATLAB/Simulink run and check heartbeat health. |
| No history | Confirm provider names, run setup once and collect data while the plant runs. |
| MATLAB uses stale bridge/model code | Close the integrated model without saving, run `clear classes` and `clear functions`, then initialize and rebuild. |

Stop the plant before shutting down. To stop the container, run `docker compose down` from `03_OPENPLC/Runtime`. Do not add `-v` if you want to preserve its runtime account and uploaded program.

## Source layout and verification

| Folder | Contents |
| --- | --- |
| `01_MATLAB` | Model builders, controller design, Modbus bridge, scenarios and tests |
| `02_SIMULINK` | Plant and controller models |
| `03_OPENPLC` | Structured Text source, Docker configuration and PLC instructions |
| `04_IGNITION` | Importable project ZIP, extracted editable resources and OPC tag CSV |
| `05_INTEGRATION` | Register maps, gains and connection notes |
| `06_TESTING` | Archived results and generated-output folders |
| `00_PROJECT_MANAGEMENT` | Original revision notes, implementation plan and provenance |

The root README is the combined setup guide. Older documents describe intermediate stages, including references to Ignition as future work. The implementation plan is a scope/checklist document, not proof that every proposed feature was implemented.

Repository assembly checks cover archive integrity, JSON parsing, SCADA tag coverage and source preservation. MATLAB, OpenPLC and Ignition were **not executed during repository assembly**. Included historical CSV results are from the source archive, not a fresh end-to-end run. The live checks above remain necessary on your installed software.
