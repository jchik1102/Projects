# Automated Water Pumping and Treatment Station

## Complete Project Implementation Plan

| **Project platforms** | MATLAB, Simulink, OpenPLC, Ignition                        |
|-----------------------|------------------------------------------------------------|
| **Project type**      | Software-only industrial process control and SCADA project |
| **Prepared for**      | Jonathan Chik                                              |
| **Document date**     | August 1, 2026                                             |

> **Project definition**
>
> MATLAB performs engineering analysis, controller design, scenario configuration, automated validation, and reporting. Simulink models the physical process and instrumentation. OpenPLC executes the operational controls, sequencing, PID loops, interlocks, permissives, and fault handling. Ignition provides the operator interface, alarms, trends, history, and supervisory commands.

**Final rule: Simulink is the plant, OpenPLC is the controller, Ignition is the supervisor, and MATLAB is the engineering and validation environment.**

# Document Control

| **Item**                         | **Definition**                                                                                                           |
|----------------------------------|--------------------------------------------------------------------------------------------------------------------------|
| Document objective               | Provide a complete, staged checklist for designing, building, integrating, validating, and presenting the project.       |
| Intended use                     | Use this document as the project master plan and completion checklist.                                                   |
| Primary completion standard      | Every stage must pass its stated gate and preserve evidence before the next stage begins.                                |
| Excluded scope                   | No Revit, Python, physical PLC hardware, detailed municipal process design, or safety certification.                     |
| Recommended development approach | Build and validate one subsystem at a time; do not configure the entire communications map before one closed loop works. |

# Contents

- 1. Project Definition and Objectives
- 2. System Architecture and Responsibility Boundaries
- 3. Scope, Assumptions, and Completion Criteria
- 4. Project Folder, Naming, and Version-Control Structure
- 5. Process Design and Equipment Definition
- 6. MATLAB Work Package
- 7. Simulink Work Package
- 8. OpenPLC Work Package
- 9. Ignition Work Package
- 10. Communications and Integration Work Package
- 11. Alarm, Interlock, and Permissive Design
- 12. Testing and Validation Work Package
- 13. Project Stages and Gates
- 14. Final Deliverables and Portfolio Presentation
- 15. Final Master Checklist
- Appendix A. Suggested Tag and Register Map
- Appendix B. Suggested Alarm List
- Appendix C. Suggested Test Matrix
- Appendix D. Definition of Done

# 1. Project Definition and Objectives

## 1.1 Final project statement

Design, simulate, control, supervise, and validate a software-only automated water pumping and treatment station. The physical water process will be represented in Simulink, the operational control program will execute in OpenPLC, the operator interface will be built in Ignition, and MATLAB will provide the engineering calculations, controller design, scenario management, automated testing, analysis, and final reporting.

## 1.2 Primary technical objectives

- [ ] Create a dynamic three-tank water process with realistic levels, flows, pressure, concentration, actuator lag, and sensor behavior.
- [ ] Implement a complete treatment batch sequence: ready, fill, dose, mix, verify, transfer, complete, and faulted.
- [ ] Implement distribution-pressure PI control using variable-speed booster pumps.
- [ ] Implement treatment-concentration PI control using a chemical dosing pump.
- [ ] Implement lead-lag pump rotation, standby takeover, failure-to-start handling, and runtime balancing.
- [ ] Implement manual, automatic, stopped, and maintenance operating modes.
- [ ] Implement industrial permissives, interlocks, trip logic, alarm latching, reset logic, and communication watchdogs.
- [ ] Exchange process values and commands between Simulink and OpenPLC.
- [ ] Connect Ignition to OpenPLC for HMI commands, equipment status, alarms, trends, and historical data.
- [ ] Use MATLAB to tune controllers, configure test scenarios, run automated evaluations, calculate performance metrics, and produce final plots and summary tables.

## 1.3 Portfolio outcomes

- Demonstrates classical and practical process-control design.
- Demonstrates IEC 61131-3 PLC programming using Ladder Diagram and Structured Text.
- Demonstrates industrial communications and tag mapping.
- Demonstrates SCADA/HMI design, alarming, trending, and operator workflow.
- Demonstrates verification against explicit acceptance criteria rather than subjective visual judgment.
- Complements the existing microgrid project by focusing on industrial automation and process control instead of power optimization.

## 1.4 Final project description for a resume

> **Resume-ready summary**
>
> Developed a software-in-the-loop water treatment control system using MATLAB/Simulink, OpenPLC, and Ignition. Modelled nonlinear tank, pressure, dosing, actuator, and sensor dynamics; implemented PLC sequencing, PI control, lead-lag pump management, permissives, interlocks, and alarm handling; integrated the plant and PLC over industrial communications; and automated multi-scenario performance validation and reporting in MATLAB.

# 2. System Architecture and Responsibility Boundaries

## 2.1 Architecture

```text
MATLAB

- Parameters, calculations, controller design, scenarios, automated analysis
- Starts and evaluates Simulink tests

Simulink Plant Model

- Tanks, pumps, valves, pressure, concentration, sensors, actuator feedback
- Reads actuator commands from OpenPLC
- Writes simulated measurements and feedback to OpenPLC

OpenPLC Runtime

- Modes, sequencing, PI control, interlocks, permissives, alarms
- Reads plant measurements
- Writes actuator commands and equipment states

Ignition SCADA

- Overview, faceplates, controls, alarms, history, trends, diagnostics
- Reads PLC states and process values
- Writes operator requests and approved setpoints
```

## 2.2 Responsibility matrix

| **Function**               | **MATLAB**        | **Simulink**                   | **OpenPLC**        | **Ignition**                  |
|----------------------------|-------------------|--------------------------------|--------------------|-------------------------------|
| Physical tank dynamics     | Analyze           | Own                            | No                 | Display                       |
| Pump and valve dynamics    | Parameterize      | Own                            | Command only       | Display                       |
| Pressure PI design         | Design and verify | Temporary test controller only | Final execution    | Setpoint and trends           |
| Concentration PI design    | Design and verify | Temporary test controller only | Final execution    | Setpoint and trends           |
| Batch sequencing           | Test and analyze  | Plant response only            | Own                | Display and operator requests |
| Interlocks and permissives | Test matrix       | Provide trip conditions        | Own                | Display reason for inhibit    |
| Alarms                     | Verify activation | Provide fault stimuli          | Detect and latch   | Annunciate and journal        |
| Historical trends          | Offline analysis  | Log source signals             | Expose values      | Own operator history          |
| Scenario management        | Own               | Execute scenario inputs        | Respond            | Optional trigger panel        |
| Final reporting            | Own               | Provide logs                   | Provide state data | Provide screenshots/history   |

## 2.3 Boundary rules

- [ ] Do not place final operational sequencing in Simulink.
- [ ] Do not let Ignition directly force physical actuator outputs; Ignition sends requests and OpenPLC decides whether they are permitted.
- [ ] Do not use MATLAB as the real-time operational controller.
- [ ] Do not bypass PLC interlocks in manual mode.
- [ ] Do not use perfect sensor values; include realistic lag, noise, limits, and faults.
- [ ] Do not accept controller performance without quantitative pass/fail criteria.
- [ ] Do not expand the project into detailed chemical engineering, pipe-network design, or municipal compliance.

# 3. Scope, Assumptions, and Completion Criteria

## 3.1 Included process

- Raw-water storage tank T-101.
- Treatment and mixing tank T-201.
- Clean-water storage tank T-301.
- Duty/standby raw-water transfer pumps P-101A and P-101B.
- Treatment-transfer pump P-201.
- Duty/standby booster pumps P-301A and P-301B.
- Chemical dosing pump DP-201.
- Treatment-tank mixer M-201.
- Motorized process valves and open/closed feedback.
- Level, flow, pressure, concentration, motor, and valve feedback signals.
- Distribution-demand disturbances and equipment/sensor faults.

## 3.2 Excluded scope

- Detailed piping hydraulics, water hammer, pipe sizing, pump manufacturer selection, or computational fluid dynamics.
- Real chemical hazard analysis or municipal drinking-water certification.
- Physical PLC hardware, wiring, I/O modules, VFDs, relays, and sensors.
- Revit, AutoCAD, ETAP, Python, or external cloud services.
- Cybersecurity certification or safety-integrity-level design.
- Production deployment of Ignition or commercial licensing decisions.

## 3.3 Modelling assumptions

| **Area**      | **Assumption**                                                                                             |
|---------------|------------------------------------------------------------------------------------------------------------|
| Tank geometry | Each tank has constant cross-sectional area and a level bounded between 0% and 100%.                       |
| Flow          | Pump flow is represented by a rated maximum multiplied by speed command and actuator dynamics.             |
| Valves        | On/off valves have finite opening time and open/closed feedback.                                           |
| Pressure      | Distribution pressure is represented by a reduced-order dynamic model suitable for controller development. |
| Concentration | Treatment concentration is modelled with mass balance, mixing, dosing, and optional decay.                 |
| Sensors       | Signals include configurable lag, noise, bias, limits, and injected faults.                                |
| Communication | Plant and PLC exchange scaled integer values through a deterministic register map.                         |
| Timing        | Simulink is paced near real time whenever the external OpenPLC controller is connected.                    |

## 3.4 Project completion criteria

- [ ] All four software environments are installed and can run the project.
- [ ] The standalone Simulink plant passes physical sanity checks.
- [ ] OpenPLC can read at least one plant measurement and command at least one simulated actuator.
- [ ] The automatic treatment sequence completes a normal batch without manual intervention.
- [ ] Both PI loops meet their response requirements under normal disturbances.
- [ ] Lead-lag pumps alternate and the standby pump takes over after a lead-pump failure.
- [ ] Ignition displays all critical process values and can issue permitted operator commands.
- [ ] Alarm history records the designed abnormal scenarios.
- [ ] Every required validation scenario produces a saved pass/fail result.
- [ ] The final project folder contains source files, exports, screenshots, plots, results, and documentation.

# 4. Project Folder, Naming, and Version-Control Structure

## 4.1 Required folder structure

```text
Water_Treatment_Control_Project/

|

|-- 00_Project_Management/

| |-- Master_Checklist.docx

| |-- Change_Log.xlsx

| |-- Assumptions_and_Decisions.docx

|

|-- 01_MATLAB/

| |-- initialize_water_plant.m

| |-- define_register_map.m

| |-- derive_plant_models.m

| |-- tune_pressure_controller.m

| |-- tune_concentration_controller.m

| |-- run_all_scenarios.m

| |-- generate_final_results.m

| |-- Scenarios/

| |-- Analysis/

| `-- Results/

|

|-- 02_SIMULINK/

| |-- Water_Treatment_Plant.slx

| |-- Pressure_Controller_Test.slx

| |-- Concentration_Controller_Test.slx

| `-- Libraries/

|

|-- 03_OPENPLC/

| |-- Water_Treatment_PLC.st

| |-- Ladder_Programs/

| |-- Structured_Text/

| |-- Exports/

| `-- Screenshots/

|

|-- 04_IGNITION/

| |-- Project_Backups/

| |-- Tag_Exports/

| |-- Alarm_Exports/

| |-- Screenshots/

| `-- Historian_Exports/

|

|-- 05_INTEGRATION/

| |-- Register_Map.xlsx

| |-- Tag_Map.xlsx

| |-- Communication_Tests/

| `-- Watchdog_Test_Results/

|

|-- 06_TESTING/

| |-- Test_Procedures/

| |-- Test_Results/

| |-- Plots/

| |-- Logs/

| `-- Final_Validation_Summary.xlsx

|

`-- 07_DOCUMENTATION/

|-- System_Architecture.docx

|-- Functional_Design_Specification.docx

|-- Control_Narrative.docx

|-- Alarm_and_Interlock_Matrix.xlsx

|-- Final_Report.docx

`-- Portfolio_Summary.pdf
```

## 4.2 Naming rules

- [ ] Use equipment tags consistently: T-101, P-101A, P-101B, T-201, P-201, DP-201, M-201, T-301, P-301A, and P-301B.
- [ ] Use signal suffixes consistently: \_PV, \_SP, \_Cmd, \_Fb, \_Run, \_Trip, \_Avail, \_Alarm, \_Mode, and \_Fault.
- [ ] Use engineering units in tag descriptions and comments.
- [ ] Use one source-of-truth register map; never maintain separate uncoordinated address lists.
- [ ] Include a revision number in exported PLC, Ignition, and report files.
- [ ] Save a known-good backup after each stage gate passes.

## 4.3 Change-control procedure

1.  Record the proposed change and reason.

2.  Identify which subsystems and tests are affected.

3.  Update the source-of-truth parameter or register document first.

4.  Implement the change in the affected software.

5.  Re-run the affected stage tests.

6.  Update screenshots, plots, and evidence.

7.  Increment the project revision and preserve the previous known-good version.

# 5. Process Design and Equipment Definition

## 5.1 Process flow

8.  Raw water enters and is stored in T-101.

9.  P-101A or P-101B transfers raw water into treatment tank T-201.

10. DP-201 adds treatment chemical while M-201 mixes the batch.

11. OpenPLC verifies concentration, minimum treatment time, and sensor health.

12. P-201 transfers an accepted batch from T-201 to clean-water tank T-301.

13. P-301A and P-301B maintain distribution pressure while network demand changes.

## 5.2 Equipment list

| **Tag**  | **Equipment**           | **Primary function**                   | **Controlled by**                        |
|----------|-------------------------|----------------------------------------|------------------------------------------|
| T-101    | Raw-water tank          | Stores incoming untreated water        | Plant dynamics; level supervision by PLC |
| P-101A/B | Transfer pumps          | Fill treatment tank                    | OpenPLC duty/standby logic               |
| T-201    | Treatment tank          | Batch dosing, mixing, and verification | OpenPLC sequence                         |
| DP-201   | Dosing pump             | Adjusts treatment concentration        | OpenPLC concentration PI                 |
| M-201    | Mixer                   | Improves concentration uniformity      | OpenPLC sequence                         |
| P-201    | Treatment-transfer pump | Moves accepted water to T-301          | OpenPLC sequence                         |
| T-301    | Clean-water tank        | Buffers treated water for distribution | Plant dynamics; level protection by PLC  |
| P-301A/B | Booster pumps           | Maintain distribution pressure         | OpenPLC pressure PI and staging          |

## 5.3 Instrument list

| **Tag** | **Measurement or feedback**     | **Units/type** | **PLC use**                                                  |
|---------|---------------------------------|----------------|--------------------------------------------------------------|
| LIT-101 | Raw-water tank level            | %              | Low and low-low protection; fill availability                |
| LIT-201 | Treatment-tank level            | %              | Batch fill target; high-high protection; transfer completion |
| LIT-301 | Clean-water tank level          | %              | Distribution permissive; high-high transfer stop             |
| FIT-101 | Flow into treatment tank        | L/s            | No-flow detection and filling verification                   |
| FIT-201 | Treatment transfer flow         | L/s            | Transfer verification                                        |
| FIT-301 | Distribution demand flow        | L/s            | Performance monitoring and disturbance input                 |
| AIT-201 | Treatment concentration         | mg/L           | Concentration PI and batch quality verification              |
| PIT-301 | Distribution pressure           | kPa            | Pressure PI and pressure alarms                              |
| ZS-201  | Treatment outlet valve feedback | Digital        | Pump permissive and valve timeout                            |
| MS-201  | Mixer running feedback          | Digital        | Dosing permissive and failure detection                      |

## 5.4 Initial design values

| **Parameter**                 | **Initial value** | **Purpose**                               |
|-------------------------------|-------------------|-------------------------------------------|
| T-101 maximum volume          | 100 m3            | Source storage                            |
| T-201 maximum volume          | 60 m3             | Batch treatment volume                    |
| T-301 maximum volume          | 125 m3            | Clean-water buffer                        |
| T-201 batch fill setpoint     | 80%               | Target fill level before dosing           |
| Pressure setpoint             | 400 kPa           | Distribution control target               |
| Concentration setpoint        | 1.20 mg/L         | Treatment target                          |
| Acceptable concentration band | 1.10 to 1.30 mg/L | Batch quality criterion                   |
| High-high concentration       | 1.50 mg/L         | Dosing trip and batch rejection threshold |
| Nominal network demand        | 30 L/s            | Base operating point                      |
| High demand                   | 55 to 70 L/s      | Pressure disturbance test                 |
| Mixing time                   | 120 s             | Initial accelerated project value         |

## 5.5 Required operating modes

| **Mode**    | **Required behavior**                                                                                                       |
|-------------|-----------------------------------------------------------------------------------------------------------------------------|
| Stopped     | All process commands off. State machine reset or held in a safe stopped state. Alarms remain visible.                       |
| Automatic   | PLC performs treatment sequencing, pressure control, dosing control, staging, alarms, and fault recovery.                   |
| Manual      | Operator may request individual equipment operation, but all critical permissives and interlocks remain active.             |
| Maintenance | Automatic sequence disabled. Selected equipment can be tested under controlled permissives and explicit maintenance enable. |

# 6. MATLAB Work Package

## 6.1 MATLAB purpose

MATLAB is the engineering and validation layer. It defines the plant parameters, derives reduced-order models, tunes the controllers, creates scenarios, starts simulations, evaluates pass/fail criteria, generates plots, calculates energy and performance metrics, and produces the final validation summary. MATLAB must not replace OpenPLC as the operational controller.

## 6.2 MATLAB environment setup

- [ ] Confirm MATLAB and Simulink open without licensing errors.
- [ ] Confirm the required control-design functionality is available in the installed MATLAB products.
- [ ] Confirm the communication method selected for OpenPLC can be used from Simulink or MATLAB in the installed release.
- [ ] Create the project folder and add only the required project directories to the MATLAB path.
- [ ] Create a startup script that clears stale variables, loads parameters, and checks dependencies.
- [ ] Create a results directory automatically if it does not exist.
- [ ] Record the installed MATLAB release and toolbox versions in the project documentation.

## 6.3 Parameter initialization script

Create \`initialize_water_plant.m\` as the single source of truth for physical and simulation parameters.

- [ ] Define simulation sample time, communication update period, real-time pacing setting, and default stop time.
- [ ] Define tank areas, heights, maximum volumes, and initial levels.
- [ ] Define pump maximum flows, time constants, minimum command, and trip behavior.
- [ ] Define valve opening and closing time constants.
- [ ] Define pressure-model gain, time constant, demand coefficient, and initial pressure.
- [ ] Define chemical stock concentration, dosing gain, process time constant, mixing time constant, and decay constant.
- [ ] Define sensor noise, lag, bias, scaling, limits, and fault parameters.
- [ ] Define alarm thresholds and hysteresis values.
- [ ] Define controller update times, output limits, initial gains, and anti-windup limits.
- [ ] Define batch setpoints, timers, retry limits, and timeout limits.
- [ ] Print a concise parameter summary when initialization completes.

## 6.4 Register-map definition script

Create \`define_register_map.m\` so all addresses and scaling rules are generated from one MATLAB structure.

- [ ] Assign separate address ranges for plant measurements, PLC actuator commands, operator setpoints, PLC status, and fault injection.
- [ ] Assign digital ranges for equipment feedback, commands, operator requests, alarm bits, and fault requests.
- [ ] Define engineering-unit scaling for every analog value.
- [ ] Include tag name, description, owner, read/write direction, data type, register, scaling, units, minimum, and maximum.
- [ ] Export the map to CSV or Excel for use when configuring OpenPLC and Ignition.
- [ ] Add automated checks for duplicate addresses, duplicate tag names, undefined scaling, and out-of-range addresses.

## 6.5 Mathematical model derivation

- [ ] Derive the level balance for each tank: area times rate of level change equals inflow minus outflow.
- [ ] Calculate theoretical fill and drain times for nominal pump flow.
- [ ] Derive the reduced-order booster-pressure model.
- [ ] Derive the reduced-order concentration model around a nominal operating point.
- [ ] Document assumptions used when linearizing nonlinear flow and concentration equations.
- [ ] Create transfer functions or state-space models for pressure and concentration control design.
- [ ] Verify poles, steady-state gains, and expected time constants.
- [ ] Save derivations and resulting model parameters in a dedicated analysis script.

## 6.6 Pressure-controller design

- [ ] Define the pressure plant model used for controller tuning.
- [ ] Select a PI controller rather than derivative action for the first implementation.
- [ ] Choose target rise time, settling time, overshoot, and allowable steady-state error.
- [ ] Tune preliminary gains using MATLAB control-design tools or an analytical method.
- [ ] Simulate the closed-loop step response.
- [ ] Test sensitivity to plant gain and time-constant variation.
- [ ] Discretize the controller at the intended PLC update period.
- [ ] Document Kp, Ki, sample time, output limits, integral limits, and anti-windup method.
- [ ] Export the final recommended gains for OpenPLC implementation.

## 6.7 Concentration-controller design

- [ ] Define the concentration plant approximation around the normal batch condition.
- [ ] Select a slower PI update period than the pressure loop.
- [ ] Tune gains for stable dosing without high overshoot.
- [ ] Apply output saturation from 0% to 100%.
- [ ] Design anti-windup behavior for saturated dosing.
- [ ] Define a concentration deadband or tolerance band used for batch verification.
- [ ] Define minimum dosing time, stability time, maximum dosing time, and retry limit.
- [ ] Simulate nominal, weak-dose, and excessive-dose conditions.
- [ ] Export the final recommended gains for OpenPLC implementation.

## 6.8 Demand and disturbance generation

- [ ] Create a nominal distribution-demand profile.
- [ ] Create step, ramp, pulse, and repeated demand changes.
- [ ] Create raw-water source loss and reduced-inflow profiles.
- [ ] Create pump degradation and complete pump-failure signals.
- [ ] Create valve stuck-open and stuck-closed conditions.
- [ ] Create pressure, level, concentration, flow, and communication faults.
- [ ] Store every scenario in a named configuration structure rather than hard-coding fault times inside Simulink.

## 6.9 Scenario scripts

| **Script**                       | **Required purpose**                           |
|----------------------------------|------------------------------------------------|
| scenario_01_normal_operation.m   | Normal batch and normal pressure operation     |
| scenario_02_demand_step.m        | Large distribution-demand increase             |
| scenario_03_booster_failure.m    | Lead booster-pump failure and standby takeover |
| scenario_04_low_raw_level.m      | Raw-water low-low protection                   |
| scenario_05_overconcentration.m  | Dosing shutdown and batch rejection            |
| scenario_06_valve_failure.m      | Valve feedback timeout and transfer prevention |
| scenario_07_sensor_freeze.m      | Frozen concentration or level sensor detection |
| scenario_08_communication_loss.m | Watchdog timeout and safe-state response       |
| scenario_09_failed_batch.m       | Treatment retry and final rejection            |
| scenario_10_manual_mode.m        | Manual requests with interlock enforcement     |

## 6.10 Automated execution framework

- [ ] Create \`run_all_scenarios.m\`.
- [ ] Load the common parameter file once per clean test run.
- [ ] Load each scenario configuration.
- [ ] Set model stop time and relevant scenario variables.
- [ ] Run the Simulink model.
- [ ] Save raw logs using a consistent scenario name and timestamp.
- [ ] Call the relevant analysis function.
- [ ] Store pass/fail status, metrics, alarm results, and notes in a results structure.
- [ ] Continue to the next scenario even if one scenario fails, while preserving the error message.
- [ ] Generate a final summary after all scenarios complete.

## 6.11 Automated performance analysis

- [ ] Calculate minimum, maximum, mean, and final pressure.
- [ ] Calculate pressure rise time, settling time, overshoot, and steady-state error after each disturbance.
- [ ] Calculate concentration maximum, minimum, final error, time in tolerance, and dosing duration.
- [ ] Calculate tank fill and transfer times.
- [ ] Calculate pump takeover delay after a failure.
- [ ] Calculate alarm detection delay and safe-state delay.
- [ ] Calculate number of batch retries and final batch outcome.
- [ ] Verify no rejected water is transferred to T-301.
- [ ] Calculate estimated pump energy use from flow, head, efficiency, and operating time.
- [ ] Compare OpenPLC responses with the earlier temporary Simulink controller where applicable.

## 6.12 MATLAB-generated evidence

- [ ] Controller design plots.
- [ ] Nominal pressure response plot.
- [ ] Pressure disturbance and pump-staging plot.
- [ ] Concentration response and dosing-output plot.
- [ ] Treatment-state timeline.
- [ ] Tank-level plot.
- [ ] Lead-pump failure and standby takeover plot.
- [ ] Alarm and fault timeline.
- [ ] Scenario summary table.
- [ ] Controller comparison table.
- [ ] Energy-use summary.
- [ ] Final pass/fail report saved to a reusable file format.

## 6.13 MATLAB completion gate

| **Gate**              | MATLAB Gate M1                                                                                                                                                                                                                               |
|-----------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| **Purpose**           | Confirm that the engineering environment and automated-analysis framework are ready before full integration.                                                                                                                                 |
| **Pass requirements** | Initialization runs without errors; model parameters are documented; register map exports successfully; both PI designs are documented; all scenario configurations load; analysis functions accept test data; no duplicate addresses exist. |
| **Evidence to save**  | Parameter summary, exported register map, controller-design plots, controller gain sheet, scenario list, and a dry-run validation report.                                                                                                    |

# 7. Simulink Work Package

## 7.1 Simulink purpose

Simulink represents the physical plant, actuator dynamics, sensor behavior, process disturbances, fault injection, and communication interface. The final model must not contain the operational batch sequence or final PI controllers except for temporary isolated controller-validation models.

## 7.2 Top-level model structure

```text
Water_Treatment_Plant.slx

|

|-- Raw_Water_System

|-- Treatment_System

|-- Clean_Water_System

|-- Distribution_Network

|-- Pump_Actuators

|-- Valve_Actuators

|-- Sensor_Models

|-- Fault_Injection

|-- PLC_Input_Interface

|-- PLC_Output_Interface

|-- Watchdog_and_Communication

`-- Data_Logging
```

## 7.3 Model configuration

- [ ] Select a solver suitable for the continuous plant and the required real-time behavior.
- [ ] Set a deterministic base sample time for discrete interfaces.
- [ ] Configure signal logging for all required plant, controller, equipment, state, and fault signals.
- [ ] Name signals consistently with the master tag map.
- [ ] Use model callbacks only for clearly documented initialization tasks.
- [ ] Avoid hidden workspace dependencies.
- [ ] Use data dictionaries or structured workspace variables consistently.
- [ ] Enable model diagnostics for unconnected signals, data-type mismatches, and algebraic loops.

## 7.4 Tank subsystems

- [ ] Implement T-101, T-201, and T-301 as separate reusable subsystems.
- [ ] Calculate volume from level and cross-sectional area.
- [ ] Calculate level derivative from total inflow minus total outflow.
- [ ] Apply physical saturation at empty and full conditions.
- [ ] Prevent negative volume and physically impossible flow from an empty tank.
- [ ] Expose actual level, actual volume, high limit, and low limit signals.
- [ ] Add overflow and empty-tank diagnostic flags.
- [ ] Verify mass balance numerically over a nominal simulation.

## 7.5 Pump actuator subsystems

- [ ] Create a reusable pump model with command, availability, trip, degradation, and feedback inputs.
- [ ] Represent finite acceleration and deceleration using first-order or rate-limited dynamics.
- [ ] Calculate actual flow from speed command and available upstream water.
- [ ] Set flow to zero when unavailable, tripped, or starved.
- [ ] Generate running feedback after a configurable start delay.
- [ ] Generate stopped feedback after a configurable stop delay.
- [ ] Provide failed-to-start and failed-to-stop fault-injection capabilities.
- [ ] Log speed command, actual speed, actual flow, run feedback, and trip state.

## 7.6 Valve actuator subsystems

- [ ] Create a reusable on/off valve subsystem.
- [ ] Model finite opening and closing time.
- [ ] Generate open and closed limit feedback.
- [ ] Allow stuck-open, stuck-closed, and slow-travel faults.
- [ ] Prevent downstream flow when actual valve position is closed.
- [ ] Provide actual position for Ignition display even if the PLC uses only limit feedback.

## 7.7 Treatment concentration subsystem

- [ ] Calculate current treatment-tank volume.
- [ ] Implement chemical mass balance using raw inflow, dosing inflow, treated outflow, and optional decay.
- [ ] Include a dosing-pump effectiveness factor.
- [ ] Include mixer effectiveness or mixing lag.
- [ ] Prevent concentration calculation from dividing by zero at very low tank volume.
- [ ] Generate actual concentration and local measured concentration.
- [ ] Provide weak-dose, excessive-dose, and failed-mixer fault options.
- [ ] Verify concentration responds in the expected direction for dosing changes.

## 7.8 Distribution-pressure subsystem

- [ ] Implement a reduced-order pressure dynamic model.
- [ ] Use the combined effect of P-301A and P-301B actual speed or flow.
- [ ] Subtract the effect of distribution demand.
- [ ] Apply physically reasonable pressure limits.
- [ ] Include pressure sensor dynamics separately from actual pressure.
- [ ] Provide demand step, ramp, pulse, and oscillatory disturbance options.
- [ ] Verify that increasing pump command increases pressure and increasing demand decreases pressure.

## 7.9 Sensor models

- [ ] Create reusable analog-sensor and digital-feedback subsystems.
- [ ] Add configurable first-order lag.
- [ ] Add configurable white noise or bounded noise.
- [ ] Add fixed bias and drift options.
- [ ] Apply resolution or quantization consistent with the register scaling.
- [ ] Apply normal operating limits.
- [ ] Support frozen, zero, maximum, noisy, drifting, and disconnected faults.
- [ ] Generate quality or health flags where useful.

## 7.10 Fault-injection subsystem

- [ ] Provide one structured scenario input for all faults.
- [ ] Allow time-based activation and reset.
- [ ] Support pump trip, pump degradation, valve failure, sensor freeze, sensor drift, source loss, dosing weakness, mixer failure, and communication loss.
- [ ] Expose active-fault status for logging.
- [ ] Ensure fault injection affects the physical plant or instrument response, not the PLC's internal alarm bits directly.
- [ ] Provide a full simulation reset path.

## 7.11 PLC communication interface

- [ ] Read actuator commands generated by OpenPLC.
- [ ] Write sensor process values and equipment feedback to OpenPLC.
- [ ] Apply the master scaling rules before writing values.
- [ ] Apply inverse scaling after reading commands.
- [ ] Clamp every received command to its valid range.
- [ ] Use default safe values before the first valid communication update.
- [ ] Implement communication health, stale-data detection, and an incrementing heartbeat.
- [ ] Log raw registers and engineering-unit signals during initial integration.

## 7.12 Temporary controller-validation models

- [ ] Create \`Pressure_Controller_Test.slx\` with the pressure plant and a temporary Simulink PI controller.
- [ ] Create \`Concentration_Controller_Test.slx\` with the concentration plant and a temporary Simulink PI controller.
- [ ] Validate MATLAB-designed gains before implementing them in OpenPLC.
- [ ] Test output saturation and anti-windup.
- [ ] Record benchmark responses to compare with the OpenPLC implementation.
- [ ] Remove or bypass temporary controllers in the integrated plant model.

## 7.13 Standalone Simulink tests

- [ ] Tank filling matches the analytical filling-time estimate.
- [ ] Tank draining matches the analytical draining-time estimate.
- [ ] Tank levels remain between 0% and 100%.
- [ ] Total water mass is conserved except for defined external inflows and outflows.
- [ ] Pump flow follows command with the intended time constant.
- [ ] Valve feedback changes after the intended travel time.
- [ ] Pressure increases with booster command and decreases with demand.
- [ ] Concentration increases with dosing and does not become negative.
- [ ] Sensor faults produce the intended measured behavior without corrupting the actual plant state.
- [ ] All logged signals have correct names and units.

## 7.14 Simulink completion gate

| **Gate**              | Simulink Gate S1                                                                                                                                                                                                                                |
|-----------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| **Purpose**           | Confirm the plant model is physically coherent before connecting the external PLC.                                                                                                                                                              |
| **Pass requirements** | All standalone plant tests pass; mass-balance error is acceptable; no negative volumes occur; all actuators and sensors respond correctly; temporary PI benchmarks meet the selected targets; communication interface uses valid safe defaults. |
| **Evidence to save**  | Standalone test plots, analytical-versus-simulated timing table, mass-balance check, sensor-fault screenshots, and controller benchmark plots.                                                                                                  |

# 8. OpenPLC Work Package

## 8.1 OpenPLC purpose

OpenPLC is the final operational controller. It owns operating modes, equipment permissives, interlocks, start and stop logic, batch sequencing, PI execution, lead-lag rotation, alarm detection, alarm latching, watchdog response, runtime counters, and command outputs.

## 8.2 PLC program organization

```text
MAIN

|

|-- IO_Scaling

|-- Communication_Health

|-- Mode_Control

|-- Equipment_Permissives

|-- Equipment_Commands

|-- Batch_State_Machine

|-- Pressure_PI

|-- Concentration_PI

|-- Booster_Pump_Staging

|-- Lead_Lag_Rotation

|-- Alarm_Logic

|-- Runtime_and_Start_Counters

|-- Diagnostics

`-- Output_Mapping
```

## 8.3 Data types and tag structure

- [ ] Create clearly separated variables for raw registers, scaled engineering values, internal commands, output commands, feedback, alarms, and HMI requests.
- [ ] Use descriptive variable names that match the master tag map.
- [ ] Document every externally mapped variable.
- [ ] Use enumerated or integer constants for operating modes and batch states.
- [ ] Create reusable function blocks where supported for motors, valves, PI control, and timers.
- [ ] Separate one-shot requests from maintained commands.
- [ ] Initialize all outputs to safe values on startup.

## 8.4 I/O scaling

- [ ] Convert scaled integer sensor registers to engineering values.
- [ ] Validate minimum and maximum limits before using each value.
- [ ] Convert internal actuator commands to scaled output registers.
- [ ] Saturate commands before conversion.
- [ ] Handle negative or invalid raw values safely.
- [ ] Expose raw and scaled values for integration troubleshooting.

## 8.5 Communication-health logic

- [ ] Read an incrementing heartbeat from Simulink.
- [ ] Store the last observed heartbeat value.
- [ ] Reset a watchdog timer whenever the heartbeat changes.
- [ ] Declare plant communication failed when the heartbeat remains unchanged beyond the timeout.
- [ ] Command process outputs to the defined safe state after communication failure.
- [ ] Latch a communication alarm.
- [ ] Require a healthy period and operator reset before automatic restart.
- [ ] Provide a separate Ignition communication-quality indication where available.

## 8.6 Mode control

- [ ] Implement stopped, automatic, manual, and maintenance modes.
- [ ] Define who is permitted to request each mode.
- [ ] Prevent mode changes during unsafe transitions where necessary.
- [ ] Force the automatic sequence to a safe stopped or faulted state when automatic mode is removed.
- [ ] Retain critical interlocks in all modes.
- [ ] Display active mode and requested mode separately.
- [ ] Provide clear mode-transition diagnostics.

## 8.7 Equipment motor logic

- [ ] Create standard logic for each pump and mixer.
- [ ] Separate start request, start permissive, output command, running feedback, trip, available, and failed-to-start alarm.
- [ ] Use a start timeout after the command is issued.
- [ ] Use a stop timeout if stopped feedback is modelled.
- [ ] Latch trip or failed-start status where appropriate.
- [ ] Require a valid reset condition before clearing faults.
- [ ] Accumulate runtime only when running feedback is true.
- [ ] Increment start count on a valid stopped-to-running transition.

## 8.8 Valve logic

- [ ] Separate open request, close request, output command, open feedback, closed feedback, travel timer, and fault.
- [ ] Prevent conflicting open and close outputs.
- [ ] Raise failed-to-open if open feedback does not arrive within the timeout.
- [ ] Raise failed-to-close if closed feedback does not arrive within the timeout.
- [ ] Prevent associated pump start until required valve feedback is confirmed.
- [ ] Display the cause of a valve inhibit in Ignition.

## 8.9 Equipment permissives

| **Equipment** | **Minimum permissives**                                                                                                                                                  |
|---------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| P-101A/B      | Mode permits operation; no emergency stop; T-101 above low-low; T-201 below high-high; pump available; no trip; fill state active or permitted manual request.           |
| DP-201        | T-201 above minimum dosing level; mixer running; concentration transmitter healthy; concentration below high-high; dosing state active or permitted maintenance request. |
| M-201         | T-201 above minimum mixing level; mixer available; no trip; dosing or mixing state active.                                                                               |
| P-201         | Treatment batch accepted; outlet valve open feedback true; T-201 above low; T-301 below high-high; pump available.                                                       |
| P-301A/B      | T-301 above low-low; pressure transmitter healthy; distribution valve available; pump available; automatic pressure control or permitted manual request.                 |

## 8.10 Batch state machine

| **State**     | **Required entry actions**                                               | **Required transition**                                                              |
|---------------|--------------------------------------------------------------------------|--------------------------------------------------------------------------------------|
| 0 - Stopped   | All batch outputs off; timers reset as designed                          | Auto mode, start request, communication healthy, and global permissives true         |
| 10 - Ready    | Evaluate new-batch permissives                                           | T-101 available, T-201 ready, T-301 capacity available, required equipment available |
| 20 - Fill     | Select duty transfer pump and fill T-201                                 | T-201 reaches batch-fill setpoint                                                    |
| 30 - Dose     | Start mixer and concentration PI                                         | Concentration stable in tolerance for required time                                  |
| 40 - Mix      | Continue mixer for required mixing time                                  | Mix timer complete                                                                   |
| 50 - Verify   | Check concentration, sensor health, treatment time, alarms, and capacity | Pass to transfer; retry dosing; or fault after maximum retries                       |
| 60 - Transfer | Open outlet valve, verify feedback, run P-201                            | T-201 reaches low transfer-complete level                                            |
| 70 - Complete | Stop transfer, close valve, increment batch count, record success        | Return to Ready                                                                      |
| 900 - Faulted | Safe outputs; latch reason; prevent automatic continuation               | Fault condition cleared and operator reset accepted                                  |

## 8.11 Batch-sequence requirements

- [ ] Create an explicit state variable visible to Ignition.
- [ ] Use separate timers for fill timeout, dosing timeout, mix time, valve timeout, and transfer timeout.
- [ ] Check stop request and critical faults in every state.
- [ ] Define the safe destination state after a normal stop.
- [ ] Define the safe destination state after a critical fault.
- [ ] Record the first-out fault or reason for sequence failure.
- [ ] Prevent stale timers from carrying into a new state.
- [ ] Increment retry count only when quality verification fails.
- [ ] Reject the batch after the configured retry limit.
- [ ] Never transfer a batch unless quality verification has passed.

## 8.12 Pressure PI implementation

- [ ] Use the MATLAB-designed Kp, Ki, and sample time.
- [ ] Calculate pressure error from setpoint minus measured pressure.
- [ ] Update the integral term only at the selected PI execution interval.
- [ ] Apply output limits from 0% to the defined total pump-demand maximum.
- [ ] Implement anti-windup by conditional integration or back-calculation.
- [ ] Provide manual/automatic controller mode.
- [ ] Provide bumpless transfer from manual to automatic.
- [ ] Freeze or reset integral action when the pressure sensor is unhealthy.
- [ ] Expose process value, setpoint, error, proportional term, integral term, raw output, and limited output.

## 8.13 Booster-pump staging

- [ ] Interpret the PI output as total pumping demand.
- [ ] Assign the initial demand to the selected lead pump.
- [ ] Start the lag pump if lead demand remains above the staging threshold for the required delay.
- [ ] Share total demand between two running pumps using a documented rule.
- [ ] Stop the lag pump only after demand remains below the de-staging threshold for the required delay.
- [ ] Use hysteresis and timers to prevent rapid cycling.
- [ ] Immediately request standby takeover if the running lead pump trips or fails to start.
- [ ] Limit commands to available pumps.
- [ ] Raise insufficient-capacity alarm when demand exceeds the available pump capacity.

## 8.14 Lead-lag rotation

- [ ] Track runtime hours for each redundant pump.
- [ ] Track number of starts for each pump.
- [ ] Select the available pump with lower runtime as lead at the next normal start.
- [ ] Provide an optional forced-lead maintenance selection.
- [ ] Ignore unavailable or tripped pumps.
- [ ] Alternate normally after a completed operating cycle.
- [ ] Display lead, lag, forced, unavailable, and runtime states in Ignition.

## 8.15 Concentration PI implementation

- [ ] Use the MATLAB-designed Kp, Ki, and update interval.
- [ ] Enable the controller only in dosing or correction states.
- [ ] Force output to zero outside permitted states.
- [ ] Apply output limits and anti-windup.
- [ ] Stop dosing immediately on concentration high-high, sensor failure, low treatment level, mixer failure, or emergency stop.
- [ ] Provide manual output only in permitted maintenance conditions.
- [ ] Implement concentration-stable logic using tolerance and duration.
- [ ] Expose controller internals for tuning and troubleshooting.

## 8.16 Alarm logic

- [ ] Create active condition, latched state, acknowledged state if handled in PLC, reset condition, priority, and message identifier.
- [ ] Use on-delay timers for noisy or transient alarms where appropriate.
- [ ] Use immediate action for emergency stop, high-high concentration, low-low tank level, and communication failure.
- [ ] Record first-out cause for each equipment trip and sequence fault.
- [ ] Prevent alarm reset while the initiating condition remains active.
- [ ] Expose enough alarm detail for Ignition to display useful messages.

## 8.17 Runtime, batch, and diagnostic counters

- [ ] Accumulated runtime for each pump and mixer.
- [ ] Start count for each motor.
- [ ] Failed-start count.
- [ ] Trip count.
- [ ] Completed batch count.
- [ ] Rejected batch count.
- [ ] Treatment retry count.
- [ ] Last batch duration.
- [ ] Last failure code.
- [ ] Maximum observed batch concentration if practical.

## 8.18 OpenPLC standalone tests

- [ ] Test each equipment object with simulated input values before connecting Simulink.
- [ ] Test every permissive false condition.
- [ ] Test every trip condition.
- [ ] Test start timeout and failed-to-start alarm.
- [ ] Test alarm latching and reset.
- [ ] Step manually through every batch state using controlled test values.
- [ ] Test stop request from every state.
- [ ] Test critical fault from every state.
- [ ] Test PI saturation and anti-windup using simulated values.
- [ ] Test lead-lag selection with different runtime values and availability combinations.

## 8.19 OpenPLC completion gate

| **Gate**              | OpenPLC Gate P1                                                                                                                                                                                                                  |
|-----------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| **Purpose**           | Confirm the control program is logically complete before connecting the full Simulink plant and Ignition project.                                                                                                                |
| **Pass requirements** | All modes operate; equipment objects pass unit tests; every batch state and transition is tested; both PI blocks execute correctly; staging and rotation work; alarms latch and reset correctly; watchdog produces a safe state. |
| **Evidence to save**  | PLC source export, variable map, unit-test checklist, state-transition screenshots, PI diagnostic screenshots, and alarm-test evidence.                                                                                          |

# 9. Ignition Work Package

## 9.1 Ignition purpose

Ignition provides the operator-facing SCADA/HMI layer. It must display process conditions, show why equipment is inhibited, allow safe operator requests, manage alarm presentation, record history, and support troubleshooting. Ignition does not directly decide whether a physical command is safe.

## 9.2 Project setup

- [ ] Create a dedicated Ignition project and record the project name and version.
- [ ] Configure the connection to OpenPLC.
- [ ] Verify one read tag and one write request before importing the full tag map.
- [ ] Create tag folders that mirror the plant areas and PLC structure.
- [ ] Import or create tags from the master register map.
- [ ] Set engineering units, descriptions, scaling, and update rates.
- [ ] Configure tag quality display and communication status.
- [ ] Create a reusable project backup after every major screen group is completed.

## 9.3 Recommended tag folder structure

```text
WaterTreatment/

|

|-- Plant/

| |-- T101/

| |-- T201/

| |-- T301/

| |-- Distribution/

| `-- Utilities/

|

|-- Equipment/

| |-- P101A/

| |-- P101B/

| |-- P201/

| |-- P301A/

| |-- P301B/

| |-- DP201/

| `-- M201/

|

|-- Control/

| |-- Batch/

| |-- PressurePI/

| |-- ConcentrationPI/

| `-- Modes/

|

|-- Alarms/

|-- Commands/

|-- Diagnostics/

`-- Simulation/
```

## 9.4 HMI design principles

- [ ] Use consistent symbols, colors, units, and status labels.
- [ ] Do not rely on color alone; include text or icons for running, stopped, faulted, inhibited, and manual states.
- [ ] Show command and feedback separately.
- [ ] Show requested mode and active mode separately.
- [ ] Show process value and setpoint together.
- [ ] Show why an equipment start request is blocked.
- [ ] Require confirmation for consequential actions such as reset, mode change, fault injection, and batch rejection.
- [ ] Avoid decorative animation that does not communicate process state.
- [ ] Use readable trend scales and engineering units.

## 9.5 Required screen set

| **Screen**            | **Minimum required content**                                                                                                          |
|-----------------------|---------------------------------------------------------------------------------------------------------------------------------------|
| 1\. Plant Overview    | Three tanks, flows, valves, pumps, levels, pressure, concentration, current batch state, mode, alarm count, and communication health. |
| 2\. Treatment Detail  | T-201 level, concentration, setpoint, dosing output, mixer status, batch timer, retry count, quality status, and treatment trend.     |
| 3\. Pump Station      | Faceplates for P-101A/B and P-301A/B, lead/lag status, speed, flow, runtime, starts, permissives, and faults.                         |
| 4\. Pressure Control  | Pressure PV, SP, error, PI output, lead and lag speed, demand, controller mode, manual output, and trend.                             |
| 5\. Alarm Summary     | Active alarms, unacknowledged alarms, priority, source, message, timestamp, acknowledgement, and filter controls.                     |
| 6\. Historical Trends | Selectable level, flow, pressure, concentration, output, and state history.                                                           |
| 7\. Maintenance       | Runtime, start count, failed starts, trips, availability, out-of-service selection, and maintenance notes.                            |
| 8\. Simulation/Test   | Scenario selection, fault injection, reset, active fault status, and test notes.                                                      |
| 9\. Diagnostics       | Raw registers, scaled values, heartbeat, watchdog timer, tag quality, update timestamps, and command/feedback comparison.             |

## 9.6 Equipment faceplates

- [ ] Equipment name and description.
- [ ] Running, stopped, faulted, unavailable, inhibited, manual, lead, and lag indication.
- [ ] Start request and stop request buttons where permitted.
- [ ] Command output and actual feedback.
- [ ] Speed command and flow where applicable.
- [ ] Permissive summary.
- [ ] Detailed permissive list.
- [ ] Trip and alarm message.
- [ ] Runtime, start count, and last-fault code.
- [ ] Reset request with appropriate confirmation.

## 9.7 Operator command design

- [ ] Ignition writes request tags, not direct motor outputs.
- [ ] Start and stop buttons write one-shot or momentary request bits.
- [ ] Mode requests require clear indication of the current active mode.
- [ ] Setpoint changes are limited to documented engineering ranges.
- [ ] Invalid values are rejected or clamped before they reach operational logic.
- [ ] Reset requests do not clear active unsafe conditions.
- [ ] Fault-injection controls are separated from normal operator controls.

## 9.8 Alarm configuration

- [ ] Map each required alarm from the PLC or configure it against the appropriate tag condition.
- [ ] Assign critical, high, medium, or low priority.
- [ ] Use clear messages that name the equipment, condition, and expected operator response.
- [ ] Configure alarm acknowledgement.
- [ ] Configure alarm history or journal.
- [ ] Verify alarm activation, acknowledgement, clearing, and reactivation.
- [ ] Provide filters by priority, equipment, area, and active/cleared state.
- [ ] Display communication-quality alarms prominently.

## 9.9 Tag history and trends

- [ ] Record all tank levels.
- [ ] Record pressure PV, pressure SP, PI output, lead speed, and lag speed.
- [ ] Record concentration PV, concentration SP, and dosing output.
- [ ] Record important flow measurements.
- [ ] Record batch state and batch outcome.
- [ ] Record equipment running and fault states.
- [ ] Use an appropriate sample or deadband so trends are useful without excessive data.
- [ ] Confirm historical data survives project restarts as intended.

## 9.10 Ignition testing

- [ ] Every displayed value matches the PLC value and engineering units.
- [ ] Every writable tag changes the intended PLC request.
- [ ] No HMI command bypasses a PLC interlock.
- [ ] All faceplate statuses are tested.
- [ ] All alarm priorities and messages are tested.
- [ ] Alarm history contains the correct event sequence.
- [ ] Trends show the correct values and time alignment.
- [ ] Loss of OpenPLC communication produces visible bad quality or communication alarm.
- [ ] Project backup can be restored successfully.

## 9.11 Ignition completion gate

| **Gate**              | Ignition Gate I1                                                                                                                                                                                                                          |
|-----------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| **Purpose**           | Confirm the operator interface is complete and trustworthy before final multi-scenario validation.                                                                                                                                        |
| **Pass requirements** | All required screens exist; all tags have valid scaling and quality; operator requests reach the PLC; faceplates show command and feedback; alarm history works; trends record required signals; diagnostics expose communication status. |
| **Evidence to save**  | Ignition project backup, tag export, alarm export, screen screenshots, alarm-journal screenshots, and trend screenshots.                                                                                                                  |

# 10. Communications and Integration Work Package

## 10.1 Integration sequence

14. Prove OpenPLC runtime is running.

15. Prove one Ignition read tag from OpenPLC.

16. Prove one Ignition write request to OpenPLC.

17. Prove one Simulink measurement write to OpenPLC.

18. Prove one OpenPLC actuator command read by Simulink.

19. Close one simple level-based pump loop.

20. Add all measurements and commands for one process area.

21. Add the remaining process areas.

22. Enable watchdogs and safe-state logic.

23. Connect full Ignition screens only after the underlying tag behavior is stable.

## 10.2 Master register-map requirements

- [ ] Unique address for every external value.
- [ ] Defined owner for every writable value.
- [ ] Defined direction: Simulink to PLC, PLC to Simulink, Ignition to PLC, or PLC to Ignition.
- [ ] Defined data type and scaling.
- [ ] Defined engineering units.
- [ ] Defined valid range and safe default.
- [ ] Defined update period.
- [ ] Defined description and associated equipment.
- [ ] Defined test method.

## 10.3 Scaled-integer strategy

Use scaled 16-bit integers for the initial integrated version. Examples: 67.3% becomes 673, 1.24 mg/L becomes 124, and 400.0 kPa becomes 4000 if 0.1 kPa resolution is selected. Floating-point exchange can be added later only after the complete scaled-integer system is stable.

- [ ] Document the scale factor for every analog tag.
- [ ] Use consistent rounding.
- [ ] Clamp values before conversion.
- [ ] Test negative values if any are permitted.
- [ ] Test the maximum representable value.
- [ ] Verify address-offset conventions in each client with a single known register.

## 10.4 Timing and pacing

| **Layer**                        | **Initial update period**                               |
|----------------------------------|---------------------------------------------------------|
| Simulink continuous plant        | 0.01 to 0.05 s base step, selected after solver testing |
| Simulink communication interface | 0.10 s                                                  |
| OpenPLC scan                     | 20 to 100 ms                                            |
| Pressure PI                      | 0.10 to 0.25 s                                          |
| Concentration PI                 | 0.5 to 1.0 s                                            |
| Ignition process tags            | 0.25 to 1.0 s                                           |
| Historical logging               | 1 to 5 s or deadband-based                              |

## 10.5 Watchdog and safe-state design

- [ ] Simulink sends an incrementing heartbeat.
- [ ] OpenPLC declares plant communication failed after a configurable timeout.
- [ ] OpenPLC commands pumps, valves, mixer, and dosing to defined safe states.
- [ ] Simulink detects stale PLC commands and applies its own safe command defaults.
- [ ] Ignition displays PLC, plant, and HMI communication health separately where possible.
- [ ] Automatic restart is inhibited until communication is stable and an operator reset is completed.
- [ ] The safe state is tested during each important batch state.

## 10.6 Integration troubleshooting checklist

- [ ] Confirm IP address and port.
- [ ] Confirm OpenPLC server is listening.
- [ ] Confirm register type: holding register, input register, coil, or discrete input.
- [ ] Confirm zero-based versus one-based address interpretation.
- [ ] Confirm byte and word order if multi-register values are used.
- [ ] Confirm scaling and signed/unsigned interpretation.
- [ ] Confirm update periods are not excessively slow.
- [ ] Confirm no two applications are writing the same value.
- [ ] Confirm firewall rules permit local communication.
- [ ] Confirm stale values are not coming from cached HMI tags.
- [ ] Confirm Simulink runs near real time when external control timers are active.

## 10.7 Integration completion gate

| **Gate**              | Integration Gate C1                                                                                                                                                                                                                                       |
|-----------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| **Purpose**           | Confirm stable closed-loop operation across all four software environments.                                                                                                                                                                               |
| **Pass requirements** | All required tags exchange correctly; command and feedback directions are verified; watchdogs detect loss; safe states work; one complete automatic batch executes; pressure control operates through OpenPLC; Ignition displays and records the process. |
| **Evidence to save**  | Communication test sheet, register screenshots, closed-loop trend, normal-batch video or screenshots, watchdog test, and known-good backups of all four environments.                                                                                     |

# 11. Alarm, Interlock, and Permissive Design

## 11.1 Design documents to create

- [ ] Alarm list with tag, priority, trigger, delay, latching, reset, consequence, and message.
- [ ] Interlock matrix with equipment, initiating condition, commanded action, and reset requirement.
- [ ] Permissive matrix with equipment and all required start conditions.
- [ ] Cause-and-effect matrix linking abnormal conditions to PLC actions, alarms, sequence transitions, and HMI indication.
- [ ] First-out fault code list.

## 11.2 Critical conditions

| **Condition**                  | **Minimum required response**                                                                 |
|--------------------------------|-----------------------------------------------------------------------------------------------|
| Emergency stop                 | Immediate process outputs off; automatic sequence faulted or stopped; critical alarm latched. |
| T-101 low-low                  | Stop P-101A/B; prevent restart; low-low alarm.                                                |
| T-201 high-high                | Stop fill pumps; prevent additional fill; high-high alarm.                                    |
| T-301 low-low                  | Stop booster pumps; prevent dry running; distribution low-level alarm.                        |
| T-301 high-high                | Stop P-201 transfer; prevent additional accepted-water transfer.                              |
| Concentration high-high        | Stop dosing immediately; reject or fault batch; critical alarm.                               |
| Pressure transmitter failure   | Disable automatic pressure PI; move pumps to safe defined behavior; alarm.                    |
| Both booster pumps unavailable | Declare insufficient distribution capacity; critical alarm.                                   |
| Plant communication failure    | Safe output state; sequence fault; communication alarm.                                       |
| Valve failed to open           | Prevent associated pump start; fault sequence; alarm.                                         |

## 11.3 Alarm message standard

Use messages that state the equipment, condition, and consequence.

| **Weak message** | **Preferred message**                                                       |
|------------------|-----------------------------------------------------------------------------|
| Pump fault       | P-301A failed to start; standby takeover requested.                         |
| High level       | T-201 high-high level; raw-water transfer stopped.                          |
| Sensor bad       | AIT-201 concentration signal invalid; dosing disabled and batch faulted.    |
| Comms lost       | Simulink plant heartbeat lost; all process outputs commanded to safe state. |

## 11.4 Reset philosophy

- [ ] Do not reset while the initiating condition remains active.
- [ ] Use an operator reset for critical trips and sequence faults.
- [ ] Allow non-latched warnings to clear automatically when the condition clears.
- [ ] Preserve first-out cause until reset.
- [ ] Do not automatically restart the batch after a critical fault.
- [ ] Require communication healthy and operating permissives true before reset is accepted.

# 12. Testing and Validation Work Package

## 12.1 Testing levels

| **Level**           | **Purpose**                                                                                        |
|---------------------|----------------------------------------------------------------------------------------------------|
| Unit testing        | Test individual tank, pump, valve, sensor, function block, alarm, and HMI component.               |
| Subsystem testing   | Test raw-water transfer, treatment sequence, distribution pressure, and communications separately. |
| Integration testing | Test data exchange and command flow among Simulink, OpenPLC, and Ignition.                         |
| Scenario validation | Test the complete system against defined normal and fault scenarios.                               |
| Regression testing  | Re-run all affected tests after changes.                                                           |

## 12.2 Test procedure format

- [ ] Test identifier and title.
- [ ] Objective.
- [ ] Initial conditions.
- [ ] Software versions and project revision.
- [ ] Required setup.
- [ ] Input or disturbance.
- [ ] Expected PLC response.
- [ ] Expected plant response.
- [ ] Expected Ignition indication.
- [ ] Quantitative acceptance criteria.
- [ ] Observed result.
- [ ] Pass/fail.
- [ ] Evidence filename.
- [ ] Notes and corrective action.

## 12.3 Required validation scenarios

| **ID** | **Scenario**                 | **Main acceptance criteria**                                                             |
|--------|------------------------------|------------------------------------------------------------------------------------------|
| T01    | Normal automatic batch       | Batch completes; concentration accepted; no critical alarms; batch count increments.     |
| T02    | Distribution-demand increase | Pressure recovers within allowed time and error; staging occurs if required.             |
| T03    | Lead booster-pump failure    | Failure alarm; standby starts; pressure recovers; failed pump excluded.                  |
| T04    | Raw-water low-low            | Transfer pumps stop before dry running; sequence pauses or faults safely.                |
| T05    | Overconcentration            | Dosing stops immediately; batch cannot transfer; critical alarm.                         |
| T06    | Valve stuck closed           | P-201 does not start; valve timeout alarm; batch remains contained.                      |
| T07    | Concentration sensor frozen  | PLC detects implausible response; dosing stops; batch faults.                            |
| T08    | Communication loss           | Watchdog expires; outputs safe; restart inhibited; alarm visible.                        |
| T09    | Failed treatment batch       | Retry count increments; batch rejected after maximum retries.                            |
| T10    | Manual operation             | Permitted commands work; unsafe requests are rejected.                                   |
| T11    | Lag-pump de-staging          | Lag pump stops only after sustained low demand without pressure instability.             |
| T12    | Low clean-water level        | Booster pumps stop and cannot restart until level recovers and reset conditions are met. |

## 12.4 Suggested numerical acceptance criteria

| **Metric**                          | **Initial target**                                          |
|-------------------------------------|-------------------------------------------------------------|
| Normal pressure steady-state error  | Within +/-10 kPa                                            |
| Pressure after major demand step    | Return within +/-10 kPa of setpoint within 15 s             |
| Pressure overshoot                  | Less than 15% of setpoint change or project-defined limit   |
| Minimum pressure during demand step | At or above 320 kPa                                         |
| Standby takeover command delay      | Less than 2 s after failure detection                       |
| Concentration acceptance            | 1.10 to 1.30 mg/L for at least 20 s                         |
| Concentration high-high response    | Dosing command reaches 0% within one PLC execution interval |
| Valve failed-to-open detection      | Within configured valve travel timeout plus one scan        |
| Communication loss safe state       | Within watchdog timeout plus one scan                       |
| Rejected batch transfer             | Zero treated-transfer command while batch is unaccepted     |

## 12.5 Evidence required for every scenario

- [ ] MATLAB summary text or results row.
- [ ] Plot of the main process variables.
- [ ] PLC state and alarm evidence.
- [ ] Ignition screenshot of the event or trend.
- [ ] Pass/fail result.
- [ ] Project revision and timestamp.
- [ ] Notes on any deviation.

## 12.6 Regression-test rule

> **Regression requirement**
>
> Any change to parameters, PLC logic, register mapping, communication timing, alarm logic, or HMI commands requires re-running every scenario that could be affected. The final validation summary must be generated from the final project revision, not from mixed earlier versions.

## 12.7 Final validation gate

| **Gate**              | Validation Gate V1                                                                                                                                                                                      |
|-----------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| **Purpose**           | Confirm the completed system meets the defined functional and performance requirements.                                                                                                                 |
| **Pass requirements** | All mandatory scenarios have saved evidence; no critical scenario fails; quantitative response targets are met or deviations are explicitly justified; final project backups match the tested revision. |
| **Evidence to save**  | Final validation summary, plots, alarm history, PLC exports, Ignition backup, Simulink model, MATLAB results folder, and signed completion checklist.                                                   |

# 13. Project Stages and Gates

## Stage 0 - Environment and planning
- [ ] Install and verify MATLAB/Simulink, OpenPLC, and Ignition.
- [ ] Create the folder structure and master source-of-truth documents.
- [ ] Freeze the first-version project scope and equipment list.
- [ ] Define naming, units, scaling, and backup procedures.

**Gate G0: All software opens, project folders exist, and scope is frozen.**

## Stage 1 - MATLAB foundation
- [ ] Create parameter initialization.
- [ ] Create register-map generator.
- [ ] Derive tank, pressure, and concentration models.
- [ ] Create scenario structures and result framework.

**Gate G1: MATLAB Gate M1 passes.**

## Stage 2 - Standalone Simulink plant
- [ ] Build tanks, pumps, valves, pressure, concentration, sensors, and faults.
- [ ] Validate mass balance and physical responses.
- [ ] Create temporary controller-test models.

**Gate G2: Simulink Gate S1 passes.**

## Stage 3 - OpenPLC core logic
- [ ] Build I/O scaling, modes, equipment objects, alarms, and watchdog.
- [ ] Build and unit-test the batch state machine.
- [ ] Build and unit-test PI blocks, staging, and lead-lag rotation.

**Gate G3: OpenPLC Gate P1 passes.**

## Stage 4 - Basic plant/PLC integration
- [ ] Prove one measurement and one command.
- [ ] Close one simple loop.
- [ ] Expand mapping area by area.
- [ ] Test scaling and safe defaults.

**Gate G4: Stable command/feedback exchange with watchdog.**

## Stage 5 - Full control integration
- [ ] Run a complete automatic batch.
- [ ] Run pressure PI through OpenPLC.
- [ ] Run concentration PI through OpenPLC.
- [ ] Verify staging, alternation, and standby takeover.

**Gate G5: Integration Gate C1 passes.**

## Stage 6 - Ignition SCADA
- [ ] Create tags, overview, faceplates, control screens, alarms, history, and diagnostics.
- [ ] Verify every operator command and indication.

**Gate G6: Ignition Gate I1 passes.**

## Stage 7 - Fault handling
- [ ] Implement and verify all required plant, instrument, equipment, and communication faults.
- [ ] Verify first-out cause, safe state, alarms, and reset behavior.

**Gate G7: All abnormal operations have deterministic safe responses.**

## Stage 8 - Automated validation
- [ ] Run all MATLAB scenarios.
- [ ] Evaluate acceptance criteria.
- [ ] Correct failures and repeat regression testing.

**Gate G8: Validation Gate V1 passes.**

## Stage 9 - Final documentation and portfolio package
- [ ] Create final architecture, control narrative, FDS, alarm/interlock matrix, plots, screenshots, and project summary.
- [ ] Clean the project folder and preserve known-good backups.

**Gate G9: Definition of Done satisfied.**

# 14. Final Deliverables and Portfolio Presentation

## 14.1 Required source deliverables

- [ ] MATLAB scripts and scenario files.
- [ ] Final integrated Simulink plant model.
- [ ] Temporary controller-validation models.
- [ ] OpenPLC source and exported project.
- [ ] Ignition project backup and tag exports.
- [ ] Master register map.
- [ ] Alarm, interlock, permissive, and cause-and-effect matrices.
- [ ] Final validation results and plots.

## 14.2 Required documentation

| **Document**                    | **Minimum content**                                                                         |
|---------------------------------|---------------------------------------------------------------------------------------------|
| System Architecture             | Software responsibilities, communication paths, timing, and data ownership.                 |
| Functional Design Specification | Process description, equipment, modes, sequence, controls, alarms, and interfaces.          |
| Control Narrative               | Detailed state-by-state and equipment-by-equipment behavior.                                |
| Register and Tag Map            | All addresses, scaling, units, ownership, and ranges.                                       |
| Alarm and Interlock Matrix      | Conditions, actions, priorities, latching, reset, and messages.                             |
| Test Plan                       | Procedures, initial conditions, expected results, and acceptance criteria.                  |
| Validation Report               | Results, plots, pass/fail summary, deviations, and final conclusion.                        |
| User Guide                      | How to start the software, load the project, run the plant, use the HMI, and execute tests. |

## 14.3 Required visual evidence

- [ ] Simulink top-level plant model.
- [ ] Representative tank, pump, sensor, and fault subsystems.
- [ ] OpenPLC main program organization.
- [ ] Batch state-machine logic.
- [ ] Pressure PI and concentration PI diagnostics.
- [ ] Ignition plant overview.
- [ ] Pump faceplate.
- [ ] Pressure-control trend.
- [ ] Treatment-concentration trend.
- [ ] Alarm summary and alarm history.
- [ ] Communication diagnostics.
- [ ] MATLAB scenario summary and plots.

## 14.4 Demonstration sequence

24. Show the architecture and explain each software boundary.

25. Start the plant, OpenPLC runtime, and Ignition project.

26. Run a normal automatic batch.

27. Show concentration control and quality verification.

28. Apply a distribution-demand increase and show pressure recovery and pump staging.

29. Trip the lead booster pump and show standby takeover.

30. Inject a sensor or valve fault and show the alarm, safe response, and reset process.

31. Show the MATLAB validation summary and final performance metrics.

## 14.5 Suggested final report structure

32. Executive summary.

33. Project objectives and scope.

34. System architecture.

35. Physical process model.

36. Control-system design.

37. PLC implementation.

38. SCADA implementation.

39. Communications and timing.

40. Safety, alarms, and fault handling.

41. Controller tuning.

42. Validation method.

43. Results.

44. Limitations.

45. Future improvements.

46. Conclusion.

# 15. Final Master Checklist

## 15.1 Planning and organization

- [ ] Scope is frozen and excludes Revit, Python, and physical PLC hardware.
- [ ] Folder structure exists.
- [ ] Naming convention is documented.
- [ ] Equipment and instrument lists are complete.
- [ ] Master register map exists.
- [ ] Change log is maintained.

## 15.2 MATLAB

- [ ] Initialization script runs from a clean workspace.
- [ ] Register map exports without duplicates.
- [ ] Pressure model and PI design are documented.
- [ ] Concentration model and PI design are documented.
- [ ] Scenario configurations are complete.
- [ ] Automated result analysis is complete.
- [ ] Final plots and summary tables generate automatically.

## 15.3 Simulink

- [ ] All three tanks are complete.
- [ ] All pumps and valves are complete.
- [ ] Pressure model is complete.
- [ ] Concentration model is complete.
- [ ] Sensor models are complete.
- [ ] Fault injection is complete.
- [ ] Communication interface is complete.
- [ ] Standalone physical validation passes.

## 15.4 OpenPLC

- [ ] I/O scaling is complete.
- [ ] Watchdog is complete.
- [ ] Modes are complete.
- [ ] Equipment objects are complete.
- [ ] Permissives and interlocks are complete.
- [ ] Batch sequence is complete.
- [ ] Pressure PI is complete.
- [ ] Concentration PI is complete.
- [ ] Pump staging and rotation are complete.
- [ ] Alarm and reset logic are complete.
- [ ] Runtime and batch counters are complete.

## 15.5 Ignition

- [ ] OpenPLC connection is stable.
- [ ] Tag structure is complete.
- [ ] Plant overview is complete.
- [ ] Treatment screen is complete.
- [ ] Pump screen and faceplates are complete.
- [ ] Pressure-control screen is complete.
- [ ] Alarm screen and journal are complete.
- [ ] Historical trends are complete.
- [ ] Maintenance screen is complete.
- [ ] Simulation/test screen is complete.
- [ ] Diagnostics screen is complete.
- [ ] Project backup is verified.

## 15.6 Integration

- [ ] All analog scaling is verified.
- [ ] All digital commands and feedback are verified.
- [ ] One complete automatic batch runs.
- [ ] Pressure PI operates through OpenPLC.
- [ ] Concentration PI operates through OpenPLC.
- [ ] Watchdog safe state is verified.
- [ ] Ignition commands do not bypass PLC logic.
- [ ] Known-good backups exist.

## 15.7 Validation and delivery

- [ ] All required scenarios have procedures.
- [ ] All required scenarios have saved evidence.
- [ ] All critical scenarios pass.
- [ ] Regression testing is complete.
- [ ] Final validation summary is generated.
- [ ] Final report is complete.
- [ ] Portfolio screenshots and project description are complete.
- [ ] Project can be restored and demonstrated from the saved files.

# Appendix A. Suggested Tag and Register Map

| **Address** | **Tag**                 | **Direction**    | **Scaling** | **Description**         |
|-------------|-------------------------|------------------|-------------|-------------------------|
| HR001       | LIT101_PV               | Simulink -\> PLC | % x10       | Raw-water level         |
| HR002       | LIT201_PV               | Simulink -\> PLC | % x10       | Treatment-tank level    |
| HR003       | LIT301_PV               | Simulink -\> PLC | % x10       | Clean-water level       |
| HR004       | FIT101_PV               | Simulink -\> PLC | L/s x10     | Raw transfer flow       |
| HR005       | FIT201_PV               | Simulink -\> PLC | L/s x10     | Treatment transfer flow |
| HR006       | FIT301_PV               | Simulink -\> PLC | L/s x10     | Distribution demand     |
| HR007       | AIT201_PV               | Simulink -\> PLC | mg/L x100   | Treatment concentration |
| HR008       | PIT301_PV               | Simulink -\> PLC | kPa x10     | Distribution pressure   |
| HR009       | Plant_Heartbeat         | Simulink -\> PLC | count       | Incrementing heartbeat  |
| HR101       | P101A_SpeedCmd          | PLC -\> Simulink | % x10       | Pump speed command      |
| HR102       | P101B_SpeedCmd          | PLC -\> Simulink | % x10       | Pump speed command      |
| HR103       | P201_SpeedCmd           | PLC -\> Simulink | % x10       | Pump speed command      |
| HR104       | P301A_SpeedCmd          | PLC -\> Simulink | % x10       | Pump speed command      |
| HR105       | P301B_SpeedCmd          | PLC -\> Simulink | % x10       | Pump speed command      |
| HR106       | DP201_Output            | PLC -\> Simulink | % x10       | Dosing output           |
| HR201       | Pressure_SP             | Ignition -\> PLC | kPa x10     | Pressure setpoint       |
| HR202       | Concentration_SP        | Ignition -\> PLC | mg/L x100   | Concentration setpoint  |
| HR203       | BatchFill_SP            | Ignition -\> PLC | % x10       | Treatment fill target   |
| HR204       | MixTime_SP              | Ignition -\> PLC | s           | Mixing time             |
| HR301       | Batch_State             | PLC -\> Ignition | integer     | Active sequence state   |
| HR302       | Batch_Count             | PLC -\> Ignition | integer     | Completed batches       |
| HR303       | Rejected_Batch_Count    | PLC -\> Ignition | integer     | Rejected batches        |
| HR304       | Pressure_PI_Output      | PLC -\> Ignition | % x10       | Total booster demand    |
| HR305       | Concentration_PI_Output | PLC -\> Ignition | % x10       | Dosing output           |

| **Address** | **Tag**                 | **Direction**                | **Description**             |
|-------------|-------------------------|------------------------------|-----------------------------|
| C001        | P101A_RunFb             | Simulink -\> PLC             | Pump running feedback       |
| C002        | P101B_RunFb             | Simulink -\> PLC             | Pump running feedback       |
| C003        | P201_RunFb              | Simulink -\> PLC             | Pump running feedback       |
| C004        | P301A_RunFb             | Simulink -\> PLC             | Pump running feedback       |
| C005        | P301B_RunFb             | Simulink -\> PLC             | Pump running feedback       |
| C006        | M201_RunFb              | Simulink -\> PLC             | Mixer running feedback      |
| C007        | XV201_OpenFb            | Simulink -\> PLC             | Valve open feedback         |
| C008        | XV201_ClosedFb          | Simulink -\> PLC             | Valve closed feedback       |
| C051        | P101A_StartCmd          | PLC -\> Simulink             | Pump start command          |
| C052        | P101B_StartCmd          | PLC -\> Simulink             | Pump start command          |
| C053        | P201_StartCmd           | PLC -\> Simulink             | Pump start command          |
| C054        | P301A_StartCmd          | PLC -\> Simulink             | Pump start command          |
| C055        | P301B_StartCmd          | PLC -\> Simulink             | Pump start command          |
| C056        | M201_StartCmd           | PLC -\> Simulink             | Mixer start command         |
| C057        | XV201_OpenCmd           | PLC -\> Simulink             | Valve open command          |
| C101        | System_StartReq         | Ignition -\> PLC             | Automatic start request     |
| C102        | System_StopReq          | Ignition -\> PLC             | Stop request                |
| C103        | System_ResetReq         | Ignition -\> PLC             | Reset request               |
| C104        | Auto_ModeReq            | Ignition -\> PLC             | Automatic-mode request      |
| C105        | Manual_ModeReq          | Ignition -\> PLC             | Manual-mode request         |
| C251        | Fault_P301A_Trip        | Ignition/MATLAB -\> Simulink | Lead booster fault          |
| C252        | Fault_AIT201_Freeze     | Ignition/MATLAB -\> Simulink | Concentration sensor freeze |
| C253        | Fault_XV201_StuckClosed | Ignition/MATLAB -\> Simulink | Valve stuck closed          |
| C254        | Fault_CommsLoss         | Test control                 | Communication interruption  |

> **Register-map warning**
>
> The addresses above are a suggested logical layout, not a guaranteed software-specific address convention. Verify zero-based versus one-based interpretation and register type in each installed application before configuring the complete map.

# Appendix B. Suggested Alarm List

| **ID** | **Alarm**                       | **Priority** | **Required action**                            |
|--------|---------------------------------|--------------|------------------------------------------------|
| A001   | Emergency stop active           | Critical     | Global outputs safe; sequence stopped/faulted  |
| A002   | Plant communication lost        | Critical     | Outputs safe; automatic restart inhibited      |
| A010   | T-101 low-low level             | Critical     | Stop and inhibit P-101A/B                      |
| A011   | T-201 high-high level           | Critical     | Stop raw-water filling                         |
| A012   | T-301 low-low level             | Critical     | Stop and inhibit P-301A/B                      |
| A013   | T-301 high-high level           | High         | Stop P-201 transfer                            |
| A020   | AIT-201 concentration high-high | Critical     | Stop dosing; reject/fault batch                |
| A021   | AIT-201 signal invalid          | High         | Disable dosing; fault batch                    |
| A022   | Concentration out of range      | High         | Retry or reject batch                          |
| A030   | P-101A failed to start          | High         | Remove from service; request standby           |
| A031   | P-101B failed to start          | High         | Remove from service; request standby           |
| A032   | P-201 failed to start           | High         | Stop transfer and fault sequence               |
| A033   | P-301A failed to start          | High         | Remove from service; request standby           |
| A034   | P-301B failed to start          | High         | Remove from service; request standby           |
| A035   | Both booster pumps unavailable  | Critical     | Insufficient distribution capacity             |
| A040   | XV-201 failed to open           | High         | Prevent P-201 start; fault sequence            |
| A041   | XV-201 failed to close          | High         | Isolate sequence; operator action              |
| A050   | Distribution pressure low       | High         | Increase demand output; stage lag if available |
| A051   | Distribution pressure high      | High         | Reduce output; alarm                           |
| A052   | Pressure transmitter invalid    | Critical     | Disable automatic PI; safe response            |
| A060   | Batch fill timeout              | High         | Stop fill; sequence fault                      |
| A061   | Dosing timeout                  | High         | Stop dosing; retry or reject                   |
| A062   | Transfer timeout                | High         | Stop P-201; sequence fault                     |
| A070   | Manual mode active              | Low          | Operator awareness                             |
| A071   | Equipment forced out of service | Medium       | Reduced redundancy                             |

# Appendix C. Suggested Test Matrix

| **ID** | **Test**               | **Input**                    | **Expected result**               | **Primary evidence**         |
|--------|------------------------|------------------------------|-----------------------------------|------------------------------|
| T01    | Normal automatic batch | Normal levels and equipment  | One complete accepted batch       | MATLAB plot + Ignition trend |
| T02    | Demand step            | 30 to 60 L/s                 | Pressure recovery and staging     | Pressure/staging plot        |
| T03    | Lead booster failure   | Trip P-301A while running    | Standby takeover                  | Alarm + takeover trend       |
| T04    | Raw low-low            | Reduce T-101 inflow          | P-101 stop and inhibit            | Level and command plot       |
| T05    | Overconcentration      | Inject excessive dose effect | Dose off and batch rejected       | Concentration plot + alarm   |
| T06    | Valve stuck closed     | Hold XV-201 feedback closed  | P-201 blocked and timeout         | PLC state + alarm            |
| T07    | Sensor freeze          | Freeze AIT-201               | Plausibility fault and safe state | Measured/actual plot         |
| T08    | Communication loss     | Stop heartbeat               | Safe state within timeout         | Watchdog plot                |
| T09    | Failed batch           | Weak dosing effectiveness    | Retries then rejection            | State timeline               |
| T10    | Manual interlock       | Request unsafe pump start    | Request rejected                  | Permissive screen            |
| T11    | Lag de-stage           | Sustained demand reduction   | Lag stops without cycling         | Pressure and pump trend      |
| T12    | Clean tank low-low     | Drain T-301                  | Boosters stop                     | Level and pump plot          |

# Appendix D. Definition of Done

> **The project is done only when**
>
> A clean copy of the final project can be opened, started, and demonstrated; the normal automatic sequence completes; both PI loops meet their defined criteria; required faults cause deterministic safe responses; alarms and history are correct; all mandatory scenarios pass; final source files and backups match the tested revision; and the documentation is sufficient for another engineering student to understand the architecture, run the project, and reproduce the validation.

## D.1 Final sign-off checklist

- [ ] Final MATLAB revision saved.
- [ ] Final Simulink revision saved.
- [ ] Final OpenPLC revision exported.
- [ ] Final Ignition backup verified.
- [ ] Master register map matches all applications.
- [ ] All mandatory test results are from the final revision.
- [ ] No unresolved critical alarms or known unsafe logic remain.
- [ ] Final report and user guide are complete.
- [ ] Project demonstration sequence has been rehearsed.
- [ ] Archive copy created.
