# Revision 0.2

- Replaced hand-derived-only PI recommendations with Control System Toolbox `pidtune` designs.
- Added continuous and discrete closed-loop stability and robustness checks.
- Added generated pressure and concentration controller-validation Simulink models.
- Added built-in PI output saturation and clamping anti-windup in validation models.
- Expanded standalone plant tests with sensor-lag verification.
- Added PLC-ready controller parameter export.
- Retained the architectural boundary: the integrated Simulink plant remains open loop for future OpenPLC control.


## Revision 0.3 — OpenPLC integration
- Added OpenPLC v4 Structured Text controller.
- Added Docker Runtime v4 and Modbus slave configuration.
- Added MATLAB Modbus smoke and acceptance tests.
- Added Simulink and Ignition Modbus connection guides.
- Added expanded OpenPLC register/tag map.

## Revision 0.4.1 — MATLAB R2026a integration-builder fix
- Declared `getSimulateUsingImpl` as a static protected System object method.
- Explicitly disconnects old standalone command lines before wiring PLC commands.
- No OpenPLC program, register-map, or Docker configuration changes are required.

## Revision 0.4.2 — MATLAB R2026a read-only mask fix
- Removed the builder's direct assignment to the MATLAB System block's
  read-only `SimulateUsing` mask parameter.
- Retained `OpenPLCModbusBridge.getSimulateUsingImpl`, so the block continues
  to use interpreted execution without a mask write.
- Retained the generated-model validation check for interpreted execution.
- No OpenPLC program, register-map, or Docker configuration changes are required.

## Revision 0.4.3 — live-loop client ownership and diagnostics
- The live demo no longer keeps a second MATLAB Modbus client open while the
  Simulink bridge is running.
- The bridge performs the demo reset, automatic-mode, and start pulses through
  its own connection; normal HMI ownership is unchanged outside the demo.
- Added a direct bridge preflight that reports the actual Modbus exception
  before starting the 30-second paced simulation.
- Added measurement and physical-feedback logs, and the demo now evaluates the
  closed loop from captured Simulink signals after the bridge releases its
  client.
- Added persistent last-error diagnostics for communication failures that were
  previously reduced to safe zero outputs.
- No OpenPLC program, register-map, or Docker configuration changes are required.

## Revision 0.4.4 — MATLAB R2026a discrete-state specification fix
- Added `getDiscreteStateSpecificationImpl` to define the size, data type, and
  complexity of all five `OpenPLCModbusBridge` discrete states.
- Expanded the generated-model validator so the build now checks every bridge
  discrete-state specification before the live Simulink run.
- No OpenPLC program, register-map, or Docker configuration changes are required.

## Revision 0.4.5 — MATLAB R2026a output-shape specification fix
- Changed all four `OpenPLCModbusBridge` output-size declarations from scalar
  lengths to explicit row-vector sizes: `[1 6]`, `[1 7]`, `[1 4]`, and `[1 1]`.
- Added a shared output-size declaration and a generated-model validation check
  so the sizes declared to Simulink stay identical to the values returned by
  `stepImpl`.
- No OpenPLC program, register-map, or Docker configuration changes are required.

## Revision 0.5.0 — Stage 5 full control integration
- Added a 260-second live commissioning scenario that runs one complete
  automatic batch through the real Simulink plant and OpenPLC controller.
- Added quantitative checks for pressure PI execution, concentration PI
  execution, high-demand two-pump staging, lead alternation, and P-301B
  physical takeover after a P-301A trip.
- Added HR301-HR317 and C151-C181 diagnostic logging from the MATLAB System
  bridge into the generated integrated Simulink model.
- Added automatic Stage 5 CSV, MAT-file, and PNG trend evidence under
  `06_TESTING`.
- Uses a commissioning-only T-201/DP-201 0.10 scale factor to preserve the
  concentration-process gain while reducing the live batch from roughly
  40 minutes to about 4.5 minutes.
- Added explicit safe cleanup of request and fault-injection coils after the
  test or after a handled failure.
- Retuned the final OpenPLC concentration PI from the reduced-order candidate
  (`Kp=85.6`, `Ki*Ts=3.88`) to the nonlinear-validated Stage 5 values
  (`Kp=100.0`, `Ki*Ts=0.10`). Rebuild and upload the v0.5.0 PLC program before
  running Stage 5. The pressure gains, register map, and Docker configuration
  are unchanged.
- Stage 5 closes the controller/plant portion of Integration Gate C1. The
  Ignition display/history requirement remains for Stage 6.
## Revision 0.5.1 — OpenPLC Runtime v4 CASE-label compatibility

- Replaced the named `STATE_*` identifiers only in the `CASE Batch_State OF` branch labels with their unchanged numeric values (`0`, `10`, `20`, `30`, `40`, `50`, `60`, `70`, and `900`).
- This prevents Runtime v4's STruC++ generator from emitting non-constant `IECVar<UINT>` expressions as C++ `case` labels.
- Kept the named state constants for assignments and comparisons, so the state machine behavior and Modbus-visible state values are unchanged.
- Synchronized `MAIN_Body.st` and `Water_Treatment_Main_FULL_REFERENCE.st`.

## Revision 0.5.2 — XV-201 supervision timing correction

- Increased the XV-201 failed-to-open supervision window from 50 scans
  (5.0 seconds) to 80 scans (8.0 seconds).
- The simulated valve has a 2.0-second first-order time constant and asserts
  open feedback at 95% position, which requires approximately 6.0 seconds.
  The former 5.0-second window therefore rejected a healthy batch immediately
  after entry to the transfer state.
- Added Stage 5 result details for the PLC first-out code, both alarm words,
  and the XV-201 failed-open diagnostic.
- No PI gain, Modbus address, plant model, or normal sequence-state value was
  changed.
