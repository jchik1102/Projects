# Stage 5 Full Control Integration Test

## Scope

This test validates the complete controller/plant portion of Integration Gate C1:

- one complete automatic treatment batch;
- pressure PI execution in OpenPLC;
- concentration PI execution in OpenPLC;
- high-demand booster staging;
- lead-booster alternation after a completed batch; and
- physical P-301B takeover after a P-301A trip.

Ignition display and history are intentionally excluded until Stage 6.

`05_INTEGRATION/Controller_Parameters.csv` retains the Stage 2 reduced-order
design candidates. The final PLC implementation values for Stage 5 are in
`05_INTEGRATION/OpenPLC_Stage5_Controller_Gains.csv`.

## Preconditions

1. Docker Desktop is running.
2. The OpenPLC Runtime container is running on Modbus TCP port `5020`.
3. The v0.5.2 water-treatment PLC source is rebuilt, uploaded, and running.
   Confirm `CONCENTRATION_KP = 100.0` and `CONCENTRATION_KI_TS = 0.10`.
4. MATLAB's Current Folder is the extracted project root.
5. No smoke test, PLC-only acceptance test, Modbus Explorer session, or other MATLAB Modbus client is active.

## Run

```matlab
if bdIsLoaded('Water_Treatment_Plant_OpenPLC_v0_4')
    close_system('Water_Treatment_Plant_OpenPLC_v0_4', 0);
end

clear classes
clear functions

RUN_STAGE5_FULL_CONTROL
```

The live test is paced at 1x and takes approximately 260 seconds. The test uses the same plant equations and controller gains as the nominal design. For commissioning only, T-201 capacity and DP-201 dosing capacity are both scaled to 10%, preserving the concentration-process gain while shortening the test.

## Automated scenario

| Time | Action |
|---:|---|
| 0-3 s | Establish heartbeat, write Stage 5 setpoints, reset, select automatic mode, and start |
| 15 s | Raise distribution demand from 30 L/s to 60 L/s |
| 25-44 s | Verify both booster commands are staged |
| 45 s | Return demand to 30 L/s and verify pressure recovery |
| 60-75 s | Inject P-301A trip and verify P-301B command plus physical running feedback |
| Whole run | Fill, dose, mix, verify, transfer, and complete one treatment batch |

## Pass evidence

The script prints seven checks and writes:

- `06_TESTING/Test_Results/Stage5_Full_Control_Integration.csv`
- `06_TESTING/MAT_Files/Stage5_Full_Control_Integration.mat`
- `06_TESTING/Plots/Stage5_Full_Control_Integration.png`

The final console output must state:

```text
STAGE 5 - FULL CONTROL INTEGRATION: PASSED
Controller/plant portion of Integration Gate C1: PASSED
Ignition display/history portion remains for Stage 6.
```

## Safety and cleanup

The script clears C251-C254, clears C101-C106, and sends a stop request after the run or after a handled test failure. The bridge never generates the Stage 5 requests unless the private MATLAB test flag is active.
