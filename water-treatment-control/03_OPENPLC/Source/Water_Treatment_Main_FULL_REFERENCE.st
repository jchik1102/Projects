PROGRAM main
VAR
    LIT101_PV AT %QW0 : UINT;
    LIT201_PV AT %QW1 : UINT;
    LIT301_PV AT %QW2 : UINT;
    FIT101_PV AT %QW3 : UINT;
    FIT201_PV AT %QW4 : UINT;
    FIT301_PV AT %QW5 : UINT;
    AIT201_PV AT %QW6 : UINT;
    PIT301_PV AT %QW7 : UINT;
    Plant_Heartbeat AT %QW8 : UINT;
    P101A_SpeedCmd AT %QW100 : UINT;
    P101B_SpeedCmd AT %QW101 : UINT;
    P201_SpeedCmd AT %QW102 : UINT;
    P301A_SpeedCmd AT %QW103 : UINT;
    P301B_SpeedCmd AT %QW104 : UINT;
    DP201_Output AT %QW105 : UINT;
    Pressure_SP AT %QW200 : UINT;
    Concentration_SP AT %QW201 : UINT;
    BatchFill_SP AT %QW202 : UINT;
    MixTime_SP AT %QW203 : UINT;
    Manual_Dose_Output AT %QW204 : UINT;
    Batch_State AT %QW300 : UINT;
    Batch_Count AT %QW301 : UINT;
    Rejected_Batch_Count AT %QW302 : UINT;
    Pressure_PI_Output AT %QW303 : UINT;
    Concentration_PI_Output AT %QW304 : UINT;
    Active_Mode AT %QW305 : UINT;
    Requested_Mode AT %QW306 : UINT;
    Lead_Booster AT %QW307 : UINT;
    FirstOut_Code AT %QW308 : UINT;
    Alarm_Word_1 AT %QW309 : UINT;
    Alarm_Word_2 AT %QW310 : UINT;
    P101A_Runtime_s AT %QW311 : UINT;
    P101B_Runtime_s AT %QW312 : UINT;
    P301A_Runtime_s AT %QW313 : UINT;
    P301B_Runtime_s AT %QW314 : UINT;
    Batch_Retry_Count AT %QW315 : UINT;
    Last_Batch_Duration_s AT %QW316 : UINT;
    P101A_RunFb AT %QX0.0 : BOOL;
    P101B_RunFb AT %QX0.1 : BOOL;
    P201_RunFb AT %QX0.2 : BOOL;
    P301A_RunFb AT %QX0.3 : BOOL;
    P301B_RunFb AT %QX0.4 : BOOL;
    M201_RunFb AT %QX0.5 : BOOL;
    XV201_OpenFb AT %QX0.6 : BOOL;
    XV201_ClosedFb AT %QX0.7 : BOOL;
    P101A_StartCmd AT %QX6.2 : BOOL;
    P101B_StartCmd AT %QX6.3 : BOOL;
    P201_StartCmd AT %QX6.4 : BOOL;
    P301A_StartCmd AT %QX6.5 : BOOL;
    P301B_StartCmd AT %QX6.6 : BOOL;
    M201_StartCmd AT %QX6.7 : BOOL;
    XV201_OpenCmd AT %QX7.0 : BOOL;
    System_StartReq AT %QX12.4 : BOOL;
    System_StopReq AT %QX12.5 : BOOL;
    System_ResetReq AT %QX12.6 : BOOL;
    Auto_ModeReq AT %QX12.7 : BOOL;
    Manual_ModeReq AT %QX13.0 : BOOL;
    Maintenance_ModeReq AT %QX13.1 : BOOL;
    Emergency_Stop AT %QX13.5 : BOOL;
    P101A_ManualStartReq AT %QX13.6 : BOOL;
    P101B_ManualStartReq AT %QX13.7 : BOOL;
    P201_ManualStartReq AT %QX14.0 : BOOL;
    P301A_ManualStartReq AT %QX14.1 : BOOL;
    P301B_ManualStartReq AT %QX14.2 : BOOL;
    DP201_ManualEnableReq AT %QX14.3 : BOOL;
    M201_ManualStartReq AT %QX14.4 : BOOL;
    XV201_ManualOpenReq AT %QX14.5 : BOOL;
    Comm_Healthy AT %QX18.6 : BOOL;
    System_Running AT %QX18.7 : BOOL;
    Batch_Accepted AT %QX19.0 : BOOL;
    P101A_Available AT %QX19.1 : BOOL;
    P101B_Available AT %QX19.2 : BOOL;
    P301A_Available AT %QX19.3 : BOOL;
    P301B_Available AT %QX19.4 : BOOL;
    Pressure_PI_Enabled AT %QX19.5 : BOOL;
    Concentration_PI_Enabled AT %QX19.6 : BOOL;
    P301A_Lead AT %QX19.7 : BOOL;
    P301B_Lead AT %QX20.0 : BOOL;
    Any_Alarm AT %QX20.1 : BOOL;
    Sequence_Faulted AT %QX20.2 : BOOL;
    Batch_Quality_OK AT %QX20.3 : BOOL;
    T101_LowLow AT %QX20.4 : BOOL;
    T201_HighHigh AT %QX20.5 : BOOL;
    T301_LowLow AT %QX20.6 : BOOL;
    T301_HighHigh AT %QX20.7 : BOOL;
    Concentration_HighHigh AT %QX21.0 : BOOL;
    Pressure_Low AT %QX21.1 : BOOL;
    Pressure_High AT %QX21.2 : BOOL;
    P301A_FailedStart AT %QX21.3 : BOOL;
    P301B_FailedStart AT %QX21.4 : BOOL;
    P201_FailedStart AT %QX21.5 : BOOL;
    XV201_FailedOpen AT %QX21.6 : BOOL;
    Communication_Fault AT %QX21.7 : BOOL;
    Both_Boosters_Unavailable AT %QX22.0 : BOOL;
    Dosing_Timeout AT %QX22.1 : BOOL;
    Fill_Timeout AT %QX22.2 : BOOL;
    Transfer_Timeout AT %QX22.3 : BOOL;
    AIT201_Invalid AT %QX22.4 : BOOL;
    Fault_P301A_Trip AT %QX31.2 : BOOL;
    Fault_AIT201_Freeze AT %QX31.3 : BOOL;
    Fault_XV201_StuckClosed AT %QX31.4 : BOOL;
    Fault_CommsLoss AT %QX31.5 : BOOL;
END_VAR

VAR CONSTANT
    MODE_STOPPED : UINT := 0;
    MODE_AUTO : UINT := 1;
    MODE_MANUAL : UINT := 2;
    MODE_MAINTENANCE : UINT := 3;

    STATE_STOPPED : UINT := 0;
    STATE_READY : UINT := 10;
    STATE_FILL : UINT := 20;
    STATE_DOSE : UINT := 30;
    STATE_MIX : UINT := 40;
    STATE_VERIFY : UINT := 50;
    STATE_TRANSFER : UINT := 60;
    STATE_COMPLETE : UINT := 70;
    STATE_FAULTED : UINT := 900;

    PRESSURE_KP : REAL := 0.1721;
    PRESSURE_KI_TS : REAL := 0.01033;
    PRESSURE_BIAS_PCT : REAL := 91.6667;
    CONCENTRATION_KP : REAL := 100.0000;
    CONCENTRATION_KI_TS : REAL := 0.1000;

    PRESSURE_OUT_MIN : REAL := 0.0;
    PRESSURE_OUT_MAX : REAL := 200.0;
    CONCENTRATION_OUT_MIN : REAL := 0.0;
    CONCENTRATION_OUT_MAX : REAL := 100.0;
END_VAR

VAR
    LIT101_pct : REAL := 0.0;
    LIT201_pct : REAL := 0.0;
    LIT301_pct : REAL := 0.0;
    FIT101_Lps : REAL := 0.0;
    FIT201_Lps : REAL := 0.0;
    FIT301_Lps : REAL := 0.0;
    AIT201_mgL : REAL := 0.0;
    PIT301_kPa : REAL := 0.0;
    PressureSP_kPa : REAL := 400.0;
    ConcentrationSP_mgL : REAL := 1.20;
    BatchFillSP_pct : REAL := 80.0;
    MixTime_s : UINT := 120;
    ManualDose_pct : REAL := 0.0;

    StartEdge : BOOL := FALSE;
    StopEdge : BOOL := FALSE;
    ResetEdge : BOOL := FALSE;
    AutoModeEdge : BOOL := FALSE;
    ManualModeEdge : BOOL := FALSE;
    MaintenanceModeEdge : BOOL := FALSE;
    Last_StartReq : BOOL := FALSE;
    Last_StopReq : BOOL := FALSE;
    Last_ResetReq : BOOL := FALSE;
    Last_AutoModeReq : BOOL := FALSE;
    Last_ManualModeReq : BOOL := FALSE;
    Last_MaintenanceModeReq : BOOL := FALSE;

    LastHeartbeat : UINT := 0;
    WatchdogScans : UINT := 65535;
    HeartbeatSeen : BOOL := FALSE;

    PressureError : REAL := 0.0;
    PressureP : REAL := 0.0;
    PressureI : REAL := 0.0;
    PressureICandidate : REAL := 0.0;
    PressureUnsat : REAL := 0.0;
    PressureOut_pct : REAL := 0.0;
    ConcError : REAL := 0.0;
    ConcP : REAL := 0.0;
    ConcI : REAL := 0.0;
    ConcICandidate : REAL := 0.0;
    ConcUnsat : REAL := 0.0;
    ConcOut_pct : REAL := 0.0;
    ConcExecutionCounter : UINT := 0;

    P101A_Speed_pct : REAL := 0.0;
    P101B_Speed_pct : REAL := 0.0;
    P201_Speed_pct : REAL := 0.0;
    P301A_Speed_pct : REAL := 0.0;
    P301B_Speed_pct : REAL := 0.0;
    DoseOutput_pct : REAL := 0.0;
    Cmd_P101A : BOOL := FALSE;
    Cmd_P101B : BOOL := FALSE;
    Cmd_P201 : BOOL := FALSE;
    Cmd_P301A : BOOL := FALSE;
    Cmd_P301B : BOOL := FALSE;
    Cmd_M201 : BOOL := FALSE;
    Cmd_XV201 : BOOL := FALSE;

    P101Duty : UINT := 0;
    LeadBoosterInternal : UINT := 1;
    LagEnabled : BOOL := FALSE;
    StageCounter : UINT := 0;
    DestageCounter : UINT := 0;

    P101A_StartCounter : UINT := 0;
    P101B_StartCounter : UINT := 0;
    P201_StartCounter : UINT := 0;
    P301A_StartCounter : UINT := 0;
    P301B_StartCounter : UINT := 0;
    ValveOpenCounter : UINT := 0;
    P101A_FailedStart : BOOL := FALSE;
    P101B_FailedStart : BOOL := FALSE;

    FillCounter : UDINT := 0;
    DoseCounter : UDINT := 0;
    MixCounter : UDINT := 0;
    TransferCounter : UDINT := 0;
    StableCounter : UINT := 0;
    BatchDurationScans : UDINT := 0;
    LastState : UINT := 0;

    AIT_LastRaw : UINT := 0;
    AIT_FrozenCounter : UINT := 0;

    RuntimeSubsecond : UINT := 0;
    P101A_RuntimeInternal_s : UDINT := 0;
    P101B_RuntimeInternal_s : UDINT := 0;
    P301A_RuntimeInternal_s : UDINT := 0;
    P301B_RuntimeInternal_s : UDINT := 0;

    AlarmWord1Internal : UINT := 0;
    AlarmWord2Internal : UINT := 0;
END_VAR

(* ============================================================
   AUTOMATED WATER TREATMENT STATION - OPENPLC MAIN PROGRAM
   Scan period required: 100 ms
   External analog values are scaled UINT16 values over Modbus TCP.
   Simulink owns the physical plant. OpenPLC owns operational control.
   ============================================================ *)

(* Scale external values into engineering units. *)
LIT101_pct := UINT_TO_REAL(LIT101_PV) / 10.0;
LIT201_pct := UINT_TO_REAL(LIT201_PV) / 10.0;
LIT301_pct := UINT_TO_REAL(LIT301_PV) / 10.0;
FIT101_Lps := UINT_TO_REAL(FIT101_PV) / 10.0;
FIT201_Lps := UINT_TO_REAL(FIT201_PV) / 10.0;
FIT301_Lps := UINT_TO_REAL(FIT301_PV) / 10.0;
AIT201_mgL := UINT_TO_REAL(AIT201_PV) / 100.0;
PIT301_kPa := UINT_TO_REAL(PIT301_PV) / 10.0;

IF (Pressure_SP >= 3200) AND (Pressure_SP <= 5000) THEN
    PressureSP_kPa := UINT_TO_REAL(Pressure_SP) / 10.0;
ELSE
    PressureSP_kPa := 400.0;
END_IF;
IF (Concentration_SP >= 80) AND (Concentration_SP <= 150) THEN
    ConcentrationSP_mgL := UINT_TO_REAL(Concentration_SP) / 100.0;
ELSE
    ConcentrationSP_mgL := 1.20;
END_IF;
IF (BatchFill_SP >= 500) AND (BatchFill_SP <= 900) THEN
    BatchFillSP_pct := UINT_TO_REAL(BatchFill_SP) / 10.0;
ELSE
    BatchFillSP_pct := 80.0;
END_IF;
IF (MixTime_SP >= 1) AND (MixTime_SP <= 600) THEN
    MixTime_s := MixTime_SP;
ELSE
    MixTime_s := 120;
END_IF;
IF Manual_Dose_Output <= 1000 THEN
    ManualDose_pct := UINT_TO_REAL(Manual_Dose_Output) / 10.0;
ELSE
    ManualDose_pct := 100.0;
END_IF;

(* Rising-edge detection for operator requests. *)
StartEdge := System_StartReq AND NOT Last_StartReq;
StopEdge := System_StopReq AND NOT Last_StopReq;
ResetEdge := System_ResetReq AND NOT Last_ResetReq;
AutoModeEdge := Auto_ModeReq AND NOT Last_AutoModeReq;
ManualModeEdge := Manual_ModeReq AND NOT Last_ManualModeReq;
MaintenanceModeEdge := Maintenance_ModeReq AND NOT Last_MaintenanceModeReq;
Last_StartReq := System_StartReq;
Last_StopReq := System_StopReq;
Last_ResetReq := System_ResetReq;
Last_AutoModeReq := Auto_ModeReq;
Last_ManualModeReq := Manual_ModeReq;
Last_MaintenanceModeReq := Maintenance_ModeReq;

(* Simulink heartbeat watchdog: 20 scans = 2.0 seconds. *)
IF Plant_Heartbeat <> LastHeartbeat THEN
    LastHeartbeat := Plant_Heartbeat;
    WatchdogScans := 0;
    HeartbeatSeen := TRUE;
ELSE
    IF WatchdogScans < 65535 THEN WatchdogScans := WatchdogScans + 1; END_IF;
END_IF;
Comm_Healthy := HeartbeatSeen AND (WatchdogScans < 20) AND NOT Fault_CommsLoss;
Communication_Fault := NOT Comm_Healthy;

(* Alarm conditions and basic availability. *)
T101_LowLow := LIT101_pct <= 5.0;
T201_HighHigh := LIT201_pct >= 95.0;
T301_LowLow := LIT301_pct <= 5.0;
T301_HighHigh := LIT301_pct >= 95.0;
Concentration_HighHigh := AIT201_mgL >= 1.50;
Pressure_Low := PIT301_kPa < 320.0;
Pressure_High := PIT301_kPa > 500.0;
Batch_Quality_OK := (AIT201_mgL >= 1.10) AND (AIT201_mgL <= 1.30) AND NOT AIT201_Invalid;
P101A_Available := NOT P101A_FailedStart;
P101B_Available := NOT P101B_FailedStart;
P301A_Available := NOT Fault_P301A_Trip AND NOT P301A_FailedStart;
P301B_Available := NOT P301B_FailedStart;
Both_Boosters_Unavailable := NOT P301A_Available AND NOT P301B_Available;

(* Detect a frozen concentration signal during active dosing. *)
IF Concentration_PI_Enabled AND (DoseOutput_pct > 20.0) AND
   (((ConcentrationSP_mgL - AIT201_mgL) > 0.15) OR ((ConcentrationSP_mgL - AIT201_mgL) < -0.15)) THEN
    IF AIT201_PV = AIT_LastRaw THEN
        IF AIT_FrozenCounter < 65535 THEN AIT_FrozenCounter := AIT_FrozenCounter + 1; END_IF;
    ELSE
        AIT_FrozenCounter := 0;
    END_IF;
ELSE
    AIT_FrozenCounter := 0;
END_IF;
AIT_LastRaw := AIT201_PV;
AIT201_Invalid := (AIT_FrozenCounter >= 150);

(* Mode handling. Critical faults always force a safe stopped mode. *)
IF AutoModeEdge THEN Requested_Mode := MODE_AUTO; END_IF;
IF ManualModeEdge THEN Requested_Mode := MODE_MANUAL; END_IF;
IF MaintenanceModeEdge THEN Requested_Mode := MODE_MAINTENANCE; END_IF;
IF StopEdge THEN
    System_Running := FALSE;
    Batch_State := STATE_STOPPED;
    Batch_Accepted := FALSE;
END_IF;
IF Emergency_Stop OR Communication_Fault THEN
    Active_Mode := MODE_STOPPED;
    System_Running := FALSE;
    IF Batch_State <> STATE_STOPPED THEN Batch_State := STATE_FAULTED; END_IF;
ELSE
    IF Requested_Mode <= MODE_MAINTENANCE THEN Active_Mode := Requested_Mode; END_IF;
END_IF;
IF StartEdge AND (Active_Mode = MODE_AUTO) AND Comm_Healthy AND NOT Emergency_Stop THEN
    System_Running := TRUE;
    IF Batch_State = STATE_STOPPED THEN Batch_State := STATE_READY; END_IF;
    IF P301A_RuntimeInternal_s <= P301B_RuntimeInternal_s THEN LeadBoosterInternal := 1; ELSE LeadBoosterInternal := 2; END_IF;
END_IF;

(* Reset only when initiating critical conditions are clear. *)
IF ResetEdge AND Comm_Healthy AND NOT Emergency_Stop AND NOT Concentration_HighHigh THEN
    FirstOut_Code := 0;
    P101A_FailedStart := FALSE;
    P101B_FailedStart := FALSE;
    P201_FailedStart := FALSE;
    P301A_FailedStart := FALSE;
    P301B_FailedStart := FALSE;
    XV201_FailedOpen := FALSE;
    Fill_Timeout := FALSE;
    Dosing_Timeout := FALSE;
    Transfer_Timeout := FALSE;
    AIT201_Invalid := FALSE;
    AIT_FrozenCounter := 0;
    PressureI := 0.0;
    ConcI := 0.0;
    IF Batch_State = STATE_FAULTED THEN Batch_State := STATE_STOPPED; END_IF;
END_IF;

(* Default commands are safe each scan; logic below must explicitly enable them. *)
Cmd_P101A := FALSE; Cmd_P101B := FALSE; Cmd_P201 := FALSE;
Cmd_P301A := FALSE; Cmd_P301B := FALSE; Cmd_M201 := FALSE; Cmd_XV201 := FALSE;
P101A_Speed_pct := 0.0; P101B_Speed_pct := 0.0; P201_Speed_pct := 0.0;
P301A_Speed_pct := 0.0; P301B_Speed_pct := 0.0; DoseOutput_pct := 0.0;
Concentration_PI_Enabled := FALSE;
Pressure_PI_Enabled := FALSE;

(* Automatic treatment batch state machine. *)
IF (Active_Mode = MODE_AUTO) AND System_Running AND Comm_Healthy AND NOT Emergency_Stop THEN
    BatchDurationScans := BatchDurationScans + 1;
    CASE Batch_State OF
        0: (* STATE_STOPPED; literal required by OpenPLC Runtime v4 C++ code generation *)
            Batch_State := STATE_READY;

        10: (* STATE_READY *)
            FillCounter := 0; DoseCounter := 0; MixCounter := 0; TransferCounter := 0;
            StableCounter := 0; Batch_Accepted := FALSE; Batch_Retry_Count := 0;
            IF NOT T101_LowLow AND NOT T201_HighHigh AND NOT T301_HighHigh AND (P101A_Available OR P101B_Available) THEN
                P101Duty := 0;
                Batch_State := STATE_FILL;
            END_IF;

        20: (* STATE_FILL *)
            FillCounter := FillCounter + 1;
            IF P101Duty = 0 THEN
                IF P101A_Available AND ((P101A_RuntimeInternal_s <= P101B_RuntimeInternal_s) OR NOT P101B_Available) THEN
                    P101Duty := 1;
                ELSIF P101B_Available THEN
                    P101Duty := 2;
                ELSE
                    Batch_State := STATE_FAULTED;
                END_IF;
            END_IF;
            IF NOT T101_LowLow AND NOT T201_HighHigh THEN
                IF (P101Duty = 1) AND P101A_Available THEN Cmd_P101A := TRUE; P101A_Speed_pct := 100.0; END_IF;
                IF (P101Duty = 2) AND P101B_Available THEN Cmd_P101B := TRUE; P101B_Speed_pct := 100.0; END_IF;
            END_IF;
            IF LIT201_pct >= BatchFillSP_pct THEN
                Batch_State := STATE_DOSE; FillCounter := 0; StableCounter := 0;
            ELSIF FillCounter >= 15000 THEN
                Fill_Timeout := TRUE; Batch_State := STATE_FAULTED;
            END_IF;

        30: (* STATE_DOSE *)
            DoseCounter := DoseCounter + 1;
            Cmd_M201 := TRUE;
            Concentration_PI_Enabled := (LIT201_pct >= 20.0) AND M201_RunFb AND NOT Concentration_HighHigh AND NOT AIT201_Invalid;
            IF Concentration_PI_Enabled THEN
                ConcExecutionCounter := ConcExecutionCounter + 1;
                IF ConcExecutionCounter >= 5 THEN
                    ConcExecutionCounter := 0;
                    ConcError := ConcentrationSP_mgL - AIT201_mgL;
                    ConcP := CONCENTRATION_KP * ConcError;
                    ConcICandidate := ConcI + CONCENTRATION_KI_TS * ConcError;
                    ConcUnsat := ConcP + ConcICandidate;
                    IF ConcUnsat > CONCENTRATION_OUT_MAX THEN ConcOut_pct := CONCENTRATION_OUT_MAX;
                    ELSIF ConcUnsat < CONCENTRATION_OUT_MIN THEN ConcOut_pct := CONCENTRATION_OUT_MIN;
                    ELSE ConcOut_pct := ConcUnsat; END_IF;
                    IF ((ConcUnsat >= CONCENTRATION_OUT_MIN) AND (ConcUnsat <= CONCENTRATION_OUT_MAX)) OR
                       ((ConcUnsat > CONCENTRATION_OUT_MAX) AND (ConcError < 0.0)) OR
                       ((ConcUnsat < CONCENTRATION_OUT_MIN) AND (ConcError > 0.0)) THEN
                        ConcI := ConcICandidate;
                    END_IF;
                END_IF;
                DoseOutput_pct := ConcOut_pct;
            END_IF;
            IF Batch_Quality_OK THEN
                IF StableCounter < 65535 THEN StableCounter := StableCounter + 1; END_IF;
            ELSE
                StableCounter := 0;
            END_IF;
            IF StableCounter >= 200 THEN
                Batch_State := STATE_MIX; MixCounter := 0; DoseOutput_pct := 0.0;
            ELSIF DoseCounter >= 1800 THEN
                Dosing_Timeout := TRUE; Batch_State := STATE_FAULTED;
            ELSIF Concentration_HighHigh OR AIT201_Invalid THEN
                Batch_State := STATE_FAULTED;
            END_IF;

        40: (* STATE_MIX *)
            Cmd_M201 := TRUE;
            MixCounter := MixCounter + 1;
            IF MixCounter >= UINT_TO_UDINT(MixTime_s) * 10 THEN
                Batch_State := STATE_VERIFY;
            END_IF;

        50: (* STATE_VERIFY *)
            IF Batch_Quality_OK AND M201_RunFb AND NOT T301_HighHigh THEN
                Batch_Accepted := TRUE;
                Batch_State := STATE_TRANSFER;
                TransferCounter := 0; ValveOpenCounter := 0;
            ELSE
                IF Batch_Retry_Count < 2 THEN
                    Batch_Retry_Count := Batch_Retry_Count + 1;
                    DoseCounter := 0; StableCounter := 0; ConcI := 0.0;
                    Batch_State := STATE_DOSE;
                ELSE
                    Batch_State := STATE_FAULTED;
                END_IF;
            END_IF;

        60: (* STATE_TRANSFER *)
            TransferCounter := TransferCounter + 1;
            Cmd_XV201 := TRUE;
            IF NOT XV201_OpenFb THEN
                IF ValveOpenCounter < 65535 THEN ValveOpenCounter := ValveOpenCounter + 1; END_IF;
            ELSE
                ValveOpenCounter := 0;
                IF Batch_Accepted AND NOT T301_HighHigh AND (LIT201_pct > 10.0) THEN
                    Cmd_P201 := TRUE; P201_Speed_pct := 100.0;
                END_IF;
            END_IF;
            IF LIT201_pct <= 10.0 THEN
                Batch_State := STATE_COMPLETE;
            (* The simulated 2.0 s first-order valve reaches the 95% open
               feedback threshold after about 6.0 s. Allow 8.0 s so a healthy
               valve is not declared failed before its feedback can arrive. *)
            ELSIF ValveOpenCounter >= 80 THEN
                XV201_FailedOpen := TRUE; Batch_State := STATE_FAULTED;
            ELSIF TransferCounter >= 12000 THEN
                Transfer_Timeout := TRUE; Batch_State := STATE_FAULTED;
            END_IF;

        70: (* STATE_COMPLETE *)
            Batch_Count := Batch_Count + 1;
            Last_Batch_Duration_s := UDINT_TO_UINT(BatchDurationScans / 10);
            BatchDurationScans := 0;
            Batch_Accepted := FALSE;
            P101Duty := 0;
            IF LeadBoosterInternal = 1 THEN LeadBoosterInternal := 2; ELSE LeadBoosterInternal := 1; END_IF;
            Batch_State := STATE_READY;

        900: (* STATE_FAULTED *)
            System_Running := FALSE;

        ELSE
            Batch_State := STATE_FAULTED;
    END_CASE;
END_IF;

(* Manual and maintenance commands retain critical interlocks. *)
IF ((Active_Mode = MODE_MANUAL) OR (Active_Mode = MODE_MAINTENANCE)) AND Comm_Healthy AND NOT Emergency_Stop THEN
    IF P101A_ManualStartReq AND NOT T101_LowLow AND NOT T201_HighHigh AND P101A_Available THEN Cmd_P101A := TRUE; P101A_Speed_pct := 100.0; END_IF;
    IF P101B_ManualStartReq AND NOT T101_LowLow AND NOT T201_HighHigh AND P101B_Available THEN Cmd_P101B := TRUE; P101B_Speed_pct := 100.0; END_IF;
    IF XV201_ManualOpenReq THEN Cmd_XV201 := TRUE; END_IF;
    IF P201_ManualStartReq AND Batch_Accepted AND XV201_OpenFb AND NOT T301_HighHigh AND (LIT201_pct > 10.0) THEN Cmd_P201 := TRUE; P201_Speed_pct := 100.0; END_IF;
    IF M201_ManualStartReq AND (LIT201_pct >= 20.0) THEN Cmd_M201 := TRUE; END_IF;
    IF DP201_ManualEnableReq AND M201_RunFb AND (LIT201_pct >= 20.0) AND NOT Concentration_HighHigh AND NOT AIT201_Invalid THEN DoseOutput_pct := ManualDose_pct; END_IF;
    IF P301A_ManualStartReq AND NOT T301_LowLow AND P301A_Available THEN Cmd_P301A := TRUE; P301A_Speed_pct := 100.0; END_IF;
    IF P301B_ManualStartReq AND NOT T301_LowLow AND P301B_Available THEN Cmd_P301B := TRUE; P301B_Speed_pct := 100.0; END_IF;
END_IF;

(* Distribution pressure PI and booster staging in automatic mode. *)
IF (Active_Mode = MODE_AUTO) AND System_Running AND Comm_Healthy AND NOT Emergency_Stop AND NOT T301_LowLow AND NOT Both_Boosters_Unavailable THEN
    Pressure_PI_Enabled := TRUE;
    PressureError := PressureSP_kPa - PIT301_kPa;
    PressureP := PRESSURE_KP * PressureError;
    PressureICandidate := PressureI + PRESSURE_KI_TS * PressureError;
    PressureUnsat := PRESSURE_BIAS_PCT + PressureP + PressureICandidate;
    IF PressureUnsat > PRESSURE_OUT_MAX THEN PressureOut_pct := PRESSURE_OUT_MAX;
    ELSIF PressureUnsat < PRESSURE_OUT_MIN THEN PressureOut_pct := PRESSURE_OUT_MIN;
    ELSE PressureOut_pct := PressureUnsat; END_IF;
    IF ((PressureUnsat >= PRESSURE_OUT_MIN) AND (PressureUnsat <= PRESSURE_OUT_MAX)) OR
       ((PressureUnsat > PRESSURE_OUT_MAX) AND (PressureError < 0.0)) OR
       ((PressureUnsat < PRESSURE_OUT_MIN) AND (PressureError > 0.0)) THEN
        PressureI := PressureICandidate;
    END_IF;

    IF (LeadBoosterInternal = 1) AND NOT P301A_Available THEN LeadBoosterInternal := 2; LagEnabled := FALSE; END_IF;
    IF (LeadBoosterInternal = 2) AND NOT P301B_Available THEN LeadBoosterInternal := 1; LagEnabled := FALSE; END_IF;

    IF PressureOut_pct > 95.0 THEN
        IF StageCounter < 65535 THEN StageCounter := StageCounter + 1; END_IF;
        DestageCounter := 0;
    ELSIF PressureOut_pct < 80.0 THEN
        IF DestageCounter < 65535 THEN DestageCounter := DestageCounter + 1; END_IF;
        StageCounter := 0;
    ELSE
        StageCounter := 0; DestageCounter := 0;
    END_IF;
    IF StageCounter >= 30 THEN LagEnabled := TRUE; END_IF;
    IF DestageCounter >= 50 THEN LagEnabled := FALSE; END_IF;
    IF PressureOut_pct > 100.0 THEN LagEnabled := TRUE; END_IF;

    IF LagEnabled AND P301A_Available AND P301B_Available THEN
        P301A_Speed_pct := PressureOut_pct / 2.0;
        P301B_Speed_pct := PressureOut_pct / 2.0;
        Cmd_P301A := P301A_Speed_pct > 5.0;
        Cmd_P301B := P301B_Speed_pct > 5.0;
    ELSIF (LeadBoosterInternal = 1) AND P301A_Available THEN
        IF PressureOut_pct > 100.0 THEN P301A_Speed_pct := 100.0; ELSE P301A_Speed_pct := PressureOut_pct; END_IF;
        Cmd_P301A := P301A_Speed_pct > 5.0;
    ELSIF (LeadBoosterInternal = 2) AND P301B_Available THEN
        IF PressureOut_pct > 100.0 THEN P301B_Speed_pct := 100.0; ELSE P301B_Speed_pct := PressureOut_pct; END_IF;
        Cmd_P301B := P301B_Speed_pct > 5.0;
    END_IF;
ELSE
    PressureOut_pct := 0.0;
END_IF;

(* Enforce global safe-state and critical process interlocks. *)
IF NOT Comm_Healthy OR Emergency_Stop OR (Active_Mode = MODE_STOPPED) THEN
    Cmd_P101A := FALSE; Cmd_P101B := FALSE; Cmd_P201 := FALSE;
    Cmd_P301A := FALSE; Cmd_P301B := FALSE; Cmd_M201 := FALSE; Cmd_XV201 := FALSE;
    P101A_Speed_pct := 0.0; P101B_Speed_pct := 0.0; P201_Speed_pct := 0.0;
    P301A_Speed_pct := 0.0; P301B_Speed_pct := 0.0; DoseOutput_pct := 0.0;
END_IF;
IF T101_LowLow THEN Cmd_P101A := FALSE; Cmd_P101B := FALSE; P101A_Speed_pct := 0.0; P101B_Speed_pct := 0.0; END_IF;
IF T201_HighHigh THEN Cmd_P101A := FALSE; Cmd_P101B := FALSE; P101A_Speed_pct := 0.0; P101B_Speed_pct := 0.0; END_IF;
IF T301_LowLow THEN Cmd_P301A := FALSE; Cmd_P301B := FALSE; P301A_Speed_pct := 0.0; P301B_Speed_pct := 0.0; END_IF;
IF T301_HighHigh THEN Cmd_P201 := FALSE; P201_Speed_pct := 0.0; END_IF;
IF Concentration_HighHigh OR AIT201_Invalid OR NOT M201_RunFb OR (LIT201_pct < 20.0) THEN DoseOutput_pct := 0.0; END_IF;
IF NOT XV201_OpenFb THEN Cmd_P201 := FALSE; P201_Speed_pct := 0.0; END_IF;

(* Failed-to-start supervision: 20 scans = 2 seconds. *)
IF Cmd_P101A AND NOT P101A_RunFb THEN P101A_StartCounter := P101A_StartCounter + 1; ELSE P101A_StartCounter := 0; END_IF;
IF Cmd_P101B AND NOT P101B_RunFb THEN P101B_StartCounter := P101B_StartCounter + 1; ELSE P101B_StartCounter := 0; END_IF;
IF Cmd_P201 AND NOT P201_RunFb THEN P201_StartCounter := P201_StartCounter + 1; ELSE P201_StartCounter := 0; END_IF;
IF Cmd_P301A AND NOT P301A_RunFb THEN P301A_StartCounter := P301A_StartCounter + 1; ELSE P301A_StartCounter := 0; END_IF;
IF Cmd_P301B AND NOT P301B_RunFb THEN P301B_StartCounter := P301B_StartCounter + 1; ELSE P301B_StartCounter := 0; END_IF;
IF P101A_StartCounter >= 20 THEN P101A_FailedStart := TRUE; IF P101Duty = 1 THEN P101Duty := 2; END_IF; END_IF;
IF P101B_StartCounter >= 20 THEN P101B_FailedStart := TRUE; IF P101Duty = 2 THEN P101Duty := 1; END_IF; END_IF;
IF P201_StartCounter >= 20 THEN P201_FailedStart := TRUE; Batch_State := STATE_FAULTED; END_IF;
IF P301A_StartCounter >= 20 THEN P301A_FailedStart := TRUE; LeadBoosterInternal := 2; END_IF;
IF P301B_StartCounter >= 20 THEN P301B_FailedStart := TRUE; LeadBoosterInternal := 1; END_IF;

(* Runtime accumulation at one-second intervals. *)
RuntimeSubsecond := RuntimeSubsecond + 1;
IF RuntimeSubsecond >= 10 THEN
    RuntimeSubsecond := 0;
    IF P101A_RunFb THEN P101A_RuntimeInternal_s := P101A_RuntimeInternal_s + 1; END_IF;
    IF P101B_RunFb THEN P101B_RuntimeInternal_s := P101B_RuntimeInternal_s + 1; END_IF;
    IF P301A_RunFb THEN P301A_RuntimeInternal_s := P301A_RuntimeInternal_s + 1; END_IF;
    IF P301B_RunFb THEN P301B_RuntimeInternal_s := P301B_RuntimeInternal_s + 1; END_IF;
END_IF;

(* Count rejected batches only on entry to the faulted state. *)
IF (Batch_State = STATE_FAULTED) AND (LastState <> STATE_FAULTED) AND (LastState >= STATE_FILL) THEN
    Rejected_Batch_Count := Rejected_Batch_Count + 1;
END_IF;
LastState := Batch_State;

(* First-out fault priority. *)
IF FirstOut_Code = 0 THEN
    IF Emergency_Stop THEN FirstOut_Code := 1;
    ELSIF Communication_Fault THEN FirstOut_Code := 2;
    ELSIF T101_LowLow THEN FirstOut_Code := 10;
    ELSIF T201_HighHigh THEN FirstOut_Code := 11;
    ELSIF T301_LowLow THEN FirstOut_Code := 12;
    ELSIF T301_HighHigh THEN FirstOut_Code := 13;
    ELSIF Concentration_HighHigh THEN FirstOut_Code := 20;
    ELSIF AIT201_Invalid THEN FirstOut_Code := 21;
    ELSIF P301A_FailedStart THEN FirstOut_Code := 33;
    ELSIF P301B_FailedStart THEN FirstOut_Code := 34;
    ELSIF P201_FailedStart THEN FirstOut_Code := 32;
    ELSIF XV201_FailedOpen THEN FirstOut_Code := 40;
    ELSIF Fill_Timeout THEN FirstOut_Code := 60;
    ELSIF Dosing_Timeout THEN FirstOut_Code := 61;
    ELSIF Transfer_Timeout THEN FirstOut_Code := 62;
    END_IF;
END_IF;

(* Alarm words for compact SCADA diagnostics. *)
AlarmWord1Internal := 0;
IF Emergency_Stop THEN AlarmWord1Internal := AlarmWord1Internal + 1; END_IF;
IF Communication_Fault THEN AlarmWord1Internal := AlarmWord1Internal + 2; END_IF;
IF T101_LowLow THEN AlarmWord1Internal := AlarmWord1Internal + 4; END_IF;
IF T201_HighHigh THEN AlarmWord1Internal := AlarmWord1Internal + 8; END_IF;
IF T301_LowLow THEN AlarmWord1Internal := AlarmWord1Internal + 16; END_IF;
IF T301_HighHigh THEN AlarmWord1Internal := AlarmWord1Internal + 32; END_IF;
IF Concentration_HighHigh THEN AlarmWord1Internal := AlarmWord1Internal + 64; END_IF;
IF AIT201_Invalid THEN AlarmWord1Internal := AlarmWord1Internal + 128; END_IF;
IF Pressure_Low THEN AlarmWord1Internal := AlarmWord1Internal + 256; END_IF;
IF Pressure_High THEN AlarmWord1Internal := AlarmWord1Internal + 512; END_IF;
AlarmWord2Internal := 0;
IF P301A_FailedStart THEN AlarmWord2Internal := AlarmWord2Internal + 1; END_IF;
IF P301B_FailedStart THEN AlarmWord2Internal := AlarmWord2Internal + 2; END_IF;
IF P201_FailedStart THEN AlarmWord2Internal := AlarmWord2Internal + 4; END_IF;
IF XV201_FailedOpen THEN AlarmWord2Internal := AlarmWord2Internal + 8; END_IF;
IF Both_Boosters_Unavailable THEN AlarmWord2Internal := AlarmWord2Internal + 16; END_IF;
IF Fill_Timeout THEN AlarmWord2Internal := AlarmWord2Internal + 32; END_IF;
IF Dosing_Timeout THEN AlarmWord2Internal := AlarmWord2Internal + 64; END_IF;
IF Transfer_Timeout THEN AlarmWord2Internal := AlarmWord2Internal + 128; END_IF;
Alarm_Word_1 := AlarmWord1Internal;
Alarm_Word_2 := AlarmWord2Internal;
Any_Alarm := (AlarmWord1Internal <> 0) OR (AlarmWord2Internal <> 0);
Sequence_Faulted := Batch_State = STATE_FAULTED;

(* Map internal commands and diagnostics to Modbus-visible locations. *)
P101A_StartCmd := Cmd_P101A; P101B_StartCmd := Cmd_P101B; P201_StartCmd := Cmd_P201;
P301A_StartCmd := Cmd_P301A; P301B_StartCmd := Cmd_P301B; M201_StartCmd := Cmd_M201; XV201_OpenCmd := Cmd_XV201;
P101A_SpeedCmd := REAL_TO_UINT(P101A_Speed_pct * 10.0);
P101B_SpeedCmd := REAL_TO_UINT(P101B_Speed_pct * 10.0);
P201_SpeedCmd := REAL_TO_UINT(P201_Speed_pct * 10.0);
P301A_SpeedCmd := REAL_TO_UINT(P301A_Speed_pct * 10.0);
P301B_SpeedCmd := REAL_TO_UINT(P301B_Speed_pct * 10.0);
DP201_Output := REAL_TO_UINT(DoseOutput_pct * 10.0);
Pressure_PI_Output := REAL_TO_UINT(PressureOut_pct * 10.0);
Concentration_PI_Output := REAL_TO_UINT(DoseOutput_pct * 10.0);
Lead_Booster := LeadBoosterInternal;
P301A_Lead := LeadBoosterInternal = 1;
P301B_Lead := LeadBoosterInternal = 2;
IF P101A_RuntimeInternal_s > 65535 THEN P101A_Runtime_s := 65535; ELSE P101A_Runtime_s := UDINT_TO_UINT(P101A_RuntimeInternal_s); END_IF;
IF P101B_RuntimeInternal_s > 65535 THEN P101B_Runtime_s := 65535; ELSE P101B_Runtime_s := UDINT_TO_UINT(P101B_RuntimeInternal_s); END_IF;
IF P301A_RuntimeInternal_s > 65535 THEN P301A_Runtime_s := 65535; ELSE P301A_Runtime_s := UDINT_TO_UINT(P301A_RuntimeInternal_s); END_IF;
IF P301B_RuntimeInternal_s > 65535 THEN P301B_Runtime_s := 65535; ELSE P301B_Runtime_s := UDINT_TO_UINT(P301B_RuntimeInternal_s); END_IF;

END_PROGRAM
