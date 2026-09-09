"""tag setup and hmi helpers; run SCADASetup.run("WaterTreatmentHistory") once after import"""

from java.lang import Thread


ROOT = "[default]WaterTreatment/"

PULSE_REQUESTS = set(['Commands/System_StartReq', 'Commands/System_StopReq', 'Commands/System_ResetReq', 'Commands/Auto_ModeReq', 'Commands/Manual_ModeReq', 'Commands/Maintenance_ModeReq'])

SUSTAINED_REQUESTS = set(['Equipment/P101A/P101A_ManualStartReq', 'Equipment/P101B/P101B_ManualStartReq', 'Equipment/P201/P201_ManualStartReq', 'Equipment/P301A/P301A_ManualStartReq', 'Equipment/P301B/P301B_ManualStartReq', 'Equipment/DP201/DP201_ManualEnableReq', 'Equipment/M201/M201_ManualStartReq', 'Equipment/XV201/XV201_ManualOpenReq'])

FAULT_REQUESTS = set(['Simulation/Fault_P301A_Trip', 'Simulation/Fault_AIT201_Freeze', 'Simulation/Fault_XV201_StuckClosed', 'Simulation/Fault_CommsLoss'])

SETPOINTS = {'Control/PressurePI/Pressure_SP': (320.0, 500.0, 5.0), 'Control/ConcentrationPI/Concentration_SP': (0.8, 1.5, 0.05), 'Control/Batch/BatchFill_SP': (50.0, 90.0, 5.0), 'Control/Batch/MixTime_SP': (1.0, 600.0, 5.0), 'Equipment/DP201/Manual_Dose_Output': (0.0, 100.0, 5.0)}

ALL_TAGS = ['[default]WaterTreatment/Plant/T101/LIT101_PV', '[default]WaterTreatment/Plant/T201/LIT201_PV', '[default]WaterTreatment/Plant/T301/LIT301_PV', '[default]WaterTreatment/Plant/Flows/FIT101_PV', '[default]WaterTreatment/Plant/Flows/FIT201_PV', '[default]WaterTreatment/Plant/Distribution/FIT301_PV', '[default]WaterTreatment/Plant/T201/AIT201_PV', '[default]WaterTreatment/Plant/Distribution/PIT301_PV', '[default]WaterTreatment/Plant/Utilities/Plant_Heartbeat', '[default]WaterTreatment/Equipment/P101A/P101A_SpeedCmd', '[default]WaterTreatment/Equipment/P101B/P101B_SpeedCmd', '[default]WaterTreatment/Equipment/P201/P201_SpeedCmd', '[default]WaterTreatment/Equipment/P301A/P301A_SpeedCmd', '[default]WaterTreatment/Equipment/P301B/P301B_SpeedCmd', '[default]WaterTreatment/Equipment/DP201/DP201_Output', '[default]WaterTreatment/Control/PressurePI/Pressure_SP', '[default]WaterTreatment/Control/ConcentrationPI/Concentration_SP', '[default]WaterTreatment/Control/Batch/BatchFill_SP', '[default]WaterTreatment/Control/Batch/MixTime_SP', '[default]WaterTreatment/Equipment/DP201/Manual_Dose_Output', '[default]WaterTreatment/Control/Batch/Batch_State', '[default]WaterTreatment/Control/Batch/Batch_Count', '[default]WaterTreatment/Control/Batch/Rejected_Batch_Count', '[default]WaterTreatment/Control/PressurePI/Pressure_PI_Output', '[default]WaterTreatment/Control/ConcentrationPI/Concentration_PI_Output', '[default]WaterTreatment/Control/Modes/Active_Mode', '[default]WaterTreatment/Control/Modes/Requested_Mode', '[default]WaterTreatment/Control/Modes/Lead_Booster', '[default]WaterTreatment/Alarms/FirstOut_Code', '[default]WaterTreatment/Alarms/Alarm_Word_1', '[default]WaterTreatment/Alarms/Alarm_Word_2', '[default]WaterTreatment/Equipment/P101A/P101A_Runtime_s', '[default]WaterTreatment/Equipment/P101B/P101B_Runtime_s', '[default]WaterTreatment/Equipment/P301A/P301A_Runtime_s', '[default]WaterTreatment/Equipment/P301B/P301B_Runtime_s', '[default]WaterTreatment/Control/Batch/Batch_Retry_Count', '[default]WaterTreatment/Control/Batch/Last_Batch_Duration_s', '[default]WaterTreatment/Equipment/P101A/P101A_RunFb', '[default]WaterTreatment/Equipment/P101B/P101B_RunFb', '[default]WaterTreatment/Equipment/P201/P201_RunFb', '[default]WaterTreatment/Equipment/P301A/P301A_RunFb', '[default]WaterTreatment/Equipment/P301B/P301B_RunFb', '[default]WaterTreatment/Equipment/M201/M201_RunFb', '[default]WaterTreatment/Equipment/XV201/XV201_OpenFb', '[default]WaterTreatment/Equipment/XV201/XV201_ClosedFb', '[default]WaterTreatment/Equipment/P101A/P101A_StartCmd', '[default]WaterTreatment/Equipment/P101B/P101B_StartCmd', '[default]WaterTreatment/Equipment/P201/P201_StartCmd', '[default]WaterTreatment/Equipment/P301A/P301A_StartCmd', '[default]WaterTreatment/Equipment/P301B/P301B_StartCmd', '[default]WaterTreatment/Equipment/M201/M201_StartCmd', '[default]WaterTreatment/Equipment/XV201/XV201_OpenCmd', '[default]WaterTreatment/Commands/System_StartReq', '[default]WaterTreatment/Commands/System_StopReq', '[default]WaterTreatment/Commands/System_ResetReq', '[default]WaterTreatment/Commands/Auto_ModeReq', '[default]WaterTreatment/Commands/Manual_ModeReq', '[default]WaterTreatment/Commands/Maintenance_ModeReq', '[default]WaterTreatment/Commands/Emergency_Stop', '[default]WaterTreatment/Equipment/P101A/P101A_ManualStartReq', '[default]WaterTreatment/Equipment/P101B/P101B_ManualStartReq', '[default]WaterTreatment/Equipment/P201/P201_ManualStartReq', '[default]WaterTreatment/Equipment/P301A/P301A_ManualStartReq', '[default]WaterTreatment/Equipment/P301B/P301B_ManualStartReq', '[default]WaterTreatment/Equipment/DP201/DP201_ManualEnableReq', '[default]WaterTreatment/Equipment/M201/M201_ManualStartReq', '[default]WaterTreatment/Equipment/XV201/XV201_ManualOpenReq', '[default]WaterTreatment/Diagnostics/Comm_Healthy', '[default]WaterTreatment/Diagnostics/System_Running', '[default]WaterTreatment/Control/Batch/Batch_Accepted', '[default]WaterTreatment/Equipment/P101A/P101A_Available', '[default]WaterTreatment/Equipment/P101B/P101B_Available', '[default]WaterTreatment/Equipment/P301A/P301A_Available', '[default]WaterTreatment/Equipment/P301B/P301B_Available', '[default]WaterTreatment/Control/PressurePI/Pressure_PI_Enabled', '[default]WaterTreatment/Control/ConcentrationPI/Concentration_PI_Enabled', '[default]WaterTreatment/Equipment/P301A/P301A_Lead', '[default]WaterTreatment/Equipment/P301B/P301B_Lead', '[default]WaterTreatment/Alarms/Any_Alarm', '[default]WaterTreatment/Alarms/Sequence_Faulted', '[default]WaterTreatment/Control/Batch/Batch_Quality_OK', '[default]WaterTreatment/Alarms/T101_LowLow', '[default]WaterTreatment/Alarms/T201_HighHigh', '[default]WaterTreatment/Alarms/T301_LowLow', '[default]WaterTreatment/Alarms/T301_HighHigh', '[default]WaterTreatment/Alarms/Concentration_HighHigh', '[default]WaterTreatment/Alarms/Pressure_Low', '[default]WaterTreatment/Alarms/Pressure_High', '[default]WaterTreatment/Alarms/P301A_FailedStart', '[default]WaterTreatment/Alarms/P301B_FailedStart', '[default]WaterTreatment/Alarms/P201_FailedStart', '[default]WaterTreatment/Alarms/XV201_FailedOpen', '[default]WaterTreatment/Alarms/Communication_Fault', '[default]WaterTreatment/Alarms/Both_Boosters_Unavailable', '[default]WaterTreatment/Alarms/Dosing_Timeout', '[default]WaterTreatment/Alarms/Fill_Timeout', '[default]WaterTreatment/Alarms/Transfer_Timeout', '[default]WaterTreatment/Alarms/AIT201_Invalid', '[default]WaterTreatment/Simulation/Fault_P301A_Trip', '[default]WaterTreatment/Simulation/Fault_AIT201_Freeze', '[default]WaterTreatment/Simulation/Fault_XV201_StuckClosed', '[default]WaterTreatment/Simulation/Fault_CommsLoss']
ALARMS = [('Alarms/Sequence_Faulted', 'Sequence Faulted', 'Critical', 'Batch Sequence', 'Automatic sequence faulted — identify and clear the first-out condition.'), ('Alarms/T101_LowLow', 'T101 Low Low', 'High', 'T-101', 'T-101 level low-low — verify raw-water supply before restart.'), ('Alarms/T201_HighHigh', 'T201 High High', 'High', 'T-201', 'T-201 level high-high — filling is inhibited.'), ('Alarms/T301_LowLow', 'T301 Low Low', 'Critical', 'T-301', 'T-301 level low-low — booster dry-run protection active.'), ('Alarms/T301_HighHigh', 'T301 High High', 'High', 'T-301', 'T-301 level high-high — treatment transfer is inhibited.'), ('Alarms/Concentration_HighHigh', 'Concentration High High', 'Critical', 'Treatment Quality', 'Treatment concentration high-high — dosing stopped and batch requires review.'), ('Alarms/Pressure_Low', 'Distribution Pressure Low', 'High', 'Distribution', 'Distribution pressure low — verify demand and booster availability.'), ('Alarms/Pressure_High', 'Distribution Pressure High', 'High', 'Distribution', 'Distribution pressure high — verify PI output and demand.'), ('Alarms/P301A_FailedStart', 'P301A Failed Start', 'Medium', 'P-301A', 'P-301A failed to start — standby takeover should be active.'), ('Alarms/P301B_FailedStart', 'P301B Failed Start', 'Medium', 'P-301B', 'P-301B failed to start — verify remaining distribution capacity.'), ('Alarms/P201_FailedStart', 'P201 Failed Start', 'High', 'P-201', 'P-201 failed to start — treatment transfer remains contained.'), ('Alarms/XV201_FailedOpen', 'XV201 Failed Open', 'High', 'XV-201', 'XV-201 failed to open — P-201 start is inhibited.'), ('Alarms/Communication_Fault', 'Plant Communication Fault', 'Critical', 'Communications', 'Simulink heartbeat lost — PLC outputs moved to the defined safe state.'), ('Alarms/Both_Boosters_Unavailable', 'Both Boosters Unavailable', 'Critical', 'Distribution', 'Both booster pumps unavailable — distribution capacity lost.'), ('Alarms/Dosing_Timeout', 'Dosing Timeout', 'High', 'Treatment Sequence', 'Dosing state timed out — inspect concentration response and dosing equipment.'), ('Alarms/Fill_Timeout', 'Fill Timeout', 'High', 'Treatment Sequence', 'Fill state timed out — inspect raw-water pumps and T-201 level response.'), ('Alarms/Transfer_Timeout', 'Transfer Timeout', 'High', 'Treatment Sequence', 'Transfer state timed out — inspect P-201, XV-201, and tank levels.'), ('Alarms/AIT201_Invalid', 'AIT201 Invalid', 'Critical', 'AIT-201', 'AIT-201 signal invalid or frozen — automatic concentration control disabled.')]
ANALOG_HISTORY = {'Plant/T101/LIT101_PV': 0.1, 'Plant/T201/LIT201_PV': 0.1, 'Plant/T301/LIT301_PV': 0.1, 'Plant/Flows/FIT101_PV': 0.2, 'Plant/Flows/FIT201_PV': 0.2, 'Plant/Distribution/FIT301_PV': 0.2, 'Plant/T201/AIT201_PV': 0.01, 'Plant/Distribution/PIT301_PV': 0.5, 'Equipment/P101A/P101A_SpeedCmd': 0.5, 'Equipment/P101B/P101B_SpeedCmd': 0.5, 'Equipment/P201/P201_SpeedCmd': 0.5, 'Equipment/P301A/P301A_SpeedCmd': 0.5, 'Equipment/P301B/P301B_SpeedCmd': 0.5, 'Equipment/DP201/DP201_Output': 0.5, 'Control/PressurePI/Pressure_SP': 0.1, 'Control/ConcentrationPI/Concentration_SP': 0.01, 'Control/Batch/BatchFill_SP': 0.1, 'Control/Batch/MixTime_SP': 1.0, 'Equipment/DP201/Manual_Dose_Output': 0.5, 'Control/PressurePI/Pressure_PI_Output': 0.5, 'Control/ConcentrationPI/Concentration_PI_Output': 0.5}
DISCRETE_HISTORY = ['Control/Batch/Batch_State', 'Control/Batch/Batch_Count', 'Control/Batch/Rejected_Batch_Count', 'Control/Batch/Batch_Accepted', 'Control/Batch/Batch_Quality_OK', 'Control/Modes/Active_Mode', 'Control/Modes/Requested_Mode', 'Control/Modes/Lead_Booster', 'Diagnostics/Comm_Healthy', 'Diagnostics/System_Running', 'Equipment/P101A/P101A_RunFb', 'Equipment/P101B/P101B_RunFb', 'Equipment/P201/P201_RunFb', 'Equipment/P301A/P301A_RunFb', 'Equipment/P301B/P301B_RunFb', 'Equipment/M201/M201_RunFb', 'Equipment/XV201/XV201_OpenFb', 'Equipment/XV201/XV201_ClosedFb', 'Alarms/Sequence_Faulted', 'Alarms/T101_LowLow', 'Alarms/T201_HighHigh', 'Alarms/T301_LowLow', 'Alarms/T301_HighHigh', 'Alarms/Concentration_HighHigh', 'Alarms/Pressure_Low', 'Alarms/Pressure_High', 'Alarms/P301A_FailedStart', 'Alarms/P301B_FailedStart', 'Alarms/P201_FailedStart', 'Alarms/XV201_FailedOpen', 'Alarms/Communication_Fault', 'Alarms/Both_Boosters_Unavailable', 'Alarms/Dosing_Timeout', 'Alarms/Fill_Timeout', 'Alarms/Transfer_Timeout', 'Alarms/AIT201_Invalid']


def _path(relative_path):
    return ROOT + relative_path


def _require(relative_path, allowed, action):
    if relative_path not in allowed:
        raise ValueError(action + " is not permitted for " + str(relative_path))
    full_path = _path(relative_path)
    if not system.tag.exists(full_path):
        raise ValueError("Required tag does not exist: " + full_path)
    return full_path


def _require_good(quality_code, action):
    if not quality_code.isGood():
        raise IOError(action + " failed: " + str(quality_code))


def pulse(relative_path, hold_ms=300):
    """pulse a plc edge-detected operator request and always restore false"""
    full_path = _require(relative_path, PULSE_REQUESTS, "Pulse")
    first = system.tag.writeBlocking([full_path], [True])[0]
    _require_good(first, "Writing True to " + full_path)
    try:
        Thread.sleep(int(hold_ms))
    finally:
        second = system.tag.writeBlocking([full_path], [False])[0]
        _require_good(second, "Restoring False to " + full_path)
    return True


def toggle_request(relative_path):
    """toggle a sustained manual request; plc interlocks remain authoritative"""
    full_path = _require(relative_path, SUSTAINED_REQUESTS, "Manual request")
    current = system.tag.readBlocking([full_path])[0]
    _require_good(current.quality, "Reading " + full_path)
    result = system.tag.writeBlocking([full_path], [not bool(current.value)])[0]
    _require_good(result, "Writing " + full_path)
    return not bool(current.value)


def toggle_fault(relative_path):
    """toggle only one of the four documented simulation fault requests"""
    full_path = _require(relative_path, FAULT_REQUESTS, "Fault injection")
    current = system.tag.readBlocking([full_path])[0]
    _require_good(current.quality, "Reading " + full_path)
    result = system.tag.writeBlocking([full_path], [not bool(current.value)])[0]
    _require_good(result, "Writing " + full_path)
    return not bool(current.value)


def reset_faults():
    paths = [_path(item) for item in sorted(FAULT_REQUESTS)]
    results = system.tag.writeBlocking(paths, [False] * len(paths))
    for index, result in enumerate(results):
        _require_good(result, "Clearing " + paths[index])
    return True


def clear_requests():
    """clear all hmi request bits without writing any plc-owned actuator output"""
    relatives = sorted(PULSE_REQUESTS | SUSTAINED_REQUESTS)
    paths = [_path(item) for item in relatives]
    results = system.tag.writeBlocking(paths, [False] * len(paths))
    for index, result in enumerate(results):
        _require_good(result, "Clearing " + paths[index])
    return True


def adjust_setpoint(relative_path, direction):
    """move a writable setpoint by one documented step and clamp to plc limits"""
    if relative_path not in SETPOINTS:
        raise ValueError("Setpoint write is not permitted for " + str(relative_path))
    full_path = _path(relative_path)
    current = system.tag.readBlocking([full_path])[0]
    _require_good(current.quality, "Reading " + full_path)
    low, high, step = SETPOINTS[relative_path]
    requested = float(current.value) + (step * (1 if int(direction) >= 0 else -1))
    clamped = min(high, max(low, requested))
    result = system.tag.writeBlocking([full_path], [clamped])[0]
    _require_good(result, "Writing " + full_path)
    return clamped


def _merge_tag(relative_path, properties):
    full_path = _path(relative_path)
    parent, name = full_path.rsplit("/", 1)
    definition = dict(properties)
    definition["name"] = name
    result = system.tag.configure(parent, [definition], "m")[0]
    _require_good(result, "Configuring " + full_path)


def configure_alarms():
    """apply 18 plc-derived boolean alarms with operator-facing messages"""
    for relative_path, name, priority, display_path, message in ALARMS:
        alarm = {
            "name": name,
            "enabled": True,
            "priority": priority,
            "mode": "WhenTrue",
            "ackMode": "Manual",
            "timestampSource": "Value",
            "displayPath": display_path,
            "label": message,
            "notes": message,
            "shelvingAllowed": True,
        }
        _merge_tag(relative_path, {"alarmEvalEnabled": True, "alarms": [alarm]})
    return len(ALARMS)


def configure_history(history_provider="WaterTreatmentHistory"):
    """enable bounded on-change history for required analog and discrete tags"""
    if not history_provider:
        raise ValueError("A historian provider name is required")
    common = {
        "historyEnabled": True,
        "historyProvider": history_provider,
        "sampleMode": "OnChange",
        "historyTimeDeadband": 1,
        "historyTimeDeadbandUnits": "SEC",
        "historyMaxAge": 10,
        "historyMaxAgeUnits": "SEC",
    }
    for relative_path, deadband in ANALOG_HISTORY.items():
        props = dict(common)
        props.update({
            "historicalDeadbandStyle": "Analog_Compressed",
            "historicalDeadbandMode": "Absolute",
            "historicalDeadband": float(deadband),
        })
        _merge_tag(relative_path, props)
    for relative_path in DISCRETE_HISTORY:
        props = dict(common)
        props.update({
            "historicalDeadbandStyle": "Discrete",
            "historicalDeadbandMode": "Off",
            "historicalDeadband": 0.0,
            "historyMaxAge": 30,
        })
        _merge_tag(relative_path, props)
    return len(ANALOG_HISTORY) + len(DISCRETE_HISTORY)


def validate():
    """read every imported v0.5.2 opc tag and report existence/quality"""
    missing = [path for path in ALL_TAGS if not system.tag.exists(path)]
    existing = [path for path in ALL_TAGS if path not in missing]
    values = system.tag.readBlocking(existing) if existing else []
    bad = []
    for index, qualified_value in enumerate(values):
        if not qualified_value.quality.isGood():
            bad.append(existing[index] + " = " + str(qualified_value.quality))
    result = {
        "expected": len(ALL_TAGS),
        "existing": len(existing),
        "good": len(existing) - len(bad),
        "missing": missing,
        "badQuality": bad,
    }
    logger = system.util.getLogger("WaterTreatmentSCADA")
    logger.info("SCADA validation: " + str(result))
    print("SCADA validation: " + str(result))
    return result


def run(history_provider="WaterTreatmentHistory"):
    """run once after project import and gateway historian/journal creation"""
    preflight = validate()
    if preflight["missing"]:
        raise ValueError("Tag setup stopped because required tags are missing")
    alarm_count = configure_alarms()
    history_count = configure_history(history_provider)
    clear_requests()
    reset_faults()
    final_result = validate()
    final_result["alarmsConfigured"] = alarm_count
    final_result["historyTagsConfigured"] = history_count
    final_result["historyProvider"] = history_provider
    final_result["alarmJournalExpected"] = "WaterTreatmentJournal"
    print("SCADA one-time setup complete: " + str(final_result))
    return final_result
