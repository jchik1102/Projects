function M = derive_plant_models(P, printSummary)
%DERIVE_PLANT_MODELS Build analytical and LTI models using Control System Toolbox.

if nargin < 1 || isempty(P)
    P = initialize_water_plant(false);
end
if nargin < 2
    printSummary = true;
end

s = tf('s');

% Tank timing calculations
M.T101.fillTimeFromEmpty_s = P.T101.maxVolume_m3*1000/P.P101A.maxFlow_Lps;
M.T201.fillTimeFromEmpty_s = P.T201.maxVolume_m3*1000/P.P101A.maxFlow_Lps;
M.T201.timeToBatchSetpointFromInitial_s = ...
    max(0, (P.T201.batchFillSetpoint_pct-P.T201.initialLevel_pct)/100 * ...
    P.T201.maxVolume_m3*1000/P.P101A.maxFlow_Lps);
M.T301.fillTimeFromEmpty_s = P.T301.maxVolume_m3*1000/P.P201.maxFlow_Lps;

% Pump/valve LTI models used for engineering checks
M.actuator.P101 = 1/(P.P101A.timeConstant_s*s + 1);
M.actuator.P201 = 1/(P.P201.timeConstant_s*s + 1);
M.actuator.P301 = 1/(P.P301A.timeConstant_s*s + 1);
M.actuator.XV201 = 1/(P.XV201.travelTimeConstant_s*s + 1);

% Distribution pressure models in deviation variables
M.pressure.pump = P.pressure.pumpGain_kPa_per_pct / ...
    (P.pressure.timeConstant_s*s + 1);
M.pressure.demand = -P.pressure.demandGain_kPa_per_Lps / ...
    (P.pressure.timeConstant_s*s + 1);
M.pressure.sensor = 1/(P.pressure.sensorTimeConstant_s*s + 1);
M.pressure.loopPlant = minreal(M.pressure.pump*M.pressure.sensor);
M.pressure.nominalTotalSpeed_pct = ...
    (P.pressure.setpoint_kPa + ...
    P.pressure.demandGain_kPa_per_Lps*P.pressure.nominalDemand_Lps) / ...
    P.pressure.pumpGain_kPa_per_pct;
M.pressure.highDemandTotalSpeed_pct = ...
    (P.pressure.setpoint_kPa + ...
    P.pressure.demandGain_kPa_per_Lps*P.pressure.highDemand_Lps) / ...
    P.pressure.pumpGain_kPa_per_pct;

% Reduced-order concentration model for controller design
M.concentration.plant = P.controller.concentration.approxGain_mgL_per_pct / ...
    (P.controller.concentration.approxTimeConstant_s*s + 1);
M.concentration.sensor = 1/(P.concentration.sensorTimeConstant_s*s + 1);
M.concentration.loopPlant = minreal(M.concentration.plant*M.concentration.sensor);

batchVolume_L = P.T201.maxVolume_m3*P.T201.batchFillSetpoint_pct/100*1000;
M.concentration.targetChemicalMass_mg = ...
    P.concentration.setpoint_mgL*batchVolume_L;
M.concentration.fullOutputDoseTime_s = ...
    M.concentration.targetChemicalMass_mg/P.DP201.maxChemicalMassRate_mg_s;

if printSummary
    fprintf('\nAnalytical and LTI plant models created.\n');
    fprintf('  T-201 fill time to %.1f%%: %.1f s\n', ...
        P.T201.batchFillSetpoint_pct, M.T201.timeToBatchSetpointFromInitial_s);
    fprintf('  Nominal booster command: %.2f%% total speed\n', ...
        M.pressure.nominalTotalSpeed_pct);
    fprintf('  High-demand booster command: %.2f%% total speed\n', ...
        M.pressure.highDemandTotalSpeed_pct);
    fprintf('  Ideal full-output dose time: %.1f s\n', ...
        M.concentration.fullOutputDoseTime_s);
end
end
