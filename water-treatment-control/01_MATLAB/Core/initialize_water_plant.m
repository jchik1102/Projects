function P = initialize_water_plant(printSummary)
%INITIALIZE_WATER_PLANT Define the source-of-truth plant/design parameters.
%
% All values used by the MATLAB calculations and generated Simulink models
% originate here. Change this file first when revising the plant.

if nargin < 1
    printSummary = true;
end

P.project.name = 'Automated Water Pumping and Treatment Station';
P.project.revision = '0.5.0';
P.project.date = '2026-08-02';
P.project.architecture = ...
    'Simulink plant; OpenPLC controller; Ignition supervisor; MATLAB engineering/validation';

% Simulation and timing
P.sim.baseStep_s = 0.02;
P.sim.communicationPeriod_s = 0.10;
P.sim.defaultStopTime_s = 180;
P.sim.maxPressure_kPa = 800;
P.sim.maxConcentration_mgL = 5.0;
P.sim.solver = 'ode4';

% Tanks
P.T101.tag = 'T-101';
P.T101.maxVolume_m3 = 100;
P.T101.height_m = 5.0;
P.T101.area_m2 = P.T101.maxVolume_m3/P.T101.height_m;
P.T101.initialLevel_pct = 70;
P.T101.initialVolume_m3 = P.T101.maxVolume_m3*P.T101.initialLevel_pct/100;

P.T201.tag = 'T-201';
P.T201.maxVolume_m3 = 60;
P.T201.height_m = 4.0;
P.T201.area_m2 = P.T201.maxVolume_m3/P.T201.height_m;
P.T201.initialLevel_pct = 10;
P.T201.initialVolume_m3 = P.T201.maxVolume_m3*P.T201.initialLevel_pct/100;
P.T201.batchFillSetpoint_pct = 80;
P.T201.transferCompleteLevel_pct = 10;
P.T201.minimumDosingLevel_pct = 20;
P.T201.initialConcentration_mgL = 0.0;

P.T301.tag = 'T-301';
P.T301.maxVolume_m3 = 125;
P.T301.height_m = 5.0;
P.T301.area_m2 = P.T301.maxVolume_m3/P.T301.height_m;
P.T301.initialLevel_pct = 70;
P.T301.initialVolume_m3 = P.T301.maxVolume_m3*P.T301.initialLevel_pct/100;

P.tank.minimumSourceLevel_pct = 0.5;

% Pumps and valve
P.P101A.maxFlow_Lps = 35;
P.P101A.timeConstant_s = 1.5;
P.P101B = P.P101A;
P.P201.maxFlow_Lps = 40;
P.P201.timeConstant_s = 1.5;
P.P301A.maxFlow_Lps = 45;
P.P301A.timeConstant_s = 1.0;
P.P301B = P.P301A;
P.pump.runFeedbackThreshold_pct = 5;

P.XV201.travelTimeConstant_s = 2.0;
P.XV201.openFeedbackThreshold = 0.95;
P.XV201.closedFeedbackThreshold = 0.05;

% Pressure process: P = Kpump*u - Kdemand*d through first-order dynamics
P.pressure.setpoint_kPa = 400;
P.pressure.initial_kPa = 400;
P.pressure.pumpGain_kPa_per_pct = 6.0;
P.pressure.demandGain_kPa_per_Lps = 5.0;
P.pressure.timeConstant_s = 3.0;
P.pressure.sensorTimeConstant_s = 0.50;
P.pressure.sensorBias_kPa = 0;
P.pressure.nominalDemand_Lps = 30;
P.pressure.highDemand_Lps = 60;
P.pressure.minimumAllowed_kPa = 320;
P.pressure.acceptanceBand_kPa = 10;
P.pressure.recoveryTimeLimit_s = 15;

% Concentration process
P.concentration.setpoint_mgL = 1.20;
P.concentration.acceptLow_mgL = 1.10;
P.concentration.acceptHigh_mgL = 1.30;
P.concentration.highHigh_mgL = 1.50;
P.concentration.stableTime_s = 20;
P.concentration.sensorTimeConstant_s = 3.0;
P.concentration.sensorBias_mgL = 0;
P.concentration.minimumMixerEffectiveness = 0.20;
P.concentration.maxPhysical_mgL = 5.0;
P.DP201.maxChemicalMassRate_mg_s = 900;
P.DP201.timeConstant_s = 0.8;
P.M201.mixTime_s = 120;

% Control System Toolbox design settings
P.controller.pressure.sampleTime_s = 0.10;
P.controller.pressure.targetCrossover_rad_s = 0.45;
P.controller.pressure.minimumPhaseMargin_deg = 45;
P.controller.pressure.maximumOvershoot_pct = 15;
P.controller.pressure.maximumReferenceSettling_s = 15;
P.controller.pressure.outputMin_pct = 0;
P.controller.pressure.outputMax_pct = 200;
P.controller.pressure.antiWindup = 'clamping';

P.controller.concentration.approxGain_mgL_per_pct = 0.015;
P.controller.concentration.approxTimeConstant_s = 20;
P.controller.concentration.sampleTime_s = 0.50;
P.controller.concentration.targetCrossover_rad_s = 0.08;
P.controller.concentration.minimumPhaseMargin_deg = 45;
P.controller.concentration.maximumOvershoot_pct = 15;
P.controller.concentration.maximumSettling_s = 120;
P.controller.concentration.outputMin_pct = 0;
P.controller.concentration.outputMax_pct = 100;
P.controller.concentration.antiWindup = 'clamping';
% Final OpenPLC gains validated against the nonlinear chemical mass-balance
% plant with output saturation, actuator lag, sensor lag, and a 0.5 s PI
% execution period. The pidtune gains above remain the Stage 2 reduced-order
% candidate; these commissioned gains are the final Stage 5 implementation.
P.controller.concentration.openplcKp = 100.0;
P.controller.concentration.openplcKi_per_s = 0.20;
P.controller.concentration.openplcKiTimesTs = 0.10;

% Alarm thresholds reserved for OpenPLC implementation
P.alarm.T101_lowLow_pct = 5;
P.alarm.T201_highHigh_pct = 95;
P.alarm.T301_lowLow_pct = 5;
P.alarm.T301_highHigh_pct = 95;
P.alarm.pressureLow_kPa = 320;
P.alarm.pressureHigh_kPa = 500;

% Numerical test tolerances
P.test.fillVolumeTolerance_m3 = 0.012;
P.test.massBalanceTolerance_m3 = 0.015;
P.test.levelNumericalTolerance_pct = 1e-6;
P.test.valveClosedFlowTolerance_Lps = 0.10;
P.test.concentrationTolerance_mgL = 0.05;
P.test.sensorLagMinimum_s = 0.05;

if printSummary
    fprintf('Plant parameters loaded (revision %s).\n', P.project.revision);
    fprintf('  Tanks: %.0f / %.0f / %.0f m^3\n', ...
        P.T101.maxVolume_m3, P.T201.maxVolume_m3, P.T301.maxVolume_m3);
    fprintf('  Pressure target: %.1f kPa\n', P.pressure.setpoint_kPa);
    fprintf('  Concentration target: %.2f mg/L (%.2f to %.2f acceptable)\n', ...
        P.concentration.setpoint_mgL, P.concentration.acceptLow_mgL, ...
        P.concentration.acceptHigh_mgL);
end
end
