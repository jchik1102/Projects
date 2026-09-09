function modelFile = build_water_treatment_plant(forceRebuild)
% generate the standalone open-loop simulink plant model

if nargin < 1
    forceRebuild = false;
end

matlabDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(fileparts(matlabDir));
modelDir = fullfile(projectRoot, '02_SIMULINK');
modelName = 'Water_Treatment_Plant';
modelFile = fullfile(modelDir, [modelName '.slx']);

if ~exist(modelDir, 'dir')
    mkdir(modelDir);
end

if exist(modelFile, 'file') && ~forceRebuild
    load_system(modelFile);
    open_system(modelName);
    fprintf('Opened existing model: %s\n', modelFile);
    return;
end

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

load_system('simulink');
new_system(modelName);

set_param(modelName, ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'ode4', ...
    'FixedStep', 'P.sim.baseStep_s', ...
    'StopTime', 'P.sim.defaultStopTime_s', ...
    'ReturnWorkspaceOutputs', 'on', ...
    'SaveTime', 'on', ...
    'TimeSaveName', 'tout', ...
    'SignalLogging', 'off');

if evalin('base', 'exist(''P'', ''var'')') == 0
    assignin('base', 'P', initialize_water_plant(false));
end
create_default_inputs(120, true);

sourceNames = { ...
    'raw_inflow', 'demand_flow', 'cmd_P101A', 'cmd_P101B', 'cmd_P201', ...
    'cmd_P301A', 'cmd_P301B', 'cmd_DP201', 'cmd_M201', 'cmd_XV201', ...
    'avail_P101A', 'avail_P101B', 'avail_P201', 'avail_P301A', 'avail_P301B'};

for k = 1:numel(sourceNames)
    y = 30 + (k-1)*55;
    addFromWorkspace(modelName, sourceNames{k}, sourceNames{k}, [25 y 150 y+25]);
end

addTankSubsystem([modelName '/Tank_T101'], ...
    'P.T101.initialVolume_m3', 'P.T101.maxVolume_m3', [360 70 575 210]);
addTankSubsystem([modelName '/Tank_T201'], ...
    'P.T201.initialVolume_m3', 'P.T201.maxVolume_m3', [760 250 975 390]);
addTankSubsystem([modelName '/Tank_T301'], ...
    'P.T301.initialVolume_m3', 'P.T301.maxVolume_m3', [1160 430 1375 570]);

addPumpSubsystem([modelName '/P101A_Actuator'], ...
    'P.P101A.maxFlow_Lps', 'P.P101A.timeConstant_s', [360 270 575 380]);
addPumpSubsystem([modelName '/P101B_Actuator'], ...
    'P.P101B.maxFlow_Lps', 'P.P101B.timeConstant_s', [360 415 575 525]);
addPumpSubsystem([modelName '/P201_Actuator'], ...
    'P.P201.maxFlow_Lps', 'P.P201.timeConstant_s', [760 500 975 610]);
addPumpSubsystem([modelName '/P301A_Actuator'], ...
    'P.P301A.maxFlow_Lps', 'P.P301A.timeConstant_s', [1160 80 1375 190]);
addPumpSubsystem([modelName '/P301B_Actuator'], ...
    'P.P301B.maxFlow_Lps', 'P.P301B.timeConstant_s', [1160 230 1375 340]);

addValveSubsystem([modelName '/XV201_Valve'], [760 650 975 760]);
addConcentrationSubsystem([modelName '/Treatment_Concentration'], [1015 250 1245 395]);
addPressureSubsystem([modelName '/Distribution_Pressure'], [1445 120 1695 300]);

add_block('simulink/Math Operations/Sum', [modelName '/P101_TotalFlow'], ...
    'Inputs', '++', 'Position', [640 330 670 380]);
add_block('simulink/Math Operations/Product', [modelName '/P201_ValveFlow'], ...
    'Inputs', '**', 'Position', [1025 535 1060 575]);
addAvailabilityLogic(modelName, 'T101_SourceAvailable', [610 105 735 155]);
addAvailabilityLogic(modelName, 'T201_SourceAvailable', [1015 445 1140 495]);
addAvailabilityLogic(modelName, 'T301_SourceAvailable', [1410 475 1535 525]);

add_block('simulink/Math Operations/Product', [modelName '/P101A_Enabled'], ...
    'Inputs', '**', 'Position', [250 285 285 325]);
add_block('simulink/Math Operations/Product', [modelName '/P101B_Enabled'], ...
    'Inputs', '**', 'Position', [250 430 285 470]);
add_block('simulink/Math Operations/Product', [modelName '/P201_Enabled'], ...
    'Inputs', '**', 'Position', [650 520 685 560]);
add_block('simulink/Math Operations/Product', [modelName '/P301A_Enabled'], ...
    'Inputs', '**', 'Position', [1050 95 1085 135]);
add_block('simulink/Math Operations/Product', [modelName '/P301B_Enabled'], ...
    'Inputs', '**', 'Position', [1050 245 1085 285]);
add_block('simulink/Math Operations/Product', [modelName '/DeliveredDemand'], ...
    'Inputs', '**', 'Position', [1550 475 1585 515]);

add_line(modelName, 'raw_inflow/1', 'Tank_T101/1', 'autorouting', 'on');
add_line(modelName, 'Tank_T101/2', 'T101_SourceAvailable/1', 'autorouting', 'on');
add_line(modelName, 'avail_P101A/1', 'P101A_Enabled/1', 'autorouting', 'on');
add_line(modelName, 'T101_SourceAvailable/1', 'P101A_Enabled/2', 'autorouting', 'on');
add_line(modelName, 'avail_P101B/1', 'P101B_Enabled/1', 'autorouting', 'on');
add_line(modelName, 'T101_SourceAvailable/1', 'P101B_Enabled/2', 'autorouting', 'on');
add_line(modelName, 'cmd_P101A/1', 'P101A_Actuator/1', 'autorouting', 'on');
add_line(modelName, 'P101A_Enabled/1', 'P101A_Actuator/2', 'autorouting', 'on');
add_line(modelName, 'cmd_P101B/1', 'P101B_Actuator/1', 'autorouting', 'on');
add_line(modelName, 'P101B_Enabled/1', 'P101B_Actuator/2', 'autorouting', 'on');
add_line(modelName, 'P101A_Actuator/2', 'P101_TotalFlow/1', 'autorouting', 'on');
add_line(modelName, 'P101B_Actuator/2', 'P101_TotalFlow/2', 'autorouting', 'on');
add_line(modelName, 'P101_TotalFlow/1', 'Tank_T101/2', 'autorouting', 'on');
add_line(modelName, 'P101_TotalFlow/1', 'Tank_T201/1', 'autorouting', 'on');

add_line(modelName, 'Tank_T201/2', 'T201_SourceAvailable/1', 'autorouting', 'on');
add_line(modelName, 'avail_P201/1', 'P201_Enabled/1', 'autorouting', 'on');
add_line(modelName, 'T201_SourceAvailable/1', 'P201_Enabled/2', 'autorouting', 'on');
add_line(modelName, 'cmd_P201/1', 'P201_Actuator/1', 'autorouting', 'on');
add_line(modelName, 'P201_Enabled/1', 'P201_Actuator/2', 'autorouting', 'on');
add_line(modelName, 'cmd_XV201/1', 'XV201_Valve/1', 'autorouting', 'on');
add_line(modelName, 'P201_Actuator/2', 'P201_ValveFlow/1', 'autorouting', 'on');
add_line(modelName, 'XV201_Valve/1', 'P201_ValveFlow/2', 'autorouting', 'on');
add_line(modelName, 'P201_ValveFlow/1', 'Tank_T201/2', 'autorouting', 'on');
add_line(modelName, 'P201_ValveFlow/1', 'Tank_T301/1', 'autorouting', 'on');

add_line(modelName, 'cmd_DP201/1', 'Treatment_Concentration/1', 'autorouting', 'on');
add_line(modelName, 'cmd_M201/1', 'Treatment_Concentration/2', 'autorouting', 'on');
add_line(modelName, 'Tank_T201/1', 'Treatment_Concentration/3', 'autorouting', 'on');
add_line(modelName, 'P201_ValveFlow/1', 'Treatment_Concentration/4', 'autorouting', 'on');

add_line(modelName, 'Tank_T301/2', 'T301_SourceAvailable/1', 'autorouting', 'on');
add_line(modelName, 'demand_flow/1', 'DeliveredDemand/1', 'autorouting', 'on');
add_line(modelName, 'T301_SourceAvailable/1', 'DeliveredDemand/2', 'autorouting', 'on');
add_line(modelName, 'DeliveredDemand/1', 'Tank_T301/2', 'autorouting', 'on');

add_line(modelName, 'avail_P301A/1', 'P301A_Enabled/1', 'autorouting', 'on');
add_line(modelName, 'T301_SourceAvailable/1', 'P301A_Enabled/2', 'autorouting', 'on');
add_line(modelName, 'avail_P301B/1', 'P301B_Enabled/1', 'autorouting', 'on');
add_line(modelName, 'T301_SourceAvailable/1', 'P301B_Enabled/2', 'autorouting', 'on');
add_line(modelName, 'cmd_P301A/1', 'P301A_Actuator/1', 'autorouting', 'on');
add_line(modelName, 'P301A_Enabled/1', 'P301A_Actuator/2', 'autorouting', 'on');
add_line(modelName, 'cmd_P301B/1', 'P301B_Actuator/1', 'autorouting', 'on');
add_line(modelName, 'P301B_Enabled/1', 'P301B_Actuator/2', 'autorouting', 'on');
add_line(modelName, 'P301A_Actuator/1', 'Distribution_Pressure/1', 'autorouting', 'on');
add_line(modelName, 'P301B_Actuator/1', 'Distribution_Pressure/2', 'autorouting', 'on');
add_line(modelName, 'demand_flow/1', 'Distribution_Pressure/3', 'autorouting', 'on');


addTerminator(modelName, 'Term_P101A_RunFb', [620 390 640 410]);
addTerminator(modelName, 'Term_P101B_Speed', [620 435 640 455]);
addTerminator(modelName, 'Term_P101B_RunFb', [620 480 640 500]);
addTerminator(modelName, 'Term_P201_Speed', [1010 590 1030 610]);
addTerminator(modelName, 'Term_P201_RunFb', [1010 615 1030 635]);
addTerminator(modelName, 'Term_P301A_Flow', [1405 90 1425 110]);
addTerminator(modelName, 'Term_P301A_RunFb', [1405 120 1425 140]);
addTerminator(modelName, 'Term_P301B_Flow', [1405 260 1425 280]);
addTerminator(modelName, 'Term_P301B_RunFb', [1405 290 1425 310]);
addTerminator(modelName, 'Term_XV201_OpenFb', [1010 700 1030 720]);
addTerminator(modelName, 'Term_XV201_ClosedFb', [1010 730 1030 750]);

add_line(modelName, 'P101A_Actuator/3', 'Term_P101A_RunFb/1', 'autorouting', 'on');
add_line(modelName, 'P101B_Actuator/1', 'Term_P101B_Speed/1', 'autorouting', 'on');
add_line(modelName, 'P101B_Actuator/3', 'Term_P101B_RunFb/1', 'autorouting', 'on');
add_line(modelName, 'P201_Actuator/1', 'Term_P201_Speed/1', 'autorouting', 'on');
add_line(modelName, 'P201_Actuator/3', 'Term_P201_RunFb/1', 'autorouting', 'on');
add_line(modelName, 'P301A_Actuator/2', 'Term_P301A_Flow/1', 'autorouting', 'on');
add_line(modelName, 'P301A_Actuator/3', 'Term_P301A_RunFb/1', 'autorouting', 'on');
add_line(modelName, 'P301B_Actuator/2', 'Term_P301B_Flow/1', 'autorouting', 'on');
add_line(modelName, 'P301B_Actuator/3', 'Term_P301B_RunFb/1', 'autorouting', 'on');
add_line(modelName, 'XV201_Valve/2', 'Term_XV201_OpenFb/1', 'autorouting', 'on');
add_line(modelName, 'XV201_Valve/3', 'Term_XV201_ClosedFb/1', 'autorouting', 'on');

addToWorkspace(modelName, 'Log_T101_Volume', 'sim_T101_Volume_m3', [1790 340 1945 365]);
addToWorkspace(modelName, 'Log_T101_Level', 'sim_T101_Level_pct', [1790 375 1945 400]);
addToWorkspace(modelName, 'Log_T201_Volume', 'sim_T201_Volume_m3', [1790 410 1945 435]);
addToWorkspace(modelName, 'Log_T201_Level', 'sim_T201_Level_pct', [1790 445 1945 470]);
addToWorkspace(modelName, 'Log_T301_Volume', 'sim_T301_Volume_m3', [1790 480 1945 505]);
addToWorkspace(modelName, 'Log_T301_Level', 'sim_T301_Level_pct', [1790 515 1945 540]);
addToWorkspace(modelName, 'Log_P101A_Speed', 'sim_P101A_Speed_pct', [1790 550 1945 575]);
addToWorkspace(modelName, 'Log_P101A_Flow', 'sim_P101A_Flow_Lps', [1790 585 1945 610]);
addToWorkspace(modelName, 'Log_P201_Flow', 'sim_P201_Flow_Lps', [1790 620 1945 645]);
addToWorkspace(modelName, 'Log_XV201_Position', 'sim_XV201_Position', [1790 655 1945 680]);
addToWorkspace(modelName, 'Log_Pressure_Actual', 'sim_Pressure_Actual_kPa', [1790 690 1945 715]);
addToWorkspace(modelName, 'Log_Pressure_PV', 'sim_Pressure_PV_kPa', [1790 725 1945 750]);
addToWorkspace(modelName, 'Log_Concentration_Actual', 'sim_Concentration_Actual_mgL', [1790 760 1945 785]);
addToWorkspace(modelName, 'Log_Concentration_PV', 'sim_Concentration_PV_mgL', [1790 795 1945 820]);
addToWorkspace(modelName, 'Log_DoseSpeed', 'sim_DP201_Speed_pct', [1790 830 1945 855]);
addToWorkspace(modelName, 'Log_DeliveredDemand', 'sim_DeliveredDemand_Lps', [1790 865 1945 890]);
addToWorkspace(modelName, 'Log_P301A_Speed', 'sim_P301A_Speed_pct', [1790 900 1945 925]);
addToWorkspace(modelName, 'Log_P301B_Speed', 'sim_P301B_Speed_pct', [1790 935 1945 960]);

add_line(modelName, 'Tank_T101/1', 'Log_T101_Volume/1', 'autorouting', 'on');
add_line(modelName, 'Tank_T101/2', 'Log_T101_Level/1', 'autorouting', 'on');
add_line(modelName, 'Tank_T201/1', 'Log_T201_Volume/1', 'autorouting', 'on');
add_line(modelName, 'Tank_T201/2', 'Log_T201_Level/1', 'autorouting', 'on');
add_line(modelName, 'Tank_T301/1', 'Log_T301_Volume/1', 'autorouting', 'on');
add_line(modelName, 'Tank_T301/2', 'Log_T301_Level/1', 'autorouting', 'on');
add_line(modelName, 'P101A_Actuator/1', 'Log_P101A_Speed/1', 'autorouting', 'on');
add_line(modelName, 'P101A_Actuator/2', 'Log_P101A_Flow/1', 'autorouting', 'on');
add_line(modelName, 'P201_ValveFlow/1', 'Log_P201_Flow/1', 'autorouting', 'on');
add_line(modelName, 'XV201_Valve/1', 'Log_XV201_Position/1', 'autorouting', 'on');
add_line(modelName, 'Distribution_Pressure/1', 'Log_Pressure_Actual/1', 'autorouting', 'on');
add_line(modelName, 'Distribution_Pressure/2', 'Log_Pressure_PV/1', 'autorouting', 'on');
add_line(modelName, 'Treatment_Concentration/1', 'Log_Concentration_Actual/1', 'autorouting', 'on');
add_line(modelName, 'Treatment_Concentration/2', 'Log_Concentration_PV/1', 'autorouting', 'on');
add_line(modelName, 'Treatment_Concentration/3', 'Log_DoseSpeed/1', 'autorouting', 'on');
add_line(modelName, 'DeliveredDemand/1', 'Log_DeliveredDemand/1', 'autorouting', 'on');
add_line(modelName, 'P301A_Actuator/1', 'Log_P301A_Speed/1', 'autorouting', 'on');
add_line(modelName, 'P301B_Actuator/1', 'Log_P301B_Speed/1', 'autorouting', 'on');

Simulink.Annotation(modelName, ...
    'Standalone physical plant. Final sequencing and PI execution belong in OpenPLC.');

save_system(modelName, modelFile);
open_system(modelName);
fprintf('Generated model: %s\n', modelFile);
end

function addFromWorkspace(model, blockName, variableName, position)
add_block('simulink/Sources/From Workspace', [model '/' blockName], ...
    'VariableName', variableName, ...
    'Interpolate', 'off', ...
    'OutputAfterFinalValue', 'Holding final value', ...
    'Position', position);
end

function addToWorkspace(model, blockName, variableName, position)
add_block('simulink/Sinks/To Workspace', [model '/' blockName], ...
    'VariableName', variableName, ...
    'SaveFormat', 'Timeseries', ...
    'MaxDataPoints', 'inf', ...
    'Position', position);
end


function addTerminator(model, blockName, position)
add_block('simulink/Sinks/Terminator', [model '/' blockName], ...
    'Position', position);
end

function addTankSubsystem(path, initialVolumeExpr, maxVolumeExpr, position)
add_block('simulink/Ports & Subsystems/Subsystem', path, 'Position', position);
Simulink.SubSystem.deleteContents(path);

add_block('simulink/Ports & Subsystems/In1', [path '/Inflow_Lps'], ...
    'Port', '1', 'Position', [25 40 55 60]);
add_block('simulink/Ports & Subsystems/In1', [path '/Outflow_Lps'], ...
    'Port', '2', 'Position', [25 95 55 115]);
add_block('simulink/Math Operations/Sum', [path '/NetFlow'], ...
    'Inputs', '+-', 'Position', [90 45 120 110]);
add_block('simulink/Math Operations/Gain', [path '/Lps_to_m3ps'], ...
    'Gain', '0.001', 'Position', [150 62 215 93]);
add_block('simulink/Continuous/Integrator', [path '/VolumeIntegrator'], ...
    'InitialCondition', initialVolumeExpr, ...
    'LimitOutput', 'on', ...
    'UpperSaturationLimit', maxVolumeExpr, ...
    'LowerSaturationLimit', '0', ...
    'Position', [255 55 300 100]);
add_block('simulink/Math Operations/Gain', [path '/Volume_to_Level'], ...
    'Gain', ['100/(' maxVolumeExpr ')'], 'Position', [340 100 435 135]);
add_block('simulink/Ports & Subsystems/Out1', [path '/Volume_m3'], ...
    'Port', '1', 'Position', [480 55 510 75]);
add_block('simulink/Ports & Subsystems/Out1', [path '/Level_pct'], ...
    'Port', '2', 'Position', [480 110 510 130]);

add_line(path, 'Inflow_Lps/1', 'NetFlow/1');
add_line(path, 'Outflow_Lps/1', 'NetFlow/2');
add_line(path, 'NetFlow/1', 'Lps_to_m3ps/1');
add_line(path, 'Lps_to_m3ps/1', 'VolumeIntegrator/1');
add_line(path, 'VolumeIntegrator/1', 'Volume_m3/1');
add_line(path, 'VolumeIntegrator/1', 'Volume_to_Level/1');
add_line(path, 'Volume_to_Level/1', 'Level_pct/1');
end

function addPumpSubsystem(path, maxFlowExpr, tauExpr, position)
add_block('simulink/Ports & Subsystems/Subsystem', path, 'Position', position);
Simulink.SubSystem.deleteContents(path);

add_block('simulink/Ports & Subsystems/In1', [path '/Cmd_pct'], ...
    'Port', '1', 'Position', [25 40 55 60]);
add_block('simulink/Ports & Subsystems/In1', [path '/Available'], ...
    'Port', '2', 'Position', [25 95 55 115]);
add_block('simulink/Discontinuities/Saturation', [path '/CmdLimit'], ...
    'UpperLimit', '100', 'LowerLimit', '0', 'Position', [90 35 130 65]);
add_block('simulink/Math Operations/Product', [path '/EnableProduct'], ...
    'Inputs', '**', 'Position', [165 48 195 102]);
add_block('simulink/Math Operations/Sum', [path '/SpeedError'], ...
    'Inputs', '+-', 'Position', [235 45 265 105]);
add_block('simulink/Math Operations/Gain', [path '/InvTau'], ...
    'Gain', ['1/(' tauExpr ')'], 'Position', [300 60 365 90]);
add_block('simulink/Continuous/Integrator', [path '/SpeedIntegrator'], ...
    'InitialCondition', '0', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', '100', 'LowerSaturationLimit', '0', ...
    'Position', [405 52 450 98]);
add_block('simulink/Math Operations/Gain', [path '/Speed_to_Flow'], ...
    'Gain', ['(' maxFlowExpr ')/100'], 'Position', [490 105 585 135]);
add_block('simulink/Sources/Constant', [path '/RunThreshold'], ...
    'Value', 'P.pump.runFeedbackThreshold_pct', 'Position', [490 175 550 195]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [path '/RunningCompare'], ...
    'Operator', '>', 'Position', [590 145 625 195]);
add_block('simulink/Ports & Subsystems/Out1', [path '/Speed_pct'], ...
    'Port', '1', 'Position', [665 50 695 70]);
add_block('simulink/Ports & Subsystems/Out1', [path '/Flow_Lps'], ...
    'Port', '2', 'Position', [665 110 695 130]);
add_block('simulink/Ports & Subsystems/Out1', [path '/RunFb'], ...
    'Port', '3', 'Position', [665 165 695 185]);

add_line(path, 'Cmd_pct/1', 'CmdLimit/1');
add_line(path, 'CmdLimit/1', 'EnableProduct/1');
add_line(path, 'Available/1', 'EnableProduct/2');
add_line(path, 'EnableProduct/1', 'SpeedError/1');
add_line(path, 'SpeedIntegrator/1', 'SpeedError/2');
add_line(path, 'SpeedError/1', 'InvTau/1');
add_line(path, 'InvTau/1', 'SpeedIntegrator/1');
add_line(path, 'SpeedIntegrator/1', 'Speed_pct/1');
add_line(path, 'SpeedIntegrator/1', 'Speed_to_Flow/1');
add_line(path, 'Speed_to_Flow/1', 'Flow_Lps/1');
add_line(path, 'SpeedIntegrator/1', 'RunningCompare/1');
add_line(path, 'RunThreshold/1', 'RunningCompare/2');
add_line(path, 'RunningCompare/1', 'RunFb/1');
end

function addValveSubsystem(path, position)
add_block('simulink/Ports & Subsystems/Subsystem', path, 'Position', position);
Simulink.SubSystem.deleteContents(path);

add_block('simulink/Ports & Subsystems/In1', [path '/OpenCmd'], ...
    'Port', '1', 'Position', [25 60 55 80]);
add_block('simulink/Discontinuities/Saturation', [path '/CmdLimit'], ...
    'UpperLimit', '1', 'LowerLimit', '0', 'Position', [90 55 130 85]);
add_block('simulink/Math Operations/Sum', [path '/PositionError'], ...
    'Inputs', '+-', 'Position', [175 45 205 105]);
add_block('simulink/Math Operations/Gain', [path '/InvTau'], ...
    'Gain', '1/P.XV201.travelTimeConstant_s', 'Position', [245 60 320 90]);
add_block('simulink/Continuous/Integrator', [path '/PositionIntegrator'], ...
    'InitialCondition', '0', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', '1', 'LowerSaturationLimit', '0', ...
    'Position', [365 52 410 98]);
add_block('simulink/Sources/Constant', [path '/OpenThreshold'], ...
    'Value', 'P.XV201.openFeedbackThreshold', 'Position', [445 125 510 145]);
add_block('simulink/Sources/Constant', [path '/ClosedThreshold'], ...
    'Value', 'P.XV201.closedFeedbackThreshold', 'Position', [445 190 510 210]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [path '/OpenCompare'], ...
    'Operator', '>=', 'Position', [550 105 585 155]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [path '/ClosedCompare'], ...
    'Operator', '<=', 'Position', [550 170 585 220]);
add_block('simulink/Ports & Subsystems/Out1', [path '/Position'], ...
    'Port', '1', 'Position', [630 55 660 75]);
add_block('simulink/Ports & Subsystems/Out1', [path '/OpenFb'], ...
    'Port', '2', 'Position', [630 120 660 140]);
add_block('simulink/Ports & Subsystems/Out1', [path '/ClosedFb'], ...
    'Port', '3', 'Position', [630 185 660 205]);

add_line(path, 'OpenCmd/1', 'CmdLimit/1');
add_line(path, 'CmdLimit/1', 'PositionError/1');
add_line(path, 'PositionIntegrator/1', 'PositionError/2');
add_line(path, 'PositionError/1', 'InvTau/1');
add_line(path, 'InvTau/1', 'PositionIntegrator/1');
add_line(path, 'PositionIntegrator/1', 'Position/1');
add_line(path, 'PositionIntegrator/1', 'OpenCompare/1');
add_line(path, 'OpenThreshold/1', 'OpenCompare/2');
add_line(path, 'OpenCompare/1', 'OpenFb/1');
add_line(path, 'PositionIntegrator/1', 'ClosedCompare/1');
add_line(path, 'ClosedThreshold/1', 'ClosedCompare/2');
add_line(path, 'ClosedCompare/1', 'ClosedFb/1');
end

function addAvailabilityLogic(model, blockName, position)
path = [model '/' blockName];
add_block('simulink/Ports & Subsystems/Subsystem', path, 'Position', position);
Simulink.SubSystem.deleteContents(path);
add_block('simulink/Ports & Subsystems/In1', [path '/Level_pct'], ...
    'Port', '1', 'Position', [25 45 55 65]);
add_block('simulink/Sources/Constant', [path '/MinimumLevel'], ...
    'Value', 'P.tank.minimumSourceLevel_pct', 'Position', [90 95 160 115]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [path '/Compare'], ...
    'Operator', '>', 'Position', [195 45 230 100]);
add_block('simulink/Ports & Subsystems/Out1', [path '/Available'], ...
    'Port', '1', 'Position', [275 60 305 80]);
add_line(path, 'Level_pct/1', 'Compare/1');
add_line(path, 'MinimumLevel/1', 'Compare/2');
add_line(path, 'Compare/1', 'Available/1');
end

function addPressureSubsystem(path, position)
add_block('simulink/Ports & Subsystems/Subsystem', path, 'Position', position);
Simulink.SubSystem.deleteContents(path);

add_block('simulink/Ports & Subsystems/In1', [path '/SpeedA_pct'], ...
    'Port', '1', 'Position', [25 35 55 55]);
add_block('simulink/Ports & Subsystems/In1', [path '/SpeedB_pct'], ...
    'Port', '2', 'Position', [25 80 55 100]);
add_block('simulink/Ports & Subsystems/In1', [path '/Demand_Lps'], ...
    'Port', '3', 'Position', [25 145 55 165]);
add_block('simulink/Math Operations/Sum', [path '/TotalSpeed'], ...
    'Inputs', '++', 'Position', [95 45 125 100]);
add_block('simulink/Math Operations/Gain', [path '/PumpPressureGain'], ...
    'Gain', 'P.pressure.pumpGain_kPa_per_pct', 'Position', [165 55 250 90]);
add_block('simulink/Math Operations/Gain', [path '/DemandPressureGain'], ...
    'Gain', 'P.pressure.demandGain_kPa_per_Lps', 'Position', [165 140 250 170]);
add_block('simulink/Math Operations/Sum', [path '/PressureTarget'], ...
    'Inputs', '+-', 'Position', [290 65 320 150]);
add_block('simulink/Math Operations/Sum', [path '/PressureError'], ...
    'Inputs', '+-', 'Position', [360 65 390 150]);
add_block('simulink/Math Operations/Gain', [path '/InvPressureTau'], ...
    'Gain', '1/P.pressure.timeConstant_s', 'Position', [430 85 510 115]);
add_block('simulink/Continuous/Integrator', [path '/PressureIntegrator'], ...
    'InitialCondition', 'P.pressure.initial_kPa', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', 'P.sim.maxPressure_kPa', 'LowerSaturationLimit', '0', ...
    'Position', [550 75 595 125]);

add_block('simulink/Math Operations/Sum', [path '/SensorError'], ...
    'Inputs', '++-', 'Position', [635 75 665 145]);
add_block('simulink/Sources/Constant', [path '/SensorBias'], ...
    'Value', 'P.pressure.sensorBias_kPa', 'Position', [550 175 610 195]);
add_block('simulink/Math Operations/Gain', [path '/InvSensorTau'], ...
    'Gain', '1/P.pressure.sensorTimeConstant_s', 'Position', [705 95 785 125]);
add_block('simulink/Continuous/Integrator', [path '/SensorIntegrator'], ...
    'InitialCondition', 'P.pressure.initial_kPa', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', 'P.sim.maxPressure_kPa', 'LowerSaturationLimit', '0', ...
    'Position', [825 85 870 135]);
add_block('simulink/Ports & Subsystems/Out1', [path '/PressureActual_kPa'], ...
    'Port', '1', 'Position', [920 75 950 95]);
add_block('simulink/Ports & Subsystems/Out1', [path '/PressurePV_kPa'], ...
    'Port', '2', 'Position', [920 125 950 145]);

add_line(path, 'SpeedA_pct/1', 'TotalSpeed/1');
add_line(path, 'SpeedB_pct/1', 'TotalSpeed/2');
add_line(path, 'TotalSpeed/1', 'PumpPressureGain/1');
add_line(path, 'Demand_Lps/1', 'DemandPressureGain/1');
add_line(path, 'PumpPressureGain/1', 'PressureTarget/1');
add_line(path, 'DemandPressureGain/1', 'PressureTarget/2');
add_line(path, 'PressureTarget/1', 'PressureError/1');
add_line(path, 'PressureIntegrator/1', 'PressureError/2');
add_line(path, 'PressureError/1', 'InvPressureTau/1');
add_line(path, 'InvPressureTau/1', 'PressureIntegrator/1');
add_line(path, 'PressureIntegrator/1', 'PressureActual_kPa/1');
add_line(path, 'PressureIntegrator/1', 'SensorError/1');
add_line(path, 'SensorBias/1', 'SensorError/2');
add_line(path, 'SensorIntegrator/1', 'SensorError/3');
add_line(path, 'SensorError/1', 'InvSensorTau/1');
add_line(path, 'InvSensorTau/1', 'SensorIntegrator/1');
add_line(path, 'SensorIntegrator/1', 'PressurePV_kPa/1');
end

function addConcentrationSubsystem(path, position)
add_block('simulink/Ports & Subsystems/Subsystem', path, 'Position', position);
Simulink.SubSystem.deleteContents(path);

add_block('simulink/Ports & Subsystems/In1', [path '/DoseCmd_pct'], ...
    'Port', '1', 'Position', [25 30 55 50]);
add_block('simulink/Ports & Subsystems/In1', [path '/MixerCmd'], ...
    'Port', '2', 'Position', [25 85 55 105]);
add_block('simulink/Ports & Subsystems/In1', [path '/Volume_m3'], ...
    'Port', '3', 'Position', [25 230 55 250]);
add_block('simulink/Ports & Subsystems/In1', [path '/Outflow_Lps'], ...
    'Port', '4', 'Position', [25 300 55 320]);

add_block('simulink/Discontinuities/Saturation', [path '/DoseCmdLimit'], ...
    'UpperLimit', '100', 'LowerLimit', '0', 'Position', [90 25 130 55]);
add_block('simulink/Math Operations/Sum', [path '/DoseSpeedError'], ...
    'Inputs', '+-', 'Position', [170 20 200 80]);
add_block('simulink/Math Operations/Gain', [path '/InvDoseTau'], ...
    'Gain', '1/P.DP201.timeConstant_s', 'Position', [240 35 315 65]);
add_block('simulink/Continuous/Integrator', [path '/DoseSpeedIntegrator'], ...
    'InitialCondition', '0', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', '100', 'LowerSaturationLimit', '0', ...
    'Position', [355 27 400 73]);

add_block('simulink/Math Operations/Gain', [path '/MixerVariablePart'], ...
    'Gain', '1-P.concentration.minimumMixerEffectiveness', ...
    'Position', [105 85 215 115]);
add_block('simulink/Sources/Constant', [path '/MinimumMixerEffect'], ...
    'Value', 'P.concentration.minimumMixerEffectiveness', ...
    'Position', [105 140 215 160]);
add_block('simulink/Math Operations/Sum', [path '/MixerEffectiveness'], ...
    'Inputs', '++', 'Position', [255 95 285 155]);
add_block('simulink/Math Operations/Product', [path '/EffectiveDoseSpeed'], ...
    'Inputs', '**', 'Position', [440 65 475 115]);
add_block('simulink/Math Operations/Gain', [path '/DoseMassRate'], ...
    'Gain', 'P.DP201.maxChemicalMassRate_mg_s/100', ...
    'Position', [515 72 625 108]);

add_block('simulink/Math Operations/Gain', [path '/Volume_to_Litres'], ...
    'Gain', '1000', 'Position', [105 220 175 250]);
add_block('simulink/Discontinuities/Saturation', [path '/MinimumVolume'], ...
    'UpperLimit', 'P.T201.maxVolume_m3*1000', 'LowerLimit', '100', ...
    'Position', [215 215 300 255]);
add_block('simulink/Math Operations/Divide', [path '/MassDividedByVolume'], ...
    'Position', [670 205 705 255]);
add_block('simulink/Math Operations/Product', [path '/ChemicalOutflow'], ...
    'Inputs', '**', 'Position', [745 280 780 330]);
add_block('simulink/Math Operations/Sum', [path '/MassRate'], ...
    'Inputs', '+-', 'Position', [825 85 855 310]);
add_block('simulink/Continuous/Integrator', [path '/ChemicalMassIntegrator'], ...
    'InitialCondition', ...
    'P.T201.initialConcentration_mgL*P.T201.initialVolume_m3*1000', ...
    'LimitOutput', 'on', ...
    'UpperSaturationLimit', ...
    'P.T201.maxVolume_m3*1000*P.concentration.maxPhysical_mgL', ...
    'LowerSaturationLimit', '0', 'Position', [900 175 945 225]);

add_block('simulink/Math Operations/Sum', [path '/SensorError'], ...
    'Inputs', '++-', 'Position', [995 190 1025 260]);
add_block('simulink/Sources/Constant', [path '/SensorBias'], ...
    'Value', 'P.concentration.sensorBias_mgL', 'Position', [900 285 965 305]);
add_block('simulink/Math Operations/Gain', [path '/InvSensorTau'], ...
    'Gain', '1/P.concentration.sensorTimeConstant_s', ...
    'Position', [1065 205 1145 235]);
add_block('simulink/Continuous/Integrator', [path '/SensorIntegrator'], ...
    'InitialCondition', 'P.T201.initialConcentration_mgL', ...
    'LimitOutput', 'on', ...
    'UpperSaturationLimit', 'P.concentration.maxPhysical_mgL', ...
    'LowerSaturationLimit', '0', 'Position', [1185 197 1230 243]);

add_block('simulink/Ports & Subsystems/Out1', [path '/ConcentrationActual_mgL'], ...
    'Port', '1', 'Position', [1280 175 1310 195]);
add_block('simulink/Ports & Subsystems/Out1', [path '/ConcentrationPV_mgL'], ...
    'Port', '2', 'Position', [1280 225 1310 245]);
add_block('simulink/Ports & Subsystems/Out1', [path '/DoseSpeed_pct'], ...
    'Port', '3', 'Position', [1280 50 1310 70]);

add_line(path, 'DoseCmd_pct/1', 'DoseCmdLimit/1');
add_line(path, 'DoseCmdLimit/1', 'DoseSpeedError/1');
add_line(path, 'DoseSpeedIntegrator/1', 'DoseSpeedError/2');
add_line(path, 'DoseSpeedError/1', 'InvDoseTau/1');
add_line(path, 'InvDoseTau/1', 'DoseSpeedIntegrator/1');
add_line(path, 'DoseSpeedIntegrator/1', 'DoseSpeed_pct/1');
add_line(path, 'MixerCmd/1', 'MixerVariablePart/1');
add_line(path, 'MixerVariablePart/1', 'MixerEffectiveness/1');
add_line(path, 'MinimumMixerEffect/1', 'MixerEffectiveness/2');
add_line(path, 'DoseSpeedIntegrator/1', 'EffectiveDoseSpeed/1');
add_line(path, 'MixerEffectiveness/1', 'EffectiveDoseSpeed/2');
add_line(path, 'EffectiveDoseSpeed/1', 'DoseMassRate/1');
add_line(path, 'Volume_m3/1', 'Volume_to_Litres/1');
add_line(path, 'Volume_to_Litres/1', 'MinimumVolume/1');
add_line(path, 'ChemicalMassIntegrator/1', 'MassDividedByVolume/1');
add_line(path, 'MinimumVolume/1', 'MassDividedByVolume/2');
add_line(path, 'MassDividedByVolume/1', 'ChemicalOutflow/1');
add_line(path, 'Outflow_Lps/1', 'ChemicalOutflow/2');
add_line(path, 'DoseMassRate/1', 'MassRate/1');
add_line(path, 'ChemicalOutflow/1', 'MassRate/2');
add_line(path, 'MassRate/1', 'ChemicalMassIntegrator/1');
add_line(path, 'MassDividedByVolume/1', 'ConcentrationActual_mgL/1');
add_line(path, 'MassDividedByVolume/1', 'SensorError/1');
add_line(path, 'SensorBias/1', 'SensorError/2');
add_line(path, 'SensorIntegrator/1', 'SensorError/3');
add_line(path, 'SensorError/1', 'InvSensorTau/1');
add_line(path, 'InvSensorTau/1', 'SensorIntegrator/1');
add_line(path, 'SensorIntegrator/1', 'ConcentrationPV_mgL/1');
end
