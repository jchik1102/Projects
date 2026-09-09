function modelFiles = build_controller_test_models(forceRebuild)
%BUILD_CONTROLLER_TEST_MODELS Generate pressure and concentration PI test models.

if nargin < 1, forceRebuild = false; end
thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(fileparts(thisDir));
modelDir = fullfile(projectRoot, '02_SIMULINK');
if ~exist(modelDir, 'dir'), mkdir(modelDir); end

P = initialize_water_plant(false);
M = derive_plant_models(P, false);
CTRL = design_all_controllers(P, M, false);
assignin('base','P',P); assignin('base','MODELS',M); assignin('base','CTRL',CTRL);

modelFiles.pressure = fullfile(modelDir, 'Pressure_Controller_Test.slx');
modelFiles.concentration = fullfile(modelDir, 'Concentration_Controller_Test.slx');

buildPressureModel(modelFiles.pressure, forceRebuild);
buildConcentrationModel(modelFiles.concentration, forceRebuild);
end

function buildPressureModel(modelFile, forceRebuild)
model = 'Pressure_Controller_Test';
if exist(modelFile,'file') && ~forceRebuild, return; end
if bdIsLoaded(model), close_system(model,0); end
load_system('simulink');
new_system(model);
set_param(model,'SolverType','Fixed-step','Solver','ode4', ...
    'FixedStep','P.sim.baseStep_s','StopTime','60', ...
    'ReturnWorkspaceOutputs','on','SaveTime','on','TimeSaveName','tout');

add_block('simulink/Sources/Constant',[model '/SetpointDeviation'], ...
    'Value','0','Position',[30 80 80 110]);
add_block('simulink/Sources/From Workspace',[model '/DemandDeviation'], ...
    'VariableName','pressure_demand_delta','Interpolate','off', ...
    'OutputAfterFinalValue','Holding final value','Position',[30 240 155 270]);
add_block('simulink/Math Operations/Sum',[model '/Error'], ...
    'Inputs','+-','Position',[200 80 230 120]);
addPID(model,[model '/Pressure_PI'], ...
    'CTRL.pressure.Kp','CTRL.pressure.Ki','CTRL.pressure.sampleTime_s', ...
    'CTRL.pressure.deltaOutputMin_pct','CTRL.pressure.deltaOutputMax_pct', ...
    [275 60 400 140]);
add_block('simulink/Continuous/Transfer Fcn',[model '/PumpPlant'], ...
    'Numerator','P.pressure.pumpGain_kPa_per_pct', ...
    'Denominator','[P.pressure.timeConstant_s 1]', ...
    'Position',[470 75 600 125]);
add_block('simulink/Continuous/Transfer Fcn',[model '/DemandPlant'], ...
    'Numerator','-P.pressure.demandGain_kPa_per_Lps', ...
    'Denominator','[P.pressure.timeConstant_s 1]', ...
    'Position',[470 225 600 275]);
add_block('simulink/Math Operations/Sum',[model '/PressureDeviation'], ...
    'Inputs','++','Position',[660 115 690 165]);
add_block('simulink/Continuous/Transfer Fcn',[model '/PressureSensor'], ...
    'Numerator','1','Denominator','[P.pressure.sensorTimeConstant_s 1]', ...
    'Position',[760 120 890 160]);
add_block('simulink/Sources/Constant',[model '/PressureBias'], ...
    'Value','P.pressure.setpoint_kPa','Position',[760 30 880 60]);
add_block('simulink/Math Operations/Sum',[model '/ActualPressure'], ...
    'Inputs','++','Position',[960 65 990 105]);
add_block('simulink/Math Operations/Sum',[model '/MeasuredPressure'], ...
    'Inputs','++','Position',[960 140 990 180]);
add_block('simulink/Sources/Constant',[model '/NominalCommand'], ...
    'Value','CTRL.pressure.nominalCommand_pct','Position',[470 10 600 40]);
add_block('simulink/Math Operations/Sum',[model '/TotalCommand'], ...
    'Inputs','++','Position',[660 20 690 60]);

addToWorkspace(model,'Log_Pressure','test_pressure_kPa',[1090 65 1235 90]);
addToWorkspace(model,'Log_PressurePV','test_pressure_pv_kPa',[1090 140 1235 165]);
addToWorkspace(model,'Log_Command','test_pressure_command_pct',[770 10 930 35]);
addToWorkspace(model,'Log_DemandDelta','test_demand_delta_Lps',[200 240 350 265]);

add_line(model,'SetpointDeviation/1','Error/1');
add_line(model,'PressureSensor/1','Error/2','autorouting','on');
add_line(model,'Error/1','Pressure_PI/1');
add_line(model,'Pressure_PI/1','PumpPlant/1');
add_line(model,'Pressure_PI/1','TotalCommand/2','autorouting','on');
add_line(model,'NominalCommand/1','TotalCommand/1');
add_line(model,'TotalCommand/1','Log_Command/1');
add_line(model,'DemandDeviation/1','DemandPlant/1');
add_line(model,'DemandDeviation/1','Log_DemandDelta/1','autorouting','on');
add_line(model,'PumpPlant/1','PressureDeviation/1');
add_line(model,'DemandPlant/1','PressureDeviation/2');
add_line(model,'PressureDeviation/1','PressureSensor/1');
add_line(model,'PressureDeviation/1','ActualPressure/2','autorouting','on');
add_line(model,'PressureSensor/1','MeasuredPressure/2','autorouting','on');
add_line(model,'PressureBias/1','ActualPressure/1','autorouting','on');
add_line(model,'PressureBias/1','MeasuredPressure/1','autorouting','on');
add_line(model,'ActualPressure/1','Log_Pressure/1');
add_line(model,'MeasuredPressure/1','Log_PressurePV/1');

Simulink.BlockDiagram.arrangeSystem(model);
save_system(model,modelFile); close_system(model,0);
end

function buildConcentrationModel(modelFile, forceRebuild)
model = 'Concentration_Controller_Test';
if exist(modelFile,'file') && ~forceRebuild, return; end
if bdIsLoaded(model), close_system(model,0); end
load_system('simulink');
new_system(model);
set_param(model,'SolverType','Fixed-step','Solver','ode4', ...
    'FixedStep','P.sim.baseStep_s','StopTime','160', ...
    'ReturnWorkspaceOutputs','on','SaveTime','on','TimeSaveName','tout');

add_block('simulink/Sources/Constant',[model '/ConcentrationSetpoint'], ...
    'Value','P.concentration.setpoint_mgL','Position',[30 70 145 100]);
add_block('simulink/Math Operations/Sum',[model '/Error'], ...
    'Inputs','+-','Position',[205 65 235 105]);
addPID(model,[model '/Concentration_PI'], ...
    'CTRL.concentration.Kp','CTRL.concentration.Ki', ...
    'CTRL.concentration.sampleTime_s','CTRL.concentration.outputMin_pct', ...
    'CTRL.concentration.outputMax_pct',[285 45 425 125]);
add_block('simulink/Continuous/Transfer Fcn',[model '/ConcentrationPlant'], ...
    'Numerator','P.controller.concentration.approxGain_mgL_per_pct', ...
    'Denominator','[P.controller.concentration.approxTimeConstant_s 1]', ...
    'Position',[500 60 650 110]);
add_block('simulink/Continuous/Transfer Fcn',[model '/ConcentrationSensor'], ...
    'Numerator','1','Denominator','[P.concentration.sensorTimeConstant_s 1]', ...
    'Position',[725 60 870 110]);

addToWorkspace(model,'Log_Concentration','test_concentration_mgL',[950 45 1115 70]);
addToWorkspace(model,'Log_ConcentrationPV','test_concentration_pv_mgL',[950 95 1115 120]);
addToWorkspace(model,'Log_DoseCommand','test_dose_command_pct',[500 10 660 35]);

add_line(model,'ConcentrationSetpoint/1','Error/1');
add_line(model,'ConcentrationSensor/1','Error/2','autorouting','on');
add_line(model,'Error/1','Concentration_PI/1');
add_line(model,'Concentration_PI/1','ConcentrationPlant/1');
add_line(model,'Concentration_PI/1','Log_DoseCommand/1','autorouting','on');
add_line(model,'ConcentrationPlant/1','ConcentrationSensor/1');
add_line(model,'ConcentrationPlant/1','Log_Concentration/1','autorouting','on');
add_line(model,'ConcentrationSensor/1','Log_ConcentrationPV/1');

Simulink.BlockDiagram.arrangeSystem(model);
save_system(model,modelFile); close_system(model,0);
end

function addPID(model,path,Kp,Ki,Ts,lo,hi,pos)
try
    add_block('simulink/Discrete/Discrete PID Controller',path,'Position',pos);
catch
    add_block('slpidlib/PID Controller',path,'Position',pos);
end
set_param(path,'Controller','PI','TimeDomain','Discrete-time', ...
    'P',Kp,'I',Ki,'SampleTime',Ts,'UseKiTs','off', ...
    'IntegratorMethod','Forward Euler','LimitOutput','on', ...
    'SatLimitsSource','internal','LowerSaturationLimit',lo, ...
    'UpperSaturationLimit',hi,'AntiWindupMode','clamping');
end

function addToWorkspace(model,name,varName,pos)
add_block('simulink/Sinks/To Workspace',[model '/' name], ...
    'VariableName',varName,'SaveFormat','Timeseries','Position',pos);
end
