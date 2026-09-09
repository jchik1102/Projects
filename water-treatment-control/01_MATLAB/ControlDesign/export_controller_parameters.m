function T = export_controller_parameters(P, CTRL, printSummary)
% export plc-ready pi parameters and metrics

if nargin < 1 || isempty(P), P = initialize_water_plant(false); end
if nargin < 2 || isempty(CTRL)
    M = derive_plant_models(P, false);
    CTRL = design_all_controllers(P, M, false);
end
if nargin < 3, printSummary = true; end

Controller = ["Pressure PI"; "Concentration PI"];
Kp = [CTRL.pressure.Kp; CTRL.concentration.Kp];
Ki_per_s = [CTRL.pressure.Ki; CTRL.concentration.Ki];
SampleTime_s = [CTRL.pressure.sampleTime_s; CTRL.concentration.sampleTime_s];
KiTimesTs = [CTRL.pressure.KiTimesTs; CTRL.concentration.KiTimesTs];
OutputMin_pct = [CTRL.pressure.outputMin_pct; CTRL.concentration.outputMin_pct];
OutputMax_pct = [CTRL.pressure.outputMax_pct; CTRL.concentration.outputMax_pct];
BiasOrInitialOutput_pct = [CTRL.pressure.nominalCommand_pct; 0];
IntegratorMethod = [string(CTRL.pressure.integratorMethod); string(CTRL.concentration.integratorMethod)];
AntiWindup = [string(CTRL.pressure.antiWindup); string(CTRL.concentration.antiWindup)];
PhaseMargin_deg = [CTRL.pressure.metrics.phaseMargin_deg; CTRL.concentration.metrics.phaseMargin_deg];
SettlingTime_s = [CTRL.pressure.metrics.referenceSettlingTime_s; ...
    CTRL.concentration.metrics.referenceSettlingTime_s];
Overshoot_pct = [CTRL.pressure.metrics.referenceOvershoot_pct; ...
    CTRL.concentration.metrics.referenceOvershoot_pct];

T = table(Controller, Kp, Ki_per_s, SampleTime_s, KiTimesTs, ...
    OutputMin_pct, OutputMax_pct, BiasOrInitialOutput_pct, IntegratorMethod, AntiWindup, ...
    PhaseMargin_deg, SettlingTime_s, Overshoot_pct);

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(fileparts(thisDir));
outDir = fullfile(projectRoot, '05_INTEGRATION');
if ~exist(outDir, 'dir'), mkdir(outDir); end
outFile = fullfile(outDir, 'Controller_Parameters.csv');
writetable(T, outFile);

save(fullfile(projectRoot, '06_TESTING', 'MAT_Files', ...
    'Controller_Design_Data.mat'), 'P', 'CTRL');

if printSummary
    fprintf('Controller parameters exported to %s\n', outFile);
end
end
