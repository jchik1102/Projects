function results = run_stage5_full_control_integration()
% run the 260 s commissioning profile
% tank volume and dose capacity are both scaled to 10% to shorten the run

check_openplc_requirements();

modelName = 'Water_Treatment_Plant_OpenPLC_v0_4';
modelFile = build_openplc_integrated_model(false);
load_system(modelFile);

stopTime_s = 260;
[P, U] = localCommissioningProfile(stopTime_s);

localDisableTestSequences();
localClearDiagnosticState();
localPrepareOpenPLC();
localVerifyBridgeDirectly();

setappdata(0, 'WaterProjectOpenPLCStage5Sequence', true);
simulationCleanup = onCleanup( ...
    @() localSimulationCleanup(modelName)); %#ok<NASGU>

simIn = Simulink.SimulationInput(modelName);
simIn = simIn.setVariable('P', P);
fields = fieldnames(U);
for k = 1:numel(fields)
    simIn = simIn.setVariable(fields{k}, U.(fields{k}));
end
simIn = simIn.setModelParameter( ...
    'SimulationMode', 'normal', ...
    'StopTime', num2str(stopTime_s), ...
    'ReturnWorkspaceOutputs', 'on');

try
    set_param(modelName, 'EnablePacing', 'on', 'PacingRate', '1');
catch
    warning( ...
        'WaterProject:SimulationPacing', ...
        ['Simulation pacing could not be enabled by script. Enable 1x ' ...
         'pacing in Simulink before rerunning Stage 5.']);
end

fprintf('\n============================================================\n');
fprintf(' STAGE 5 - FULL CONTROL INTEGRATION\n');
fprintf('============================================================\n');
fprintf('Commissioning profile duration: %d seconds (about %.1f minutes)\n', ...
    stopTime_s, stopTime_s/60);
fprintf('Do not run the smoke or PLC-only acceptance tests concurrently.\n');
fprintf('Running the paced Simulink/OpenPLC loop now...\n\n');

try
    simOut = sim(simIn);
catch ME
    bridgeMessage = localLastBridgeError();
    if strlength(bridgeMessage) > 0
        error( ...
            'WaterProject:Stage5SimulinkRun', ...
            'Stage 5 stopped. Bridge diagnostic: %s\n%s', ...
            bridgeMessage, ME.message);
    end
    rethrow(ME);
end

localDisableTestSequences();
localPostTestSafeState();

[tMeasurements, measurements] = localSeries( ...
    simOut, 'sim_OpenPLC_Measurements', 8);
[tFeedback, feedback] = localSeries( ...
    simOut, 'sim_OpenPLC_Feedback', 7);
[tAnalog, analogCommands] = localSeries( ...
    simOut, 'sim_OpenPLC_AnalogCommands', 6);
[tDigital, digitalCommands] = localSeries( ...
    simOut, 'sim_OpenPLC_DigitalCommands', 7);
[tFaults, faultInputs] = localSeries( ...
    simOut, 'sim_OpenPLC_FaultInputs', 4);
[tComm, commHealthy] = localSeries( ...
    simOut, 'sim_OpenPLC_CommHealthy', 1);
[tDiagnostics, diagnostics] = localSeries( ...
    simOut, 'sim_OpenPLC_Diagnostics', 48);

% HR301:HR317 use columns 1:17; C151:C181 use 18:48 (see Stage5_Diagnostic_Map.csv)
batchState = round(diagnostics(:, 1));
batchCount = round(diagnostics(:, 2));
rejectedBatchCount = round(diagnostics(:, 3));
pressurePIOutput_pct = diagnostics(:, 4) / 10;
concentrationPIOutput_pct = diagnostics(:, 5) / 10;
leadBooster = round(diagnostics(:, 8));
firstOutCode = round(diagnostics(:, 9));
alarmWord1 = round(diagnostics(:, 10));
alarmWord2 = round(diagnostics(:, 11));
plcCommHealthy = diagnostics(:, 18) > 0.5;
pressurePIEnabled = diagnostics(:, 25) > 0.5;
concentrationPIEnabled = diagnostics(:, 26) > 0.5;
communicationFault = diagnostics(:, 43) > 0.5;
xv201FailedOpen = diagnostics(:, 42) > 0.5;

validDiagnostics = tDiagnostics >= 5;
initialDiagnosticIndex = find(validDiagnostics, 1, 'first');
assert(~isempty(initialDiagnosticIndex), ...
    'Stage 5 did not capture PLC diagnostics after startup.');
initialBatchCount = batchCount(initialDiagnosticIndex);
initialRejectedCount = rejectedBatchCount(initialDiagnosticIndex);

bridgeExchangeHealthy = mean(commHealthy(tComm >= 5) > 0.5) >= 0.98;
watchdogHealthy = mean(plcCommHealthy(validDiagnostics)) >= 0.95 && ...
    ~any(communicationFault(validDiagnostics));

requiredStates = [20 30 40 60]; % fill, dose, mix, transfer
statesVisited = all(ismember(requiredStates, unique(batchState)));
batchCompleted = max(batchCount) >= initialBatchCount + 1;
noBatchRejected = max(rejectedBatchCount) == initialRejectedCount;
observedFirstOut = localFirstNonzero(firstOutCode);

highDemand = tDigital >= 25 & tDigital <= 44;
bothBoostersStaged = any( ...
    digitalCommands(highDemand, 4) > 0.5 & ...
    digitalCommands(highDemand, 5) > 0.5);
pressureLoopExecuted = any(pressurePIEnabled) && ...
    max(pressurePIOutput_pct) > 95;
pressureRecoveryWindow = tMeasurements >= 52 & tMeasurements < 59;
pressureRecoveryError_kPa = median(abs( ...
    measurements(pressureRecoveryWindow, 8) - 400));
pressureRecovered = pressureRecoveryError_kPa <= 25;

doseCommanded = max(analogCommands(:, 6)) > 5;
qualityBand = measurements(:, 7) >= 1.10 & ...
    measurements(:, 7) <= 1.30;
samplePeriod_s = median(diff(tMeasurements));
qualityTime_s = sum(qualityBand) * samplePeriod_s;
concentrationLoopExecuted = any(concentrationPIEnabled) && ...
    doseCommanded && qualityTime_s >= 15 && ...
    max(measurements(:, 7)) < 1.50;

completionIndex = find(batchCount >= initialBatchCount + 1, 1, 'first');
leadAlternated = false;
leadBefore = NaN;
leadAfter = NaN;
if ~isempty(completionIndex)
    completionTime_s = tDiagnostics(completionIndex);
    beforeWindow = tDiagnostics >= completionTime_s - 5 & ...
        tDiagnostics < completionTime_s;
    afterWindow = tDiagnostics >= completionTime_s & ...
        tDiagnostics <= completionTime_s + 2;
    leadBefore = localMostCommonNonzero(leadBooster(beforeWindow));
    leadAfter = localMostCommonNonzero(leadBooster(afterWindow));
    leadAlternated = ismember(leadBefore, [1 2]) && ...
        ismember(leadAfter, [1 2]) && leadBefore ~= leadAfter;
end

signalCount = min([ ...
    numel(tDigital), numel(tFeedback), numel(tFaults)]);
tripActive = faultInputs(1:signalCount, 1) > 0.5;
standbyTakeover = any( ...
    tripActive & ...
    digitalCommands(1:signalCount, 4) < 0.5 & ...
    digitalCommands(1:signalCount, 5) > 0.5 & ...
    feedback(1:signalCount, 5) > 0.5);
leadChangedToStandby = any( ...
    leadBooster(tDiagnostics >= 62 & tDiagnostics <= 75) == 2);
standbyTakeover = standbyTakeover && leadChangedToStandby;

Check = [ ...
    "Stable command/feedback exchange"
    "Complete automatic treatment batch"
    "Pressure PI control through OpenPLC"
    "Concentration PI control through OpenPLC"
    "High-demand booster staging"
    "Lead-booster alternation after batch"
    "Standby takeover after lead trip"];
Pass = [ ...
    bridgeExchangeHealthy && watchdogHealthy
    batchCompleted && statesVisited && noBatchRejected
    pressureLoopExecuted && pressureRecovered
    concentrationLoopExecuted
    bothBoostersStaged
    leadAlternated
    standbyTakeover];
Details = [ ...
    string(sprintf('Bridge healthy %.1f%%; watchdog healthy %.1f%%', ...
        100*mean(commHealthy(tComm >= 5) > 0.5), ...
        100*mean(plcCommHealthy(validDiagnostics))))
    string(sprintf(['Count %d -> %d; states [%s]; rejected +%d; ' ...
        'first-out %g; alarms [%d %d]; XV-201 failed-open %d'], ...
        initialBatchCount, max(batchCount), ...
        strtrim(sprintf('%d ', unique(batchState).')), ...
        max(rejectedBatchCount)-initialRejectedCount, ...
        observedFirstOut, max(alarmWord1), max(alarmWord2), ...
        any(xv201FailedOpen)))
    string(sprintf('Max output %.1f%%; recovery median error %.1f kPa', ...
        max(pressurePIOutput_pct), pressureRecoveryError_kPa))
    string(sprintf('Max dose %.1f%%; quality-band time %.1f s; peak %.3f mg/L', ...
        max(concentrationPIOutput_pct), qualityTime_s, ...
        max(measurements(:, 7))))
    string(sprintf('Both P-301 commands active during 60 L/s demand: %d', ...
        bothBoostersStaged))
    string(sprintf('Lead %g -> %g at completed-batch transition', ...
        leadBefore, leadAfter))
    string(sprintf('P-301B command and feedback active while P-301A tripped: %d', ...
        standbyTakeover))];

results = table(Check, Pass, Details);

projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
resultsDir = fullfile(projectRoot, '06_TESTING', 'Test_Results');
matDir = fullfile(projectRoot, '06_TESTING', 'MAT_Files');
plotDir = fullfile(projectRoot, '06_TESTING', 'Plots');
localEnsureDirectory(resultsDir);
localEnsureDirectory(matDir);
localEnsureDirectory(plotDir);

writetable(results, fullfile( ...
    resultsDir, 'Stage5_Full_Control_Integration.csv'));
save(fullfile(matDir, 'Stage5_Full_Control_Integration.mat'), ...
    'results', ...
    'tMeasurements', 'measurements', ...
    'tFeedback', 'feedback', ...
    'tAnalog', 'analogCommands', ...
    'tDigital', 'digitalCommands', ...
    'tFaults', 'faultInputs', ...
    'tDiagnostics', 'diagnostics');
localCreateEvidencePlot( ...
    plotDir, ...
    tMeasurements, measurements, ...
    tAnalog, analogCommands, ...
    tDigital, digitalCommands, ...
    tFaults, faultInputs, ...
    tDiagnostics, batchState);

fprintf('\n============================================================\n');
fprintf(' STAGE 5 RESULTS\n');
fprintf('============================================================\n');
disp(results);
fprintf('Evidence saved under 06_TESTING.\n');

assert(all(results.Pass), ...
    ['Stage 5 did not pass. Review Stage5_Full_Control_Integration.csv ' ...
     'and the Stage 5 trend plot.']);

fprintf('\nSTAGE 5 - FULL CONTROL INTEGRATION: PASSED\n');
fprintf('Controller/plant portion of Integration Gate C1: PASSED\n');
fprintf('Ignition display/history portion remains for Stage 6.\n');
end


function [P, U] = localCommissioningProfile(stopTime_s)
P = initialize_water_plant(false);

scale = 0.10;
P.T201.maxVolume_m3 = P.T201.maxVolume_m3 * scale;
P.T201.area_m2 = P.T201.maxVolume_m3 / P.T201.height_m;
P.T201.initialLevel_pct = 10;
P.T201.initialVolume_m3 = ...
    P.T201.maxVolume_m3 * P.T201.initialLevel_pct / 100;
P.T201.batchFillSetpoint_pct = 50;
P.DP201.maxChemicalMassRate_mg_s = ...
    P.DP201.maxChemicalMassRate_mg_s * scale;
P.M201.mixTime_s = 1;
P.sim.defaultStopTime_s = stopTime_s;

U = create_default_inputs(stopTime_s, false);
U.demand_flow = [ ...
    0,       30
    14.999,  30
    15.000,  60
    44.999,  60
    45.000,  30
    stopTime_s, 30];
end


function localPrepareOpenPLC()
m = modbus('tcpip', '127.0.0.1', 5020, 'Timeout', 3);
cleanup = onCleanup(@() localReleaseClient(m)); %#ok<NASGU>

localPulse(m, 102);
write(m, 'coils', 101, zeros(1, 6));
write(m, 'coils', 110, zeros(1, 9));
write(m, 'coils', 251, zeros(1, 4));
write(m, 'holdingregs', 201, [4000 120 500 1 500]);
write(m, 'holdingregs', 1, [700 100 700 0 0 300 0 4000 0]);
pause(0.25);
end


function localVerifyBridgeDirectly()
localClearDiagnosticState();
bridge = OpenPLCModbusBridge();
cleanup = onCleanup(@() localReleaseBridge(bridge)); %#ok<NASGU>

measurements = [70 10 70 0 0 30 0 400];
feedback = [0 0 0 0 0 0 1];
[~, ~, ~, commHealthy, diagnostics] = ...
    step(bridge, measurements, feedback);

if commHealthy ~= 1 || numel(diagnostics) ~= 48
    message = localLastBridgeError();
    if strlength(message) == 0
        message = "No detailed communication exception was recorded.";
    end
    error( ...
        'WaterProject:Stage5BridgePreflight', ...
        'OpenPLCModbusBridge Stage 5 preflight failed: %s', message);
end

release(bridge);
fprintf('OpenPLCModbusBridge Stage 5 preflight: PASSED\n');
end


function [t, samples] = localSeries(out, variableName, width)
ts = out.get(variableName);
if ~isa(ts, 'timeseries')
    error( ...
        'WaterProject:UnexpectedLogFormat', ...
        'Expected timeseries output for %s.', variableName);
end

t = ts.Time(:);
samples = squeeze(double(ts.Data));
if width == 1
    samples = samples(:);
elseif size(samples, 2) == width
elseif size(samples, 1) == width
    samples = samples.';
elseif mod(numel(samples), width) == 0
    samples = reshape(samples, width, []).';
else
    error( ...
        'WaterProject:UnexpectedSignalWidth', ...
        'Log %s does not contain width-%d samples.', variableName, width);
end

if isempty(samples)
    error( ...
        'WaterProject:EmptyIntegrationLog', ...
        'Integration log %s is empty.', variableName);
end
end


function value = localMostCommonNonzero(values)
values = round(values(:));
values = values(values > 0);
if isempty(values)
    value = NaN;
else
    value = mode(values);
end
end


function value = localFirstNonzero(values)
values = round(values(:));
index = find(values > 0, 1, 'first');
if isempty(index)
    value = 0;
else
    value = values(index);
end
end


function localCreateEvidencePlot( ...
        plotDir, ...
        tMeasurements, measurements, ...
        tAnalog, analogCommands, ...
        tDigital, digitalCommands, ...
        tFaults, faultInputs, ...
        tDiagnostics, batchState)
fig = figure('Visible', 'off', 'Color', 'w', ...
    'Position', [100 100 1200 900]);
layout = tiledlayout(fig, 4, 1, ...
    'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(layout);
plot(tMeasurements, measurements(:, 8), 'LineWidth', 1.2);
hold on;
yline(400, '--', '400 kPa setpoint');
ylabel('Pressure (kPa)');
grid on;
title('OpenPLC pressure PI response');

nexttile(layout);
yyaxis left;
plot(tMeasurements, measurements(:, 7), 'LineWidth', 1.2);
yline(1.10, '--');
yline(1.30, '--');
ylabel('AIT-201 (mg/L)');
yyaxis right;
plot(tAnalog, analogCommands(:, 6), 'LineWidth', 1.0);
ylabel('Dose output (%)');
grid on;
title('OpenPLC concentration PI response');

nexttile(layout);
stairs(tDigital, digitalCommands(:, 4), 'LineWidth', 1.1);
hold on;
stairs(tDigital, digitalCommands(:, 5), 'LineWidth', 1.1);
stairs(tFaults, faultInputs(:, 1), '--', 'LineWidth', 1.1);
ylabel('Boolean state');
legend('P-301A command', 'P-301B command', 'P-301A trip', ...
    'Location', 'best');
grid on;
title('Staging and standby takeover');

nexttile(layout);
stairs(tDiagnostics, batchState, 'LineWidth', 1.2);
ylabel('Batch state');
xlabel('Simulation time (s)');
yticks([0 10 20 30 40 50 60 70 900]);
grid on;
title('Automatic treatment sequence');

title(layout, 'Stage 5 Full Control Integration Evidence');
exportgraphics(fig, fullfile( ...
    plotDir, 'Stage5_Full_Control_Integration.png'), ...
    'Resolution', 160);
close(fig);
end


function localEnsureDirectory(path)
if ~exist(path, 'dir')
    mkdir(path);
end
end


function message = localLastBridgeError()
message = "";
try
    if isappdata(0, 'WaterProjectOpenPLCLastError')
        message = string(getappdata(0, ...
            'WaterProjectOpenPLCLastError'));
    end
catch
end
end


function localClearDiagnosticState()
try
    if isappdata(0, 'WaterProjectOpenPLCLastError')
        rmappdata(0, 'WaterProjectOpenPLCLastError');
    end
catch
end
end


function localDisableTestSequences()
keys = { ...
    'WaterProjectOpenPLCDemoSequence', ...
    'WaterProjectOpenPLCStage5Sequence'};
for k = 1:numel(keys)
    try
        if isappdata(0, keys{k})
            rmappdata(0, keys{k});
        end
    catch
    end
end
end


function localSimulationCleanup(modelName)
localDisableTestSequences();
try
    if bdIsLoaded(modelName) && ...
            ~strcmp(get_param(modelName, 'SimulationStatus'), 'stopped')
        set_param(modelName, 'SimulationCommand', 'stop');
    end
catch
end
try
    pause(0.25);
    localPostTestSafeState();
catch
end
end


function localPostTestSafeState()
m = modbus('tcpip', '127.0.0.1', 5020, 'Timeout', 3);
cleanup = onCleanup(@() localReleaseClient(m)); %#ok<NASGU>
write(m, 'coils', 251, zeros(1, 4));
write(m, 'coils', 101, zeros(1, 6));
localPulse(m, 102);
end


function localReleaseBridge(bridge)
try
    release(bridge);
catch
end
end


function localReleaseClient(~)
% clearing the local client releases the socket
end


function localPulse(m, address)
write(m, 'coils', address, 1);
pause(0.15);
write(m, 'coils', address, 0);
pause(0.15);
end
