function results = run_openplc_simulink_demo()
%RUN_OPENPLC_SIMULINK_DEMO Prove the first real Simulink/OpenPLC loop.
%
% The demo uses one live Modbus client at a time. The bridge performs the
% reset/automatic/start request sequence through its own connection while
% the paced simulation runs. After Simulink releases that connection, this
% function reads PLC status and evaluates the captured physical signals.

check_openplc_requirements();

modelName = 'Water_Treatment_Plant_OpenPLC_v0_4';
modelFile = build_openplc_integrated_model(false);
load_system(modelFile);

% Use a short integration-test operating point. These values affect only
% this simulation and do not change initialize_water_plant.m.
P = initialize_water_plant(false);
P.T101.initialLevel_pct = 80;
P.T101.initialVolume_m3 = P.T101.maxVolume_m3 * 0.80;
P.T201.initialLevel_pct = 75;
P.T201.initialVolume_m3 = P.T201.maxVolume_m3 * 0.75;
P.T301.initialLevel_pct = 70;
P.T301.initialVolume_m3 = P.T301.maxVolume_m3 * 0.70;
P.sim.defaultStopTime_s = 30;
U = create_default_inputs(30, false);

% Clear stale requests/faults and set the test operating values before the
% Simulink bridge takes exclusive ownership of the MATLAB Modbus session.
localDisableDemoSequence();
localClearDiagnosticState();
localPrepareOpenPLC();
localVerifyBridgeDirectly();

localClearDiagnosticState();
setappdata(0, 'WaterProjectOpenPLCDemoSequence', true);
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
    'StopTime', '30', ...
    'ReturnWorkspaceOutputs', 'on');

try
    set_param(modelName, 'EnablePacing', 'on', 'PacingRate', '1');
catch
    warning( ...
        'WaterProject:SimulationPacing', ...
        ['Simulation pacing could not be enabled by script. Enable 1x ' ...
         'pacing in Simulink before rerunning this demo.']);
end

fprintf('\nRunning the paced Simulink/OpenPLC loop for 30 seconds...\n');
try
    simOut = sim(simIn);
catch ME
    bridgeMessage = localLastBridgeError();
    if strlength(bridgeMessage) > 0
        error( ...
            'WaterProject:OpenPLCSimulinkRun', ...
            'The Simulink run stopped. Bridge diagnostic: %s\n%s', ...
            bridgeMessage, ME.message);
    end
    rethrow(ME);
end

% The simulation has ended and releaseImpl has closed the bridge client.
% Disable the private demo request generator before opening a status client.
localDisableDemoSequence();

[~, commExchange] = localSeries(simOut, ...
    'sim_OpenPLC_CommHealthy', 1);
[~, digitalCommands] = localSeries(simOut, ...
    'sim_OpenPLC_DigitalCommands', 7);
[~, physicalFeedback] = localSeries(simOut, ...
    'sim_OpenPLC_Feedback', 7);
[~, t201Level] = localSeries(simOut, 'sim_T201_Level_pct', 1);

bridgeHealthy = any(commExchange(:, 1) > 0.5);
startCommand = any(any(digitalCommands(:, 1:2) > 0.5, 2));
runFeedback = any(any(physicalFeedback(:, 1:2) > 0.5, 2));
initialLevel_pct = t201Level(1, 1);
finalLevel_pct = t201Level(end, 1);
levelIncreased = finalLevel_pct >= initialLevel_pct + 0.20;

% Read PLC-owned status only after Simulink has released its client.
m = modbus('tcpip', '127.0.0.1', 5020, 'Timeout', 3);
plcCleanup = onCleanup(@() localSafeStop(m)); %#ok<NASGU>
watchdogHealthy = read(m, 'coils', 151, 1) == 1;
automaticMode = read(m, 'holdingregs', 306, 1) == 1;

Check = [ ...
    "Heartbeat watchdog healthy"
    "Automatic mode active"
    "One P-101 pump commanded"
    "Physical pump feedback returned"
    "T-201 level increased"];
Pass = [ ...
    watchdogHealthy && bridgeHealthy
    automaticMode
    startCommand
    runFeedback
    levelIncreased];
Details = [ ...
    sprintf('PLC watchdog=%d; bridge exchanges=%d', ...
        watchdogHealthy, bridgeHealthy)
    string(automaticMode)
    string(startCommand)
    string(runFeedback)
    sprintf('%.2f%% -> %.2f%%', initialLevel_pct, finalLevel_pct)];

results = table(Check, Pass, Details);

fprintf('\n============================================================\n');
fprintf(' SIMULINK-OPENPLC FIRST CLOSED-LOOP TEST\n');
fprintf('============================================================\n');
disp(results);

if ~all(results.Pass)
    bridgeMessage = localLastBridgeError();
    if strlength(bridgeMessage) > 0
        fprintf('Last bridge diagnostic: %s\n', bridgeMessage);
    end
end

assert(all(results.Pass), ...
    ['The first Simulink/OpenPLC loop did not pass. Review the model ' ...
     'communication log and OpenPLC online values.']);

fprintf('SIMULINK-OPENPLC FIRST CLOSED-LOOP TEST: PASSED\n');
end


function localPrepareOpenPLC()
m = modbus('tcpip', '127.0.0.1', 5020, 'Timeout', 3);

% Stop any previous run, then clear only mapped request and fault ranges.
localPulse(m, 102);
write(m, 'coils', 101, zeros(1, 6));
write(m, 'coils', 110, zeros(1, 9));
write(m, 'coils', 251, zeros(1, 4));
write(m, 'holdingregs', 201, [4000 120 800 2 500]);

% Seed plausible plant values. The bridge overwrites them from Simulink.
write(m, 'holdingregs', 1, [800 750 700 0 0 300 0 4000 0]);
pause(0.25);
clear m
pause(0.25);
end


function localVerifyBridgeDirectly()
localClearDiagnosticState();
bridge = OpenPLCModbusBridge();
cleanup = onCleanup(@() localReleaseBridge(bridge)); %#ok<NASGU>

measurements = [80 75 70 0 0 30 0 400];
feedback = [0 0 0 0 0 0 1];
[~, ~, ~, commHealthy] = step(bridge, measurements, feedback);

if commHealthy ~= 1
    message = localLastBridgeError();
    if strlength(message) == 0
        message = "No detailed communication exception was recorded.";
    end
    error( ...
        'WaterProject:OpenPLCBridgePreflight', ...
        'OpenPLCModbusBridge preflight failed: %s', message);
end

release(bridge);
fprintf('OpenPLCModbusBridge direct preflight: PASSED\n');
end


function [t, samples] = localSeries(out, variableName, width)
ts = out.get(variableName);
if ~isa(ts, 'timeseries')
    error( ...
        'WaterProject:UnexpectedLogFormat', ...
        'Expected timeseries output for %s.', variableName);
end

t = ts.Time;
samples = squeeze(double(ts.Data));
if width == 1
    samples = samples(:);
elseif size(samples, 2) == width
    % Already one row per time sample.
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


function localDisableDemoSequence()
try
    if isappdata(0, 'WaterProjectOpenPLCDemoSequence')
        rmappdata(0, 'WaterProjectOpenPLCDemoSequence');
    end
catch
end
end


function localSimulationCleanup(modelName)
localDisableDemoSequence();
try
    if bdIsLoaded(modelName) && ...
            ~strcmp(get_param(modelName, 'SimulationStatus'), 'stopped')
        set_param(modelName, 'SimulationCommand', 'stop');
    end
catch
end
end


function localReleaseBridge(bridge)
try
    release(bridge);
catch
end
end


function localPulse(m, address)
write(m, 'coils', address, 1);
pause(0.15);
write(m, 'coils', address, 0);
pause(0.15);
end


function localSafeStop(m)
try
    localPulse(m, 102);
catch
end
end
