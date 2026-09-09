function results = run_simulink_plant_tests()
%RUN_SIMULINK_PLANT_TESTS Build, simulate, and verify the standalone Simulink plant.

fprintf('\n============================================================\n');
fprintf(' STAGE 3 STANDALONE SIMULINK PLANT TESTS\n');
fprintf('============================================================\n');

build_water_treatment_plant(false);

names = { ...
    'Pump lag and treatment-tank filling'; ...
    'Valve travel and closed-flow blocking'; ...
    'Pressure demand-step recovery'; ...
    'Concentration mass-balance response'; ...
    'Total-water mass balance'; ...
    'Tank physical bounds'; ...
    'Pressure sensor lag'};
funcs = { ...
    @testPumpAndTank; ...
    @testValve; ...
    @testPressure; ...
    @testConcentration; ...
    @testMassBalance; ...
    @testBounds; ...
    @testSensorLag};

results = executeTests(names, funcs);
display(results);
saveResults(results, 'Stage3_SIMULINK_Plant.csv');

if ~all(results.Pass)
    error('WaterProject:Stage3Failed', 'One or more Stage 3 tests failed.');
end
fprintf('Stage 3 result: PASSED\n');
end

function detail = testPumpAndTank()
P = initialize_water_plant(false);
T = 12;
U = create_default_inputs(T, false);
U.raw_inflow = holdProfile(T, P.P101A.maxFlow_Lps);
U.demand_flow = holdProfile(T, 0);
U.cmd_P101A = holdProfile(T, 100);

out = simulatePlant(P, U, T);
[tSpeed, speed] = getSeries(out, 'sim_P101A_Speed_pct');
[~, volume] = getSeries(out, 'sim_T201_Volume_m3');

expectedIncrease = P.P101A.maxFlow_Lps * ...
    (T - P.P101A.timeConstant_s * (1-exp(-T/P.P101A.timeConstant_s))) / 1000;
actualIncrease = volume(end) - P.T201.initialVolume_m3;

assert(speed(end) > 99.0, 'Pump speed did not reach steady state.');
assert(abs(actualIncrease - expectedIncrease) <= P.test.fillVolumeTolerance_m3, ...
    'T-201 volume increase differs from the first-order analytical result.');
assert(all(diff(tSpeed) > 0), 'Logged time vector is not strictly increasing.');

detail = sprintf('Final speed %.2f%%; volume increase %.4f m^3 (expected %.4f).', ...
    speed(end), actualIncrease, expectedIncrease);
end

function detail = testValve()
P = initialize_water_plant(false);
P.T201.initialLevel_pct = 80;
P.T201.initialVolume_m3 = P.T201.maxVolume_m3 * 0.80;
T = 15;
U = create_default_inputs(T, false);
U.demand_flow = holdProfile(T, 0);
U.cmd_P201 = holdProfile(T, 100);
U.cmd_XV201 = stepProfile(T, 5, 0, 1);

out = simulatePlant(P, U, T);
[t, flow] = getSeries(out, 'sim_P201_Flow_Lps');
[~, position] = getSeries(out, 'sim_XV201_Position');

closedFlow = max(flow(t < 4.9));
assert(closedFlow < P.test.valveClosedFlowTolerance_Lps, ...
    'Flow passed through the closed treatment outlet valve.');
assert(position(end) > 0.99, 'Valve did not reach the open position.');
assert(flow(end) > 39.0, 'P-201 flow did not reach the expected open-valve value.');

detail = sprintf('Closed flow %.4f L/s; final valve %.3f; final flow %.2f L/s.', ...
    closedFlow, position(end), flow(end));
end

function detail = testPressure()
P = initialize_water_plant(false);
M = derive_plant_models(P, false);
T = 40;
stepTime = 20;
U = create_default_inputs(T, false);
U.raw_inflow = holdProfile(T, P.pressure.highDemand_Lps);
U.demand_flow = stepProfile(T, stepTime, ...
    P.pressure.nominalDemand_Lps, P.pressure.highDemand_Lps);
U.cmd_P301A = stepProfile(T, stepTime, M.pressure.nominalTotalSpeed_pct, 100);
U.cmd_P301B = stepProfile(T, stepTime, 0, M.pressure.highDemandTotalSpeed_pct - 100);

out = simulatePlant(P, U, T);
[t, pressure] = getSeries(out, 'sim_Pressure_PV_kPa');

post = t >= stepTime;
minimumPressure = min(pressure(post));
finalError = abs(pressure(end) - P.pressure.setpoint_kPa);
settling = settlingTime(t, pressure, stepTime, P.pressure.setpoint_kPa, ...
    P.pressure.acceptanceBand_kPa);

assert(minimumPressure >= P.pressure.minimumAllowed_kPa, ...
    'Pressure fell below the minimum allowed value.');
assert(finalError <= P.pressure.acceptanceBand_kPa, ...
    'Final pressure is outside the acceptance band.');
assert(settling <= P.pressure.recoveryTimeLimit_s, ...
    'Pressure recovery exceeded the time limit.');

detail = sprintf('Minimum %.2f kPa; final error %.2f kPa; recovery %.2f s.', ...
    minimumPressure, finalError, settling);
end

function detail = testConcentration()
P = initialize_water_plant(false);
P.T201.initialLevel_pct = 80;
P.T201.initialVolume_m3 = P.T201.maxVolume_m3 * 0.80;
P.T201.initialConcentration_mgL = 0;
T = 75;
U = create_default_inputs(T, false);
U.demand_flow = holdProfile(T, 0);
U.cmd_M201 = holdProfile(T, 1);
U.cmd_DP201 = pulseProfile(T, 0, 64, 100);

out = simulatePlant(P, U, T);
[~, actual] = getSeries(out, 'sim_Concentration_Actual_mgL');
[~, pv] = getSeries(out, 'sim_Concentration_PV_mgL');

assert(abs(actual(end) - P.concentration.setpoint_mgL) <= ...
    P.test.concentrationTolerance_mgL, ...
    'Actual concentration did not reach the target mass-balance value.');
assert(pv(end) >= P.concentration.acceptLow_mgL && ...
       pv(end) <= P.concentration.acceptHigh_mgL, ...
    'Measured concentration is outside the acceptance band.');

detail = sprintf('Actual %.3f mg/L; measured %.3f mg/L.', actual(end), pv(end));
end

function detail = testMassBalance()
P = initialize_water_plant(false);
T = 60;
U = create_default_inputs(T, false);
U.raw_inflow = holdProfile(T, 25);
U.demand_flow = holdProfile(T, 20);
U.cmd_P101A = holdProfile(T, 50);
U.cmd_P201 = holdProfile(T, 30);
U.cmd_XV201 = holdProfile(T, 1);

out = simulatePlant(P, U, T);
[~, v1] = getSeries(out, 'sim_T101_Volume_m3');
[~, v2] = getSeries(out, 'sim_T201_Volume_m3');
[~, v3] = getSeries(out, 'sim_T301_Volume_m3');
[tDemand, deliveredDemand] = getSeries(out, 'sim_DeliveredDemand_Lps');

initialTotal = P.T101.initialVolume_m3 + P.T201.initialVolume_m3 + P.T301.initialVolume_m3;
finalTotal = v1(end) + v2(end) + v3(end);
externalIn = 25*T/1000;
externalOut = trapz(tDemand, deliveredDemand)/1000;
expectedFinal = initialTotal + externalIn - externalOut;
error_m3 = abs(finalTotal - expectedFinal);

assert(error_m3 <= P.test.massBalanceTolerance_m3, ...
    'Total-water mass balance error exceeds tolerance.');
detail = sprintf('Mass-balance error %.6f m^3.', error_m3);
end

function detail = testBounds()
P = initialize_water_plant(false);
P.T101.initialLevel_pct = 99;
P.T101.initialVolume_m3 = 0.99*P.T101.maxVolume_m3;
P.T301.initialLevel_pct = 1;
P.T301.initialVolume_m3 = 0.01*P.T301.maxVolume_m3;
T = 30;
U = create_default_inputs(T, false);
U.raw_inflow = holdProfile(T, 200);
U.demand_flow = holdProfile(T, 200);

out = simulatePlant(P, U, T);
[~, l1] = getSeries(out, 'sim_T101_Level_pct');
[~, l2] = getSeries(out, 'sim_T201_Level_pct');
[~, l3] = getSeries(out, 'sim_T301_Level_pct');
levels = [l1(:); l2(:); l3(:)];

tol = P.test.levelNumericalTolerance_pct;
assert(min(levels) >= -tol, 'A tank level became negative.');
assert(max(levels) <= 100+tol, 'A tank level exceeded 100%%.');
detail = sprintf('Observed level range %.6f%% to %.6f%%.', min(levels), max(levels));
end


function detail = testSensorLag()
P = initialize_water_plant(false);
T = 15;
U = create_default_inputs(T, false);
U.raw_inflow = holdProfile(T, P.pressure.nominalDemand_Lps);
U.demand_flow = holdProfile(T, P.pressure.nominalDemand_Lps);
U.cmd_P301A = stepProfile(T, 2, 0, 100);

out = simulatePlant(P, U, T);
[t, actual] = getSeries(out, 'sim_Pressure_Actual_kPa');
[~, measured] = getSeries(out, 'sim_Pressure_PV_kPa');
idx = find(t >= 2, 1, 'first');
window = idx:min(idx+round(0.3/P.sim.baseStep_s), numel(t));
lagEvidence = max(abs(actual(window)-measured(window)));
assert(lagEvidence > 0.1, 'Pressure sensor did not exhibit measurable lag.');
assert(abs(actual(end)-measured(end)) < 2.0, ...
    'Pressure sensor did not converge toward actual pressure.');
detail = sprintf('Peak early actual/PV separation %.3f kPa.', lagEvidence);
end

function out = simulatePlant(P, U, stopTime)
model = 'Water_Treatment_Plant';
if ~bdIsLoaded(model)
    load_system(model);
end
simIn = Simulink.SimulationInput(model);
simIn = simIn.setVariable('P', P);
fields = fieldnames(U);
for k = 1:numel(fields)
    simIn = simIn.setVariable(fields{k}, U.(fields{k}));
end
simIn = simIn.setModelParameter('StopTime', num2str(stopTime), ...
    'ReturnWorkspaceOutputs', 'on');
out = sim(simIn);
end

function [t, y] = getSeries(out, variableName)
ts = out.get(variableName);
if isa(ts, 'timeseries')
    t = ts.Time;
    y = squeeze(ts.Data);
else
    error('WaterProject:UnexpectedLogFormat', ...
        'Expected timeseries output for %s.', variableName);
end
end

function profile = holdProfile(stopTime, value)
profile = [0 value; stopTime value];
end

function profile = stepProfile(stopTime, stepTime, beforeValue, afterValue)
epsTime = max(eps(stepTime), 1e-9);
profile = [0 beforeValue; stepTime-epsTime beforeValue; stepTime afterValue; stopTime afterValue];
end

function profile = pulseProfile(stopTime, startTime, endTime, value)
epsStart = max(eps(max(startTime,1)), 1e-9);
epsEnd = max(eps(endTime), 1e-9);
if startTime <= 0
    profile = [0 value; endTime-epsEnd value; endTime 0; stopTime 0];
else
    profile = [0 0; startTime-epsStart 0; startTime value; ...
        endTime-epsEnd value; endTime 0; stopTime 0];
end
end

function value = settlingTime(t, y, stepTime, target, band)
idx = find(t >= stepTime);
value = inf;
for k = 1:numel(idx)
    j = idx(k);
    if all(abs(y(j:end)-target) <= band)
        value = t(j)-stepTime;
        return;
    end
end
end

function results = executeTests(names, funcs)
N = numel(names);
Pass = false(N,1);
Details = cell(N,1);
for k = 1:N
    try
        Details{k} = funcs{k}();
        Pass(k) = true;
        fprintf('PASS: %s\n', names{k});
    catch ME
        Pass(k) = false;
        Details{k} = ME.message;
        fprintf('FAIL: %s\n      %s\n', names{k}, ME.message);
    end
end
results = table(names, Pass, Details, 'VariableNames', {'Test','Pass','Details'});
end

function saveResults(results, fileName)
matlabDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(fileparts(matlabDir));
folder = fullfile(projectRoot, '06_TESTING', 'Test_Results');
if ~exist(folder, 'dir'), mkdir(folder); end
writetable(results, fullfile(folder, fileName));
end
