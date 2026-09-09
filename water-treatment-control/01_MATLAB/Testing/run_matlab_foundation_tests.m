function results = run_matlab_foundation_tests()
%RUN_MATLAB_FOUNDATION_TESTS Validate parameters, calculations, and maps.

fprintf('\n============================================================\n');
fprintf(' STAGE 1 MATLAB FOUNDATION TESTS\n');
fprintf('============================================================\n');

names = { ...
    'Required products available'; ...
    'Parameter structure sanity'; ...
    'Analytical operating points'; ...
    'LTI plant stability and properness'; ...
    'Register-map uniqueness and export'; ...
    'Controller parameter export'};
funcs = {@testProducts; @testParameters; @testDerivations; ...
    @testLTIModels; @testRegisterMap; @testControllerExport};
results = execute_test_group(names, funcs, 'Stage1_MATLAB_Foundation.csv');
assert_all_tests_pass(results, 'Stage 1 MATLAB foundation');
end

function detail = testProducts()
s = check_toolboxes(false);
assert(s.RequiredReady, 'One or more required MathWorks products are unavailable.');
detail = sprintf('MATLAB %s; Simulink and Control System Toolbox available.', s.Release);
end

function detail = testParameters()
P = initialize_water_plant(false);
assert(all([P.T101.maxVolume_m3 P.T201.maxVolume_m3 P.T301.maxVolume_m3] > 0));
assert(P.T201.batchFillSetpoint_pct > P.T201.initialLevel_pct);
assert(P.concentration.acceptLow_mgL < P.concentration.setpoint_mgL);
assert(P.concentration.setpoint_mgL < P.concentration.acceptHigh_mgL);
assert(P.pressure.minimumAllowed_kPa < P.pressure.setpoint_kPa);
assert(P.controller.pressure.sampleTime_s >= P.sim.baseStep_s);
assert(P.controller.concentration.sampleTime_s >= P.controller.pressure.sampleTime_s);
detail = 'Physical values, thresholds, and execution periods are internally consistent.';
end

function detail = testDerivations()
P = initialize_water_plant(false); M = derive_plant_models(P,false);
assert(M.T201.timeToBatchSetpointFromInitial_s > 0);
assert(M.pressure.nominalTotalSpeed_pct > 0 && M.pressure.nominalTotalSpeed_pct < 100);
assert(M.pressure.highDemandTotalSpeed_pct > 100 && M.pressure.highDemandTotalSpeed_pct < 200);
assert(abs(M.concentration.targetChemicalMass_mg-57600) < 1e-9);
assert(abs(M.concentration.fullOutputDoseTime_s-64) < 1e-9);
detail = sprintf('Nominal booster %.2f%%; high-demand %.2f%%; dose %.1f s.', ...
    M.pressure.nominalTotalSpeed_pct, M.pressure.highDemandTotalSpeed_pct, ...
    M.concentration.fullOutputDoseTime_s);
end

function detail = testLTIModels()
P = initialize_water_plant(false); M = derive_plant_models(P,false); %#ok<NASGU>
models = {M.pressure.pump,M.pressure.demand,M.pressure.sensor, ...
    M.concentration.plant,M.concentration.sensor,M.actuator.P101,M.actuator.XV201};
for k = 1:numel(models)
    assert(isstable(models{k}), 'An LTI plant model is unstable.');
    assert(isproper(models{k}), 'An LTI plant model is improper.');
end
detail = sprintf('%d LTI plant/actuator models are stable and proper.', numel(models));
end

function detail = testRegisterMap()
R = define_register_map([],false);
assert(height(R)==48); assert(numel(unique(R.Address))==height(R));
assert(numel(unique(R.Tag))==height(R));
detail = sprintf('%d unique external tags exported.', height(R));
end

function detail = testControllerExport()
P=initialize_water_plant(false); M=derive_plant_models(P,false);
C=design_all_controllers(P,M,false); T=export_controller_parameters(P,C,false);
assert(height(T)==2); assert(all(T.Kp>0)); assert(all(T.Ki_per_s>0));
detail = 'Two positive-gain PLC-ready PI parameter sets exported.';
end
