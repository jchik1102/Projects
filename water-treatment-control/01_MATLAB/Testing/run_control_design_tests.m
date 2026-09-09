function results = run_control_design_tests()
%RUN_CONTROL_DESIGN_TESTS Verify toolbox-based pressure and concentration PI designs.

fprintf('\n============================================================\n');
fprintf(' STAGE 2 CONTROL SYSTEM TOOLBOX DESIGN TESTS\n');
fprintf('============================================================\n');

names = { ...
    'Pressure continuous closed-loop stability'; ...
    'Pressure robustness and reference response'; ...
    'Pressure demand-step rejection'; ...
    'Pressure discrete implementation stability'; ...
    'Concentration continuous closed-loop stability'; ...
    'Concentration robustness and response'; ...
    'Concentration discrete implementation stability'};
funcs = {@pressureStability; @pressurePerformance; @pressureDisturbance; ...
    @pressureDiscrete; @concentrationStability; @concentrationPerformance; ...
    @concentrationDiscrete};
results = execute_test_group(names, funcs, 'Stage2_Control_Design.csv');
assert_all_tests_pass(results, 'Stage 2 controller design');
end

function [P,C] = data()
P=initialize_water_plant(false); M=derive_plant_models(P,false);
C=design_all_controllers(P,M,false);
end

function detail=pressureStability()
[~,C]=data(); assert(isstable(C.pressure.referenceClosedLoop));
assert(all(real(pole(C.pressure.referenceClosedLoop))<0));
detail=sprintf('Closed-loop poles have maximum real part %.5f.', ...
    max(real(pole(C.pressure.referenceClosedLoop))));
end
function detail=pressurePerformance()
[P,C]=data(); m=C.pressure.metrics;
assert(m.phaseMargin_deg>=P.controller.pressure.minimumPhaseMargin_deg);
assert(m.referenceSettlingTime_s<=P.controller.pressure.maximumReferenceSettling_s);
assert(m.referenceOvershoot_pct<=P.controller.pressure.maximumOvershoot_pct);
detail=sprintf('PM %.1f deg; settling %.2f s; overshoot %.2f%%.', ...
    m.phaseMargin_deg,m.referenceSettlingTime_s,m.referenceOvershoot_pct);
end
function detail=pressureDisturbance()
[P,C]=data(); m=C.pressure.metrics;
assert(m.minimumPressureDemandStep_kPa>=P.pressure.minimumAllowed_kPa);
assert(m.demandRecoveryTime_s<=P.pressure.recoveryTimeLimit_s);
detail=sprintf('Minimum %.2f kPa; recovery %.2f s.', ...
    m.minimumPressureDemandStep_kPa,m.demandRecoveryTime_s);
end
function detail=pressureDiscrete()
[~,C]=data(); q=C.pressure.metrics.maximumDiscretePoleMagnitude;
assert(q<1); detail=sprintf('Maximum discrete pole magnitude %.6f.',q);
end
function detail=concentrationStability()
[~,C]=data(); assert(isstable(C.concentration.referenceClosedLoop));
assert(all(real(pole(C.concentration.referenceClosedLoop))<0));
detail=sprintf('Closed-loop poles have maximum real part %.5f.', ...
    max(real(pole(C.concentration.referenceClosedLoop))));
end
function detail=concentrationPerformance()
[P,C]=data(); m=C.concentration.metrics;
assert(m.phaseMargin_deg>=P.controller.concentration.minimumPhaseMargin_deg);
assert(m.referenceSettlingTime_s<=P.controller.concentration.maximumSettling_s);
assert(m.referenceOvershoot_pct<=P.controller.concentration.maximumOvershoot_pct);
detail=sprintf('PM %.1f deg; settling %.2f s; overshoot %.2f%%.', ...
    m.phaseMargin_deg,m.referenceSettlingTime_s,m.referenceOvershoot_pct);
end
function detail=concentrationDiscrete()
[~,C]=data(); q=C.concentration.metrics.maximumDiscretePoleMagnitude;
assert(q<1); detail=sprintf('Maximum discrete pole magnitude %.6f.',q);
end
