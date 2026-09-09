function results = run_simulink_controller_tests()
% verify generated pi controller test models

fprintf('\n============================================================\n');
fprintf(' STAGE 4 SIMULINK CLOSED-LOOP CONTROLLER TESTS\n');
fprintf('============================================================\n');

build_controller_test_models(false);
names = { ...
    'Pressure PI demand-step performance'; ...
    'Pressure PI output limits'; ...
    'Concentration PI reaches acceptance band'; ...
    'Concentration PI avoids high-high limit'; ...
    'Concentration PI output limits'};
funcs = {@pressureResponse; @pressureLimits; @concentrationResponse; ...
    @concentrationHighHigh; @concentrationLimits};
results = execute_test_group(names, funcs, 'Stage4_SIMULINK_Controllers.csv');
assert_all_tests_pass(results, 'Stage 4 Simulink controller validation');
end

function out = runPressure()
P=initialize_water_plant(false); M=derive_plant_models(P,false); CTRL=design_all_controllers(P,M,false);
pressure_demand_delta=[0 0; 20-1e-9 0; 20 P.pressure.highDemand_Lps-P.pressure.nominalDemand_Lps; 60 P.pressure.highDemand_Lps-P.pressure.nominalDemand_Lps];
simIn=Simulink.SimulationInput('Pressure_Controller_Test');
simIn=simIn.setVariable('P',P);
simIn=simIn.setVariable('MODELS',M);
simIn=simIn.setVariable('CTRL',CTRL);
simIn=simIn.setVariable('pressure_demand_delta',pressure_demand_delta);
out=sim(simIn.setModelParameter('StopTime','60','ReturnWorkspaceOutputs','on'));
end

function out = runConcentration()
P=initialize_water_plant(false); M=derive_plant_models(P,false); CTRL=design_all_controllers(P,M,false);
simIn=Simulink.SimulationInput('Concentration_Controller_Test');
simIn=simIn.setVariable('P',P);
simIn=simIn.setVariable('MODELS',M);
simIn=simIn.setVariable('CTRL',CTRL);
out=sim(simIn.setModelParameter('StopTime','160','ReturnWorkspaceOutputs','on'));
end

function detail=pressureResponse()
P=initialize_water_plant(false); out=runPressure();
[t,p]=series(out,'test_pressure_pv_kPa'); post=t>=20;
minimum=min(p(post)); recovery=recoveryTime(t,p,20,P.pressure.setpoint_kPa,P.pressure.acceptanceBand_kPa);
assert(minimum>=P.pressure.minimumAllowed_kPa);
assert(recovery<=P.pressure.recoveryTimeLimit_s);
assert(abs(p(end)-P.pressure.setpoint_kPa)<=P.pressure.acceptanceBand_kPa);
detail=sprintf('Minimum %.2f kPa; recovery %.2f s; final %.2f kPa.',minimum,recovery,p(end));
end
function detail=pressureLimits()
P=initialize_water_plant(false); out=runPressure(); [~,u]=series(out,'test_pressure_command_pct');
assert(min(u)>=P.controller.pressure.outputMin_pct-1e-6);
assert(max(u)<=P.controller.pressure.outputMax_pct+1e-6);
detail=sprintf('Command range %.2f%% to %.2f%%.',min(u),max(u));
end
function detail=concentrationResponse()
P=initialize_water_plant(false); out=runConcentration(); [t,c]=series(out,'test_concentration_pv_mgL');
firstStable=stableEntryTime(t,c,P.concentration.acceptLow_mgL,P.concentration.acceptHigh_mgL,P.concentration.stableTime_s);
assert(isfinite(firstStable),'Concentration never remained in the acceptance band.');
assert(firstStable<=P.controller.concentration.maximumSettling_s);
assert(c(end)>=P.concentration.acceptLow_mgL && c(end)<=P.concentration.acceptHigh_mgL);
detail=sprintf('Stable acceptance begins at %.2f s; final %.3f mg/L.',firstStable,c(end));
end
function detail=concentrationHighHigh()
P=initialize_water_plant(false); out=runConcentration(); [~,c]=series(out,'test_concentration_mgL');
assert(max(c)<P.concentration.highHigh_mgL);
detail=sprintf('Maximum concentration %.3f mg/L.',max(c));
end
function detail=concentrationLimits()
P=initialize_water_plant(false); out=runConcentration(); [~,u]=series(out,'test_dose_command_pct');
assert(min(u)>=P.controller.concentration.outputMin_pct-1e-6);
assert(max(u)<=P.controller.concentration.outputMax_pct+1e-6);
detail=sprintf('Dose command range %.2f%% to %.2f%%.',min(u),max(u));
end

function [t,y]=series(out,name)
ts=out.get(name); assert(isa(ts,'timeseries'),'Expected timeseries %s.',name);
t=ts.Time; y=squeeze(ts.Data);
end
function value=recoveryTime(t,y,stepTime,target,band)
value=inf; idx=find(t>=stepTime);
for k=1:numel(idx)
    j=idx(k); if all(abs(y(j:end)-target)<=band), value=t(j)-stepTime; return; end
end
end
function value=stableEntryTime(t,y,lo,hi,duration)
value=inf;
for k=1:numel(t)
    j=find(t>=t(k)+duration,1,'first');
    if ~isempty(j) && all(y(k:j)>=lo & y(k:j)<=hi), value=t(k); return; end
end
end
