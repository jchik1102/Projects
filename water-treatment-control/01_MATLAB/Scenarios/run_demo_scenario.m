function outputs = run_demo_scenario()
% run both closed-loop controller demonstrations and plot

startup_project(); build_controller_test_models(false);
P=evalin('base','P'); M=evalin('base','MODELS'); CTRL=evalin('base','CTRL');
pressure_demand_delta=[0 0;20-1e-9 0;20 30;60 30];
simIn=Simulink.SimulationInput('Pressure_Controller_Test');
simIn=simIn.setVariable('P',P);
simIn=simIn.setVariable('MODELS',M);
simIn=simIn.setVariable('CTRL',CTRL);
simIn=simIn.setVariable('pressure_demand_delta',pressure_demand_delta);
outputs.pressure=sim(simIn.setModelParameter('StopTime','60','ReturnWorkspaceOutputs','on'));

simIn=Simulink.SimulationInput('Concentration_Controller_Test');
simIn=simIn.setVariable('P',P);
simIn=simIn.setVariable('MODELS',M);
simIn=simIn.setVariable('CTRL',CTRL);
outputs.concentration=sim(simIn.setModelParameter('StopTime','160','ReturnWorkspaceOutputs','on'));

p=outputs.pressure.get('test_pressure_pv_kPa'); u=outputs.pressure.get('test_pressure_command_pct');
c=outputs.concentration.get('test_concentration_pv_mgL'); d=outputs.concentration.get('test_dose_command_pct');

figure('Name','Pressure controller demonstration');
plot(p.Time,p.Data,'LineWidth',1.2); yline(P.pressure.setpoint_kPa,'--');
yline(P.pressure.minimumAllowed_kPa,':'); grid on; xlabel('Time (s)'); ylabel('kPa');
title('Pressure response to 30 L/s demand increase');
figure('Name','Booster demand'); plot(u.Time,u.Data,'LineWidth',1.2); grid on;
xlabel('Time (s)'); ylabel('Total speed demand (%)'); title('Pressure PI output');
figure('Name','Concentration controller demonstration');
plot(c.Time,c.Data,'LineWidth',1.2); yline(P.concentration.acceptLow_mgL,'--');
yline(P.concentration.setpoint_mgL,'-.'); yline(P.concentration.acceptHigh_mgL,'--');
grid on; xlabel('Time (s)'); ylabel('mg/L'); title('Concentration PI response');
figure('Name','Dosing demand'); plot(d.Time,d.Data,'LineWidth',1.2); grid on;
xlabel('Time (s)'); ylabel('Dosing output (%)'); title('Concentration PI output');
end
