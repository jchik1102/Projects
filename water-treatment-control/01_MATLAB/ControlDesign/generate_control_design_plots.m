function generate_control_design_plots()
%GENERATE_CONTROL_DESIGN_PLOTS Save controller response and margin plots.

P = initialize_water_plant(false);
M = derive_plant_models(P, false);
CTRL = design_all_controllers(P, M, false);

thisDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(fileparts(thisDir));
outDir = fullfile(projectRoot, '06_TESTING', 'Plots');
if ~exist(outDir, 'dir'), mkdir(outDir); end

f = figure('Visible','off','Name','Pressure Reference Response');
step(CTRL.pressure.referenceClosedLoop, 25); grid on;
title('Pressure PI Closed-Loop Reference Response');
ylabel('Pressure deviation / setpoint deviation');
exportgraphics(f, fullfile(outDir, 'Pressure_PI_Reference_Response.png'), 'Resolution', 180);
close(f);

f = figure('Visible','off','Name','Pressure Loop Margins');
margin(CTRL.pressure.loop); grid on;
title('Pressure PI Open-Loop Margins');
exportgraphics(f, fullfile(outDir, 'Pressure_PI_Margins.png'), 'Resolution', 180);
close(f);

f = figure('Visible','off','Name','Concentration Reference Response');
step(P.concentration.setpoint_mgL*CTRL.concentration.referenceClosedLoop, 160); grid on;
title('Concentration PI Closed-Loop Reference Response');
ylabel('Concentration (mg/L)');
exportgraphics(f, fullfile(outDir, 'Concentration_PI_Reference_Response.png'), 'Resolution', 180);
close(f);

f = figure('Visible','off','Name','Concentration Loop Margins');
margin(CTRL.concentration.loop); grid on;
title('Concentration PI Open-Loop Margins');
exportgraphics(f, fullfile(outDir, 'Concentration_PI_Margins.png'), 'Resolution', 180);
close(f);

fprintf('Control-design plots saved to %s\n', outDir);
end
