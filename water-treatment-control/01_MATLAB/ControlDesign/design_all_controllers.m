function CTRL = design_all_controllers(P, M, printSummary)
%DESIGN_ALL_CONTROLLERS Design both toolbox-based PI controllers.

if nargin < 1 || isempty(P), P = initialize_water_plant(false); end
if nargin < 2 || isempty(M), M = derive_plant_models(P, false); end
if nargin < 3, printSummary = true; end

CTRL.pressure = design_pressure_controller(P, M, false);
CTRL.concentration = design_concentration_controller(P, M, false);

if printSummary
    fprintf('\nControl System Toolbox PI designs:\n');
    fprintf('  Pressure:      Kp %.6f, Ki %.6f, PM %.1f deg\n', ...
        CTRL.pressure.Kp, CTRL.pressure.Ki, ...
        CTRL.pressure.metrics.phaseMargin_deg);
    fprintf('  Concentration: Kp %.6f, Ki %.6f, PM %.1f deg\n', ...
        CTRL.concentration.Kp, CTRL.concentration.Ki, ...
        CTRL.concentration.metrics.phaseMargin_deg);
end
end
