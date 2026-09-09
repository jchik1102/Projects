function state = startup_project()
%STARTUP_PROJECT Configure paths, validate products, and load design data.

thisFile = mfilename('fullpath');
projectRoot = fileparts(fileparts(fileparts(thisFile)));
addpath(genpath(fullfile(projectRoot, '01_MATLAB')));
addpath(fullfile(projectRoot, '02_SIMULINK'));

fprintf('\n============================================================\n');
fprintf(' WATER TREATMENT CONTROL PROJECT - FULL CONTROL v0.5.2\n');
fprintf('============================================================\n');
fprintf('Project root: %s\n\n', projectRoot);

products = check_toolboxes(true);
P = initialize_water_plant(true);
MODELS = derive_plant_models(P, true);
CTRL = design_all_controllers(P, MODELS, false);
U = create_default_inputs(P.sim.defaultStopTime_s, true);

assignin('base', 'P', P);
assignin('base', 'MODELS', MODELS);
assignin('base', 'CTRL', CTRL);
assignin('base', 'U', U);

export_controller_parameters(P, CTRL, false);
define_register_map([], false);

state = struct('projectRoot', projectRoot, 'products', products, ...
    'P', P, 'MODELS', MODELS, 'CTRL', CTRL, 'U', U);

fprintf('\nInitialization complete.\n');
fprintf('Run run_all_tests to build the models and execute all checks.\n');
fprintf('Run generate_control_design_plots for controller plots.\n\n');
end
