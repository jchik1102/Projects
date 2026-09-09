function results = validate_openplc_integrated_model(model, printSummary)
% static structure check for the v0.4 model

if nargin < 1 || isempty(model)
    model = 'Water_Treatment_Plant_OpenPLC_v0_4';
end
if nargin < 2
    printSummary = true;
end

if endsWith(model, '.slx') || contains(model, filesep)
    load_system(model);
    [~, modelName] = fileparts(model);
else
    modelName = model;
    if ~bdIsLoaded(modelName)
        load_system(modelName);
    end
end

requiredBlocks = { ...
    'OpenPLC_Communication', ...
    'OpenPLC_Communication/OpenPLCModbusBridge', ...
    'OpenPLC_Measurements', ...
    'OpenPLC_Feedback', ...
    'Log_OpenPLC_Measurements', ...
    'Log_OpenPLC_Feedback', ...
    'Log_OpenPLC_Diagnostics', ...
    'OpenPLC_Analog_Demux', ...
    'OpenPLC_Digital_Demux', ...
    'PLC_P101A_Command', ...
    'PLC_P101B_Command', ...
    'PLC_P201_Command', ...
    'PLC_P301A_Command', ...
    'PLC_P301B_Command', ...
    'PLC_XV201_Command'};

blockPresent = false(numel(requiredBlocks), 1);
for k = 1:numel(requiredBlocks)
    blockPresent(k) = ...
        getSimulinkBlockHandle([modelName '/' requiredBlocks{k}]) ~= -1;
end

standaloneCommandSources = { ...
    'cmd_P101A', 'cmd_P101B', 'cmd_P201', 'cmd_P301A', ...
    'cmd_P301B', 'cmd_DP201', 'cmd_M201', 'cmd_XV201'};
commandSourceRemoved = false(numel(standaloneCommandSources), 1);
for k = 1:numel(standaloneCommandSources)
    commandSourceRemoved(k) = ...
        getSimulinkBlockHandle( ...
            [modelName '/' standaloneCommandSources{k}]) == -1;
end

systemBlock = [modelName '/OpenPLC_Communication/OpenPLCModbusBridge'];
systemClassCorrect = false;
interpretedExecution = false;
if getSimulinkBlockHandle(systemBlock) ~= -1
    systemClassCorrect = strcmp( ...
        get_param(systemBlock, 'System'), 'OpenPLCModbusBridge');
    interpretedExecution = strcmpi( ...
        get_param(systemBlock, 'SimulateUsing'), 'Interpreted execution');
end

discreteStateSpecificationValid = false;
try
    bridge = OpenPLCModbusBridge();
    stateNames = { ...
        'Heartbeat', ...
        'LastAIT201', ...
        'ExchangeCount', ...
        'ReconnectCountdown', ...
        'WarningIssued'};
    stateChecks = false(size(stateNames));
    for k = 1:numel(stateNames)
        [stateSize, stateDataType, stateComplexity] = ...
            getDiscreteStateSpecification(bridge, stateNames{k});
        stateChecks(k) = isequal(stateSize, [1 1]) && ...
            strcmp(char(stateDataType), 'double') && ...
            isequal(stateComplexity, false);
    end
    discreteStateSpecificationValid = all(stateChecks);
catch
    discreteStateSpecificationValid = false;
end

outputSizeSpecificationValid = false;
try
    [analogSize, digitalSize, faultSize, commSize, diagnosticSize] = ...
        OpenPLCModbusBridge.outputPortSizes();
    outputSizeSpecificationValid = ...
        isequal(analogSize, [1 6]) && ...
        isequal(digitalSize, [1 7]) && ...
        isequal(faultSize, [1 4]) && ...
        isequal(commSize, [1 1]) && ...
        isequal(diagnosticSize, [1 48]);
catch
    outputSizeSpecificationValid = false;
end

Check = [ ...
    "All required integration blocks exist"
    "Standalone actuator sources were removed"
    "MATLAB System block uses OpenPLCModbusBridge"
    "MATLAB System block uses interpreted execution"
    "Bridge discrete states are specified for Simulink"
    "Bridge output sizes match the returned row vectors"
    "Industrial Communication Toolbox is available"];
Pass = [ ...
    all(blockPresent)
    all(commandSourceRemoved)
    systemClassCorrect
    interpretedExecution
    discreteStateSpecificationValid
    outputSizeSpecificationValid
    logical(exist('modbus', 'file'))];
Details = [ ...
    sprintf('%d of %d blocks present', sum(blockPresent), numel(blockPresent))
    sprintf('%d of %d sources removed', sum(commandSourceRemoved), numel(commandSourceRemoved))
    string(systemClassCorrect)
    string(interpretedExecution)
    string(discreteStateSpecificationValid)
    string(outputSizeSpecificationValid)
    string(logical(exist('modbus', 'file')))];

results = table(Check, Pass, Details);

if printSummary
    fprintf('\n============================================================\n');
    fprintf(' OPENPLC INTEGRATED MODEL STATIC CHECK\n');
    fprintf('============================================================\n');
    disp(results);
end

assert(all(results.Pass), ...
    'The generated OpenPLC integrated model failed its static check.');
end
