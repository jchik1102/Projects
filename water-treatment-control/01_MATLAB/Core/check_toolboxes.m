function status = check_toolboxes(printSummary)
% verify required, optional, and future mathworks products

if nargin < 1
    printSummary = true;
end

products = ver;
names = string({products.Name});
releaseText = version('-release');

status.MATLAB = any(strcmpi(names, 'MATLAB'));
status.Simulink = any(strcmpi(names, 'Simulink'));
status.ControlSystemToolbox = any(strcmpi(names, 'Control System Toolbox'));
status.SimulinkControlDesign = any(strcmpi(names, 'Simulink Control Design'));
status.SimulinkTest = any(strcmpi(names, 'Simulink Test'));
status.IndustrialCommunicationToolbox = any(strcmpi(names, 'Industrial Communication Toolbox'));
status.Release = releaseText;
status.RequiredReady = status.MATLAB && status.Simulink && status.ControlSystemToolbox;

if printSummary
    fprintf('MathWorks product check (%s):\n', releaseText);
    fprintf('  MATLAB:                           %s [required]\n', passText(status.MATLAB));
    fprintf('  Simulink:                         %s [required]\n', passText(status.Simulink));
    fprintf('  Control System Toolbox:           %s [required]\n', passText(status.ControlSystemToolbox));
    fprintf('  Simulink Control Design:          %s [optional]\n', passText(status.SimulinkControlDesign));
    fprintf('  Simulink Test:                    %s [optional]\n', passText(status.SimulinkTest));
    fprintf('  Industrial Communication Toolbox:%s [required for OpenPLC/Stage 5]\n', ...
        [' ' passText(status.IndustrialCommunicationToolbox)]);
end

if ~status.RequiredReady
    missing = strings(0,1);
    if ~status.MATLAB, missing(end+1) = "MATLAB"; end %#ok<AGROW>
    if ~status.Simulink, missing(end+1) = "Simulink"; end %#ok<AGROW>
    if ~status.ControlSystemToolbox, missing(end+1) = "Control System Toolbox"; end %#ok<AGROW>
    error('WaterProject:MissingProducts', ...
        'Install or enable the required product(s): %s.', strjoin(missing, ', '));
end
end

function txt = passText(value)
if value
    txt = 'AVAILABLE';
else
    txt = 'NOT FOUND';
end
end
