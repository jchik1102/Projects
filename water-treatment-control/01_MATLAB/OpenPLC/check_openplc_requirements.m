function report = check_openplc_requirements()
%CHECK_OPENPLC_REQUIREMENTS Verify software needed for OpenPLC integration.
products = ver;
names = string({products.Name});
report = table(["MATLAB";"Simulink";"Control System Toolbox";"Industrial Communication Toolbox"], ...
    [any(names=="MATLAB"); any(names=="Simulink"); any(names=="Control System Toolbox"); any(names=="Industrial Communication Toolbox")], ...
    'VariableNames',{'Product','Available'});
disp(report);
assert(report.Available(1) && report.Available(2) && report.Available(3), ...
    'MATLAB, Simulink, and Control System Toolbox are required.');
assert(report.Available(4), ['Industrial Communication Toolbox is required now for MATLAB/Simulink ', ...
    'Modbus TCP communication with OpenPLC. Install it from Add-On Explorer.']);

fprintf('\nChecking TCP ports...\n');
fprintf('  OpenPLC Editor/runtime API: localhost:8443\n');
fprintf('  OpenPLC Modbus TCP:        localhost:5020\n');
fprintf('Use: Test-NetConnection localhost -Port 5020 in PowerShell if connection fails.\n');
end
