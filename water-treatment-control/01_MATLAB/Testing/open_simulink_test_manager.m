function open_simulink_test_manager()
% open test manager when simulink test is installed
s=check_toolboxes(false);
if ~s.SimulinkTest
    error('WaterProject:MissingSimulinkTest', ...
        'Simulink Test is optional and is not installed. Core tests still work.');
end
sltest.testmanager.view;
end
