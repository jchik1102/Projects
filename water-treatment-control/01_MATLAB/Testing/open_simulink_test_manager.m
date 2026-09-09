function open_simulink_test_manager()
%OPEN_SIMULINK_TEST_MANAGER Open Test Manager when Simulink Test is installed.
s=check_toolboxes(false);
if ~s.SimulinkTest
    error('WaterProject:MissingSimulinkTest', ...
        'Simulink Test is optional and is not installed. Core tests still work.');
end
sltest.testmanager.view;
end
