function summary = run_all_tests()
% build all models and execute the complete v0.2 test suite

startup_project();
build_water_treatment_plant(true);
build_controller_test_models(true);

stages = cell(4,1);
stages{1} = run_matlab_foundation_tests();
stages{2} = run_control_design_tests();
stages{3} = run_simulink_plant_tests();
stages{4} = run_simulink_controller_tests();

names = ["MATLAB foundation";"Control design";"Simulink plant";"Simulink controllers"];
passed = cellfun(@(x) all(x.Pass), stages);
summary = table(names, passed, 'VariableNames', {'Stage','Pass'});
summary.AllPassed = repmat(all(passed),height(summary),1);

disp(summary);
fprintf('\n============================================================\n');
fprintf(' FINAL MATLAB/SIMULINK RESULT: %s\n', passText(all(passed)));
fprintf('============================================================\n\n');

thisDir=fileparts(mfilename('fullpath'));
projectRoot=fileparts(fileparts(thisDir));
writetable(summary,fullfile(projectRoot,'06_TESTING','Test_Results','Overall_Test_Summary.csv'));

if ~all(passed)
    error('WaterProject:TestsFailed','One or more MATLAB/Simulink stages failed.');
end
end
function txt=passText(v)
if v, txt='PASSED'; else, txt='FAILED'; end
end
