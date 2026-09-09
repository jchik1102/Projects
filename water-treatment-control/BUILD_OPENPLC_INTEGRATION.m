% build and open the generated simulink/openplc closed-loop model

START_HERE
build_openplc_integrated_model(true)

fprintf('\nThe integrated model is ready.\n');
fprintf('With the OpenPLC Runtime and PLC program running, execute:\n');
fprintf('  run_openplc_simulink_demo\n\n');
fprintf('After the first-loop demo has passed, Stage 5 is:\n');
fprintf('  RUN_STAGE5_FULL_CONTROL\n\n');
