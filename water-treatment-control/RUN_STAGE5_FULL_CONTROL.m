% RUN_STAGE5_FULL_CONTROL
% Build/validate the live OpenPLC model and execute the approximately
% 4.5-minute Stage 5 commissioning test.

BUILD_OPENPLC_INTEGRATION
stage5_results = run_stage5_full_control_integration()
