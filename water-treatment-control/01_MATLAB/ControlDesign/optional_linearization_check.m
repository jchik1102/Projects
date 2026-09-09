function result = optional_linearization_check()
%OPTIONAL_LINEARIZATION_CHECK Demonstrate Simulink Control Design availability.
%
% The core project does not require this product. When installed, this
% function verifies that command-line linearization tools are available and
% records that the generated controller test models are candidates for exact
% model linearization in the next revision.

s=check_toolboxes(false);
if ~s.SimulinkControlDesign
    fprintf('Simulink Control Design is not installed. Optional check skipped.\n');
    result=struct('available',false,'passed',false,'message','Optional product not installed');
    return;
end
build_controller_test_models(false);
assert(exist('linearize','file')==2,'linearize was not found.');
assert(exist('linio','file')==2,'linio was not found.');
result=struct('available',true,'passed',true, ...
    'message','Linearization commands and generated models are available.');
fprintf('Optional Simulink Control Design readiness check: PASSED\n');
end
