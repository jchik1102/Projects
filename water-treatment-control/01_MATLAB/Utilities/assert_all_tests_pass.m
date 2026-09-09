function assert_all_tests_pass(results, stageName)
% raise a clear error when any test in a stage fails
if ~all(results.Pass)
    failed=string(results.Test(~results.Pass));
    error('WaterProject:StageFailed','%s failed: %s',stageName,strjoin(failed,', '));
end
fprintf('%s: PASSED\n',stageName);
end
