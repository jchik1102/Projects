function results = execute_test_group(names, funcs, outputFile)
%EXECUTE_TEST_GROUP Run test functions and save a standardized result table.

N=numel(names); Pass=false(N,1); Details=cell(N,1);
for k=1:N
    try
        Details{k}=funcs{k}(); Pass(k)=true;
        fprintf('PASS: %s\n',names{k});
    catch ME
        Pass(k)=false; Details{k}=getReport(ME,'basic','hyperlinks','off');
        fprintf('FAIL: %s\n      %s\n',names{k},ME.message);
    end
end
results=table(names(:),Pass,Details,'VariableNames',{'Test','Pass','Details'});
disp(results);

thisDir=fileparts(mfilename('fullpath'));
projectRoot=fileparts(fileparts(thisDir));
outDir=fullfile(projectRoot,'06_TESTING','Test_Results');
if ~exist(outDir,'dir'),mkdir(outDir);end
writetable(results,fullfile(outDir,outputFile));
end
