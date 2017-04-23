function num_test_failed = CASPRTestScript()
    num_test_failed = 0;

    CASPR_homepath = CASPR_configuration.LoadHomePath();
    CASPR_config_tests = matlab.unittest.TestSuite.fromFolder([CASPR_homepath '/unit_tests/ModelConfig'], 'IncludingSubfolders', true);
    CASPR_model_tests = matlab.unittest.TestSuite.fromFolder([CASPR_homepath '/unit_tests/Model'], 'IncludingSubfolders', true);
    CASPR_analysis_tests = matlab.unittest.TestSuite.fromFolder([CASPR_homepath '/unit_tests/Analysis'], 'IncludingSubfolders', true);

    warning off;

    CASPR_config_testresults = CASPR_config_tests.run();
    CASPR_model_testresults = CASPR_model_tests.run();
    CASPR_analysis_testresults = CASPR_analysis_tests.run();

    warning on;

    disp('----------------------------------------');
    disp('Summary of Unit Tests');
    disp('----------------------------------------');

    disp('Test results for ModelConfig:');
    tr = CASPR_config_testresults;
    fprintf('%d Passed, %d Failed, %d Incomplete, %f seconds testing time.\n', nnz([tr.Passed]), nnz([tr.Failed]), nnz([tr.Incomplete]), sum([tr.Duration]));
    if any([tr.Failed])
        CASPR_config_testfailed = tr([tr.Failed]);
        num_test_failed = num_test_failed + nnz([tr.Passed]);
        table(CASPR_config_testfailed)
    end

    disp('Test results for Model:');
    tr = CASPR_model_testresults;
    fprintf('%d Passed, %d Failed, %d Incomplete, %f seconds testing time.\n', nnz([tr.Passed]), nnz([tr.Failed]), nnz([tr.Incomplete]), sum([tr.Duration]));
    if any([tr.Failed])
        CASPR_model_testfailed = tr([tr.Failed]);
        num_test_failed = num_test_failed + nnz([tr.Passed]);
        table(CASPR_model_testfailed)
    end

    disp('Test results for Analysis:');
    tr = CASPR_analysis_testresults;
    fprintf('%d Passed, %d Failed, %d Incomplete, %f seconds testing time.\n', nnz([tr.Passed]), nnz([tr.Failed]), nnz([tr.Incomplete]), sum([tr.Duration]));
    if any([tr.Failed])
        CASPR_analysis_testfailed = tr([tr.Failed]);
        num_test_failed = num_test_failed + nnz([tr.Passed]);
        table(CASPR_analysis_testfailed)
    end
end