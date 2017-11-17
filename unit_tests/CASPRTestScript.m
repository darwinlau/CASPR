function num_test_failed = CASPRTestScript(mode)
    
    if(nargin == 0)
        mode = TestModeType.DEFAULT;
    end
    
    num_test_failed = 0;
    CASPR_homepath = CASPR_configuration.LoadHomePath();
    warning off;

    % Running of the tests
    if((mode == TestModeType.MODEL_CONFIG)||(mode == TestModeType.DEFAULT)||(mode == TestModeType.ALL))
        CASPR_config_tests = matlab.unittest.TestSuite.fromFolder([CASPR_homepath '/unit_tests/ModelConfig'], 'IncludingSubfolders', true);
        CASPR_config_testresults = CASPR_config_tests.run();
    end
    if((mode == TestModeType.MODEL_COMPONENTS)||(mode == TestModeType.DEFAULT)||(mode == TestModeType.ALL))
        CASPR_model_tests = matlab.unittest.TestSuite.fromFolder([CASPR_homepath '/unit_tests/Model'], 'IncludingSubfolders', true);
        CASPR_model_testresults = CASPR_model_tests.run();
    end
    if((mode == TestModeType.ANALYSIS)||(mode == TestModeType.DEFAULT)||(mode == TestModeType.ALL))
        CASPR_analysis_tests = matlab.unittest.TestSuite.fromFolder([CASPR_homepath '/unit_tests/Analysis'], 'IncludingSubfolders', true);
        CASPR_analysis_testresults = CASPR_analysis_tests.run();
    end
    if((mode == TestModeType.SCRIPTS)||(mode == TestModeType.ALL))
        CASPR_scripts_tests = matlab.unittest.TestSuite.fromFolder([CASPR_homepath '/unit_tests/Scripts'], 'IncludingSubfolders', true);
        CASPR_scripts_testresults = CASPR_scripts_tests.run();
    end
    
    warning on;    

    disp('----------------------------------------');
    disp('Summary of Unit Tests');
    disp('----------------------------------------');

    if((mode == TestModeType.MODEL_CONFIG)||(mode == TestModeType.DEFAULT)||(mode == TestModeType.ALL))
        disp('Test results for ModelConfig:');
        tr = CASPR_config_testresults;
        fprintf('%d Passed, %d Failed, %d Incomplete, %f seconds testing time.\n', nnz([tr.Passed]), nnz([tr.Failed]), nnz([tr.Incomplete]), sum([tr.Duration]));
        if any([tr.Failed])
            CASPR_config_testfailed = tr([tr.Failed]);
            num_test_failed = num_test_failed + nnz([tr.Passed]);
            table(CASPR_config_testfailed)
        end
    end
    if((mode == TestModeType.MODEL_COMPONENTS)||(mode == TestModeType.DEFAULT)||(mode == TestModeType.ALL))
        disp('Test results for Model:');
        tr = CASPR_model_testresults;
        fprintf('%d Passed, %d Failed, %d Incomplete, %f seconds testing time.\n', nnz([tr.Passed]), nnz([tr.Failed]), nnz([tr.Incomplete]), sum([tr.Duration]));
        if any([tr.Failed])
            CASPR_model_testfailed = tr([tr.Failed]);
            num_test_failed = num_test_failed + nnz([tr.Passed]);
            table(CASPR_model_testfailed)
        end
    end
    if((mode == TestModeType.ANALYSIS)||(mode == TestModeType.DEFAULT)||(mode == TestModeType.ALL))
        disp('Test results for Analysis:');
        tr = CASPR_analysis_testresults;
        fprintf('%d Passed, %d Failed, %d Incomplete, %f seconds testing time.\n', nnz([tr.Passed]), nnz([tr.Failed]), nnz([tr.Incomplete]), sum([tr.Duration]));
        if any([tr.Failed])
            CASPR_analysis_testfailed = tr([tr.Failed]);
            num_test_failed = num_test_failed + nnz([tr.Passed]);
            table(CASPR_analysis_testfailed)
        end
    end
    if((mode == TestModeType.SCRIPTS)||(mode == TestModeType.ALL))
        disp('Test results for Scripts:');
        tr = CASPR_scripts_testresults;
        fprintf('%d Passed, %d Failed, %d Incomplete, %f seconds testing time.\n', nnz([tr.Passed]), nnz([tr.Failed]), nnz([tr.Incomplete]), sum([tr.Duration]));
        if any([tr.Failed])
            CASPR_analysis_testfailed = tr([tr.Failed]);
            num_test_failed = num_test_failed + nnz([tr.Passed]);
            table(CASPR_analysis_testfailed)
        end
    end
end