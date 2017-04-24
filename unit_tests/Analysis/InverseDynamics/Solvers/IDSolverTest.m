% Base object for testing the IDObjective objects
%
% Author        : Darwin LAU
% Created       : 2016
% Description    :
classdef IDSolverTest < matlab.unittest.TestCase
    properties (ClassSetupParameter)
        model_config_type = struct('SCDM', 'test_SCDM', ...
            'MCDM', 'test_MCDM', ...
            'Active_passive_cables', 'test_active_passive_cables', ...
            'HCDM', 'test_HCDM');
    end
    
    properties
        modelObj;
    end
    
    properties (TestParameter)
        qp_solver_type = struct('MATLAB', ID_QP_SolverType.MATLAB, ...
            'MATLAB_warm_start', ID_QP_SolverType.MATLAB_ACTIVE_SET_WARM_START, ...
            'OptiToolbox_IPOPT', ID_QP_SolverType.OPTITOOLBOX_IPOPT, ...
            'OptiToolbox_OOQP', ID_QP_SolverType.OPTITOOLBOX_OOQP);
        lp_solver_type = struct('MATLAB', ID_LP_SolverType.MATLAB, ...
            'OptiToolbox_OOQP', ID_LP_SolverType.OPTITOOLBOX_OOQP, ...
            'OptiToolbox_LP', ID_LP_SolverType.OPTITOOLBOX_LP_SOLVE);
        infp_solver_type = struct('MATLAB', ID_LP_SolverType.MATLAB, ...
            'OptiToolbox_OOQP', ID_LP_SolverType.OPTITOOLBOX_OOQP, ...
            'OptiToolbox_LP', ID_LP_SolverType.OPTITOOLBOX_LP_SOLVE);
%         os_solver_type = struct('LP', ID_OS_SolverType.LP, ...
%             'Efficient_LP', ID_OS_SolverType.EFFICIENT_LP);
%         fp_solver_type = struct('NORM_1', ID_FP_SolverType.NORM_1, ...
%             'NORM_2', ID_FP_SolverType.NORM_2, ...
%             'CENTROID', ID_FP_SolverType.CENTROID);
        cf_solver_type = struct('CLOSED_FORM', ID_CF_SolverType.CLOSED_FORM, ...
            'IMPROVED_CLOSED_FORM', ID_CF_SolverType.IMPROVED_CLOSED_FORM, ...
            'PUNCTURE_METHOD', ID_CF_SolverType.PUNCTURE_METHOD, ...
            'IMPROVED_PUNCTURE_METHOD',ID_CF_SolverType.IMPROVED_PUNCTURE_METHOD);
    end
    
    methods (TestClassSetup)
        function setupModelObj(testCase, model_config_type)
            model_config = TestModelConfig(model_config_type);
            testCase.modelObj = model_config.getModel(model_config.defaultCableSetId);
        end
    end
    
    methods (Test)
        function testQPSolver(testCase, qp_solver_type)
            CASPR_log.Debug('Running IDSolverTest/testQPSolver');
            id_objective = IDObjectiveMinQuadCableForce(ones(1, testCase.modelObj.numActuators));
            id_solver = IDSolverQuadProg(testCase.modelObj, id_objective, qp_solver_type);
            [actuation_soln, Q_opt, id_exit_type] = id_solver.resolveFunction(testCase.modelObj);
            testCase.assertIDSolverOutput(actuation_soln, Q_opt, id_exit_type);
            CASPR_log.Debug('Done IDSolverTest/testQPSolver');
        end
        
        function testLPSolver(testCase, lp_solver_type)
            CASPR_log.Debug('Running IDSolverTest/testLPSolver');
            id_objective = IDObjectiveMinLinCableForce(ones(testCase.modelObj.numActuators, 1));
            id_solver = IDSolverLinProg(testCase.modelObj, id_objective, lp_solver_type);
            [actuation_soln, Q_opt, id_exit_type] = id_solver.resolveFunction(testCase.modelObj);
            testCase.assertIDSolverOutput(actuation_soln, Q_opt, id_exit_type);
            CASPR_log.Debug('Done IDSolverTest/testLPSolver');
        end
        
        function testInfPSolver(testCase, infp_solver_type)
            CASPR_log.Debug('Running IDSolverTest/testInfPSolver');
            id_objective = IDObjectiveMinInfCableForce(ones(testCase.modelObj.numActuators, 1));
            id_solver = IDSolverMinInfNorm(testCase.modelObj, id_objective, infp_solver_type);
            [actuation_soln, Q_opt, id_exit_type] = id_solver.resolveFunction(testCase.modelObj);
            testCase.assertIDSolverOutput(actuation_soln, Q_opt, id_exit_type);
            CASPR_log.Debug('Done IDSolverTest/testInfPSolver');
        end
        
%         function testOptimallySafeSolver(testCase, os_solver_type)
%             CASPR_log.Debug('Running IDSolverTest/testOptimallySafeSolver');
%             id_solver = IDSolverOptimallySafe(testCase.modelObj, 1, os_solver_type);
%             [cable_f, Q_opt, id_exit_type] = id_solver.resolveFunction(testCase.modelObj);
%             testCase.assertIDSolverOutput(cable_f, Q_opt, id_exit_type);
%             CASPR_log.Debug('Done IDSolverTest/testOptimallySafeSolver');
%         end
%         
%         function testFeasiblePolygon(testCase, fp_solver_type)
%             CASPR_log.Debug('Running IDSolverTest/testFeasiblePolygon');
%             id_solver = IDSolverFeasiblePolygon(testCase.modelObj, fp_solver_type);
%             [cable_f, Q_opt, id_exit_type] = id_solver.resolveFunction(testCase.modelObj);
%             testCase.assertIDSolverOutput(cable_f, Q_opt, id_exit_type);
%             CASPR_log.Debug('Done IDSolverTest/testFeasiblePolygon');
%         end
%         
        function testClosedForm(testCase, cf_solver_type)
            CASPR_log.Debug('Running IDSolverTest/testClosedForm');
            id_solver = IDSolverClosedForm(testCase.modelObj, cf_solver_type);
            [cable_f, Q_opt, id_exit_type] = id_solver.resolveFunction(testCase.modelObj);
            testCase.assertIDSolverOutput(cable_f, Q_opt, id_exit_type);
            CASPR_log.Debug('Done IDSolverTest/testClosedForm');
        end
        
    end
    
    methods
        function assertIDSolverOutput(testCase, actuation_soln, Q_opt, id_exit_type)
            testCase.assertLength(actuation_soln, testCase.modelObj.numActuatorsActive, 'Cable forces output of wrong dimension');
            testCase.assertInstanceOf(Q_opt, 'double');
            testCase.assertInstanceOf(id_exit_type, 'IDSolverExitType');
        end
    end
end