% Base object for testing the IDObjective objects
%
% Author        : Darwin LAU
% Created       : 2016
% Description    :
classdef IDSolverTest < matlab.unittest.TestCase
    properties (ClassSetupParameter)
        model_config_type = struct('SCDM', TestModelConfigType.T_SCDM, ...
            'MCDM', TestModelConfigType.T_MCDM, ...
            'Active_passive_cables', TestModelConfigType.T_ACTIVE_PASSIVE_CABLES);
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
        os_solver_type = struct('LP', ID_OS_SolverType.LP, ...
            'Efficient_LP', ID_OS_SolverType.EFFICIENT_LP);
       fp_solver_type = struct('NORM_1', ID_FP_SolverType.NORM_1, ...
            'NORM_2', ID_FP_SolverType.NORM_2, ...
            'CENTROID', ID_FP_SolverType.CENTROID);
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
            disp('Testing QP solver')
            id_objective = IDObjectiveMinQuadCableForce(ones(1, testCase.modelObj.numCables));
            id_solver = IDSolverQuadProg(testCase.modelObj, id_objective, qp_solver_type);
            [cable_f, Q_opt, id_exit_type] = id_solver.resolveFunction(testCase.modelObj);
            testCase.assertIDSolverOutput(cable_f, Q_opt, id_exit_type);
        end
        
        function testLPSolver(testCase, lp_solver_type)
            disp('Testing LP solver')
            id_objective = IDObjectiveMinLinCableForce(ones(testCase.modelObj.numCables, 1));
            id_solver = IDSolverLinProg(testCase.modelObj, id_objective, lp_solver_type);
            [cable_f, Q_opt, id_exit_type] = id_solver.resolveFunction(testCase.modelObj);
            testCase.assertIDSolverOutput(cable_f, Q_opt, id_exit_type);
        end
        
        function testInfPSolver(testCase, infp_solver_type)
            disp('Testing Inf norm solver')
            id_objective = IDObjectiveMinInfCableForce(ones(testCase.modelObj.numCables, 1));
            id_solver = IDSolverMinInfNorm(testCase.modelObj, id_objective, infp_solver_type);
            [cable_f, Q_opt, id_exit_type] = id_solver.resolveFunction(testCase.modelObj);
            testCase.assertIDSolverOutput(cable_f, Q_opt, id_exit_type);
        end
        
%         function testOptimallySafeSolver(testCase, os_solver_type)
%             id_solver = IDSolverOptimallySafe(testCase.modelObj, 1, os_solver_type);
%             [cable_f, Q_opt, id_exit_type] = id_solver.resolveFunction(testCase.modelObj);
%             testCase.assertIDSolverOutput(cable_f, Q_opt, id_exit_type);
%         end
        
%        function testFeasiblePolygon(testCase, fp_solver_type)
%            id_solver = IDSolverFeasiblePolygon(testCase.modelObj, fp_solver_type);
%            [cable_f, Q_opt, id_exit_type] = id_solver.resolveFunction(testCase.modelObj);
%            testCase.assertIDSolverOutput(cable_f, Q_opt, id_exit_type);
%        end
         
         function testClosedForm(testCase, cf_solver_type)
             disp('Testing closed form solver')
             id_solver = IDSolverClosedForm(testCase.modelObj, cf_solver_type);
             [cable_f, Q_opt, id_exit_type] = id_solver.resolveFunction(testCase.modelObj);
             testCase.assertIDSolverOutput(cable_f, Q_opt, id_exit_type);
         end
        
    end
    
    methods
        function assertIDSolverOutput(testCase, cable_f, Q_opt, id_exit_type)
            testCase.assertLength(cable_f, testCase.modelObj.numCablesActive, 'Cable forces output of wrong dimension');
            testCase.assertInstanceOf(Q_opt, 'double');
            testCase.assertInstanceOf(id_exit_type, 'IDSolverExitType');
        end
    end
end