% Base class for CDPR controllers to inherit from when creating new
% controllers
%
% Author        : Darwin LAU
% Created       : 2016
% Description    :
classdef ControllerBase < handle
    properties
        dynModel            % The model of the system
    end

    methods
        % A constructor for the controller base.
        function cb = ControllerBase(dyn_model)
            cb.dynModel = dyn_model;
        end

        function [f_active, cable_indices_active, cable_forces, exit_flag]  = execute(obj, q, q_d, q_dd, y_ref, y_ref_d, y_ref_dd, disturbance_est, t)
            % here as a temporary measure, the disturbance estimation will
            % be passed to the controller and held inside the system model
            % object. The exact form of the disturbance estimation soly
            % depends on the variables passed when the exexute function
            % called, hence how it is defined (when being passed in,
            % usually in control simulator) and how it is used (usually
            % inside the definition of a controller) should be consistent.
            % Updating the model with the disturbance_est will store the
            % disturbance estimation into the model, and will be used
            % closely after by the executeFunction() function call
            obj.dynModel.update(q, q_d, q_dd, disturbance_est);
            % here the f_active contains the active cable forces as well as
            % the active joint torques
            [f_active, model_result, exit_flag] = obj.executeFunction(q, q_d, q_dd, y_ref, y_ref_d, y_ref_dd, t);
            cable_indices_active = model_result.cableModel.cableIndicesActive;
            cable_forces = model_result.cableForces;
        end
        
        % function that converges inverse dynamics exit type into a
        % controller exit type
        function [ctrl_exit_type] = exitTypeConversion(obj, id_exit_type)
            
            switch id_exit_type
                case IDSolverExitType.NO_ERROR
                    ctrl_exit_type = ControllerExitType.NO_ERROR;
                case IDSolverExitType.ITERATION_LIMIT_REACHED
                    CASPR_log.Info('Controller - Max iteration limit reached');
                    ctrl_exit_type = ControllerExitType.ITERATION_LIMIT_REACHED;
                case IDSolverExitType.INFEASIBLE
                    CASPR_log.Info('Controller - Problem infeasible');
                    ctrl_exit_type = ControllerExitType.INFEASIBLE;
                otherwise
                    CASPR_log.Info('Controller - Other error');
                    ctrl_exit_type = ControllerExitType.SOLVER_SPECIFIC_ERROR;
            end
            
        end
    end

    methods (Abstract)
        % An abstract executeFunction for all controllers. This should take
        % in the generalised coordinate information and produces a control
        % input.
        [f_active, model, exit_flag] = executeFunction(obj, q, q_d, q_dd, q_ref, q_ref_d, q_ref_dd, t);
    end    
end
