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

        function [cable_forces_active, cable_indices_active, cable_forces]  = execute(obj, q, q_d, q_dd, y_ref, y_ref_d, y_ref_dd, disturbance_est, t)
            % here as a temporary measure, the disturbance estimation will
            % be passed to the controller and held inside the system model
            % object. The exact form of the disturbance estimation soly
            % depends on the variables passed when the exexute function
            % called, hence how it is defined (when being passed in,
            % usually in control simulator) and how it is used (usually
            % inside the definition of a controller) should be consistent.
            obj.dynModel.update(q, q_d, q_dd, disturbance_est);
            [cable_forces_active, model_result] = obj.executeFunction(q, q_d, q_dd, y_ref, y_ref_d, y_ref_dd, t);
            cable_indices_active = model_result.cableModel.cableIndicesActive;
            cable_forces = model_result.cableForces;
        end
    end

    methods (Abstract)
        % An abstract executeFunction for all controllers. This should take
        % in the generalised coordinate information and produces a control
        % input.
        [cable_forces_active, model] = executeFunction(obj, q, q_d, q_dd, q_ref, q_ref_d, q_ref_dd, t);
    end    
end
