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
        
        function [cable_forces_active, cable_indices_active, cable_forces]  = execute(obj, q, q_d, q_dd, q_ref, q_ref_d, q_ref_dd, t)
            [cable_forces_active, model_result] = obj.executeFunction(q, q_d, q_dd, q_ref, q_ref_d, q_ref_dd, t);
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

