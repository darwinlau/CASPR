% Base class for CDPR controllers to inherit from when creating new
% controllers
%
% Author        : Darwin LAU
% Created       : 2016
% Description    : 
classdef ControllerBase < handle
    properties
        dynModel
    end
    
    methods 
        function cb = ControllerBase(dyn_model)
            cb.dynModel = dyn_model;
        end
        
    end
    
    methods (Abstract)
         [cable_forces] = executeFunction(obj, q, q_d, q_dd, q_ref, q_ref_d, q_ref_dd);
    end
    
end

