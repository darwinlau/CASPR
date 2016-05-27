% Abstract class for black-box optimisers
% 
% Author        : Darwin LAU
% Created       : 2016
% Description	:
classdef (Abstract) BBOptimiserBase < handle
    properties
        numVars             % Number of variables for x
        objectiveFn         % Objective function for black box optimiser
    end
    
    methods
        function op = BBOptimiserBase(numVars, objectiveFn)
            op.numVars = numVars;
            op.objectiveFn = objectiveFn;
        end
    end
    
    methods (Abstract)
        [x_opt, Q_opt] = optimise(obj);
    end
    
end

