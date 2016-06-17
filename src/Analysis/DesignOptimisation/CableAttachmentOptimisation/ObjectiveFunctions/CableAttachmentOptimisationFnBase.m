% Abstract bass class for all cable attachment optimisation functions.
% 
% Author        : Darwin LAU
% Created       : 2016
% Description	:
classdef (Abstract) CableAttachmentOptimisationFnBase < handle
    
    properties
    end
    
    methods (Abstract)
        % The "evaluate" function
        Q = evaluate(obj);
    end
    
end

