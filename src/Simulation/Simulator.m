% Base class for different types of simulators used in analysis of CDPRs.
%
% Author        : Darwin LAU
% Created       : 2013
% Description    :
%   Different types of simulators should inherit from the base class so
%   common functionality can be achieved. The child classes should
%   implement the "run" function.
classdef (Abstract) Simulator < handle
    
    properties (SetAccess = protected)    
        model          % base SystemModel object 
    end
    
    methods 
        function s = Simulator(model)
            s.model = model;
        end
    end
    
    methods (Abstract)
        run(obj)
    end
end

