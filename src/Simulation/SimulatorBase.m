% Base class for different types of simulators used in analysis of CDPRs.
%
% Author        : Darwin LAU
% Created       : 2013
% Description    :
%   Different types of simulators should inherit from the base class so
%   common functionality can be achieved. The child classes should
%   implement the "run" function.
classdef (Abstract) SimulatorBase < handle
    
    properties (SetAccess = protected)    
        model          % base SystemModel object 
    end
    
    methods 
        % Constructor
        function s = SimulatorBase(model)
            CASPR_log.Assert(isa(model,'SystemModel'));
            s.model = model;
        end
    end
    
    methods (Abstract)
        % The function to run objects
        run(obj)
    end
end

