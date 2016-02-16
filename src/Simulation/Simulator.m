classdef (Abstract) Simulator < handle
    %DYNAMICSSIMULATION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)    
        model          % base SystemKinematics object 
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

