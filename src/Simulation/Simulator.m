classdef (Abstract) Simulator < handle
    %DYNAMICSSIMULATION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)    
        kinematicsObj          % base SystemKinematics object 
    end
    
    methods 
        function s = Simulator(kinObj)
            s.kinematicsObj = kinObj;
        end
    end
    
    methods (Abstract)
        run(obj)
    end
end

