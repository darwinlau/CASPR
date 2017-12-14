% Class to compute whether a pose (dynamics) is within the interference 
% free workspace (IFW)
%
% Author        : Benji
% Created       : 2017
% Description   : 
classdef InterferenceFreeRay < WorkspaceRayConditionBase
    properties (SetAccess = protected, GetAccess = protected)
        min_ray_percentage          % The minimum percentage of the ray at which it is included
    end
    
    methods
        % Constructor for interference free worksapce
        function w= InterferenceFreeRay(min_ray_percent)
            w.min_ray_percentage = min_ray_percent;
        end
            
        % Evaluate the interference free intervals 
        function intervals =  evaluateFunction(obj,model,workspace_ray)
            %% NEED TO BE FILLED IN
        end
    end
    
end

