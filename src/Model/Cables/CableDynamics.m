% Dynamics representation for an ideal (massless and rigid) cable
% 
% Author        : Darwin LAU
% Created       : 2011
% Description	:
%	This is the simplest type of cable that is massless, rigid and assumes 
%   a straight-line model between attachment points. The parameters that
%   are required for the dynamics are the minimum and maximum forces that
%   the cable can exert (and is constant throughout the workspace). 
classdef (Abstract) CableDynamics < handle
    properties
        % Actual cable force
        force = 0;
        % Minimum and maximum allowable cable force
        forceMin
        forceMax
        % Stiffness of the cable
        K
        % This value is used to represent the force when invalid. For
        % example, it can be set as -1 to indicate that no solution is/was
        % obtained, or set to a high value above forceMax if this is used
        % within some optimisation regime. 
        forceInvalid
        
        name = '';                  % Cable name
    end
    
    methods
        function ck = CableDynamics(name)
            ck.name = name;
        end
        
        % At this stage the method does nothing
        function update(obj, cableKinematics, bodyKinematics)
        end
    end
    
end

