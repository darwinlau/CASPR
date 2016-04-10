% Contains the flags on whether the properties of the SystemModelBodies
% need to be recomputed
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
classdef BodyFlags < handle    
    properties
        dynamics        % A flag which indicates if dynamics computation is required
        op_space        % A flag which indicates if operational space computation is required
    end
    
    methods
        function occupied = BodyFlags()
           occupied.dynamics = false;
           occupied.op_space = false;
        end
    end
    
end

