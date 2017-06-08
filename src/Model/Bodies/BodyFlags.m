% Contains the flags on whether the properties of the SystemModelBodies
% need to be recomputed
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
classdef BodyFlags < handle    
    properties
        dynamics            % A flag which indicates if dynamics computation is required
        operational_space   % A flag which indicates if operational space computation is required
        hessian             % A flag which indicates if the structural hessian matrix is need    
        linearisation       % A flag which indicates if linearisation is going to be used
    end
    
    methods
        % Constructor for the body flags
        function occupied = BodyFlags()
           occupied.dynamics                =   false;
           occupied.operational_space       =   false;
           occupied.hessian                 =   false;
           occupied.linearisation           =   false;
        end
        
        % Resets the body flags to their default state
        function reset(obj)
           obj.dynamics                 =   false;
           obj.operational_space        =   false;
           obj.hessian                  =   false;
           obj.linearisation            =   false;
        end
    end
end

