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
        hessian         % A flag which indicates if the structural hessian matrix is need    
        linearisation   % A flag which indicates if linearisation is going to be used
    end
    
    methods
        % Constructor for the body flags
        function occupied = BodyFlags()
           occupied.dynamics        =   false;
           occupied.op_space        =   false;
           occupied.hessian         =   false;
           occupied.linearisation   =   false;
        end
    end
    
end

