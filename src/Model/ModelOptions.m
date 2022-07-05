% Contains the flags on whether the properties of the SystemModelBodies
% need to be recomputed
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
classdef ModelOptions < handle    
    properties (SetAccess = private)
        isComputeDynamics         = true      % A flag which indicates if dynamics computation is required
        isComputeHessian          = false     % A flag which indicates if the structural hessian matrix is need   
        isComputeLinearisation    = false     % A flag which indicates if linearisation is going to be used
%         operational_space   % A flag which indicates if operational space computation is required
    end
    
    methods
        % Constructor for the body flags
        function bmo = ModelOptions(compute_dynamics, compute_hessian, compute_linear_model)
            if (nargin >= 1)
                bmo.isComputeDynamics = compute_dynamics;
            elseif (nargin >= 2)
                bmo.isComputeHessian = compute_hessian;
            elseif (nargin >= 3)
                bmo.isComputeLinearisation = compute_linear_model;
            end
        end
    end
end

