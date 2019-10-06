% Base class for the forward kinematics analysis of CDPRs
%
% Author        : Darwin LAU
% Created       : 2015
% Description    :
%   Child classes must implement the "computeFunction" with the desired
%   method of resolving the forward kinematics. This class provides some
%   basic functionality, such as:
%       - Determining the computational of the "computeFunction"
%       - Computing the error in length and Jacobian of the error function
classdef FKAnalysisBase < handle
    properties
        model               % The model of the system
    end

    properties (SetAccess = protected, GetAccess = protected)
        q_previous = []     % The previous joint positions
        l_previous = []     % The previous cable lengths
    end

    methods
        % Constructor for forward kinematics objects
        function fk = FKAnalysisBase(kin_model)
            fk.model = kin_model;
        end

        % Computes the joint position information given the cable
        % information.
        function [q, q_dot, comp_time] = compute(obj, len, len_prev, q_prev, q_d_prev, delta_t, cable_indices)
            if nargin < 7 || isempty(cable_indices)
                cable_indices = 1:obj.model.numCables;
            else
                CASPR_log.Assert(length(cable_indices) >= obj.model.numDofs, 'For forward kinematics, the number of cables to be used to compute must be at least the number of DoFs');
            end
            start_tic = tic;
            obj.model.update(q_prev,zeros(obj.model.numDofs,1),zeros(obj.model.numDofs,1),zeros(obj.model.numDofs,1));
            [q, q_dot] = obj.computeFunction(len, len_prev, q_prev, q_d_prev, delta_t, cable_indices);
            comp_time = toc(start_tic);
        end
    end

    methods (Abstract)
        % An abstract function for computation.
        [q, q_dot] = computeFunction(obj, len, len_prev, cable_indices, q_prev, q_d_prev, delta_t);
    end

    methods (Static)
        % Computation of the length error as a 1 norm.
        function [errorVector, jacobian] = ComputeLengthErrorVector(q, l, model, cable_indices)
            model.update(q, zeros(model.numDofs,1), zeros(model.numDofs,1),zeros(model.numDofs,1));
            errorVector = l - model.cableLengths(cable_indices);
            jacobian = - model.L(cable_indices, :);
        end
    end
end
