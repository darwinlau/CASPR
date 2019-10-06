% Solves the forward kinematics using the minimisation of least squares
% error of the cable lengths (optimisation based approach)
%
% Author        : Darwin LAU
% Created       : 2015
% Description    :
%   NOTE: This method currently does not work for quaternion
%   representations of orientation (refer to below)
classdef FKLeastSquares < FKAnalysisBase
    properties (Access = private)
        approxMethod    % Method to approximate the guess of q (FK_LS_ApproxOptionType enum)
        qDotMethod      % Method to compute q_dot (FK_LS_QDotOptionType enum)
    end

    methods
        % The constructor for least squares forward kinematics.
        function fkls = FKLeastSquares(kin_model, approxType, qDotType)
            fkls@FKAnalysisBase(kin_model);
            fkls.approxMethod = approxType;
            fkls.qDotMethod = qDotType;
        end

        % The implementatin of the abstract compute function.
        function [q, q_dot] = computeFunction(obj, len, len_prev, q_prev, q_d_prev, delta_t, cable_indices)
            L = obj.model.L(cable_indices, :);
            L_pinv = (L' * L) \ L';
            len_fk = len(cable_indices);
            len_fk_prev = len_prev(cable_indices);
            % Step 1: Compute the approximation of q for the optimiser
            switch obj.approxMethod
                case FK_LS_ApproxOptionType.USE_PREVIOUS_Q
                    q_approx = q_prev;
                case FK_LS_ApproxOptionType.FIRST_ORDER_INTEGRATE_QDOT
                    q_approx = obj.model.bodyModel.qIntegrate(q_prev, q_d_prev, delta_t);
                case FK_LS_ApproxOptionType.FIRST_ORDER_INTEGRATE_PSEUDOINV
                    obj.model.update(q_prev, zeros(obj.model.numDofs,1), zeros(obj.model.numDofs,1), zeros(obj.model.numDofs,1));
                    if delta_t ~= 0
                        q_approx = obj.model.bodyModel.qIntegrate(q_prev, L_pinv * (len_fk - len_fk_prev)/delta_t, delta_t);
                    else
                        q_approx = obj.model.bodyModel.qIntegrate(q_prev, zeros(obj.model.numDofs,1), 0);
                    end
                otherwise
                    CASPR_log.Print('approxMethod type is not defined',CASPRLogLevel.ERROR);
            end
            % Step 2: Define the function for the optimiser (minimising
            % error lengths)
            func = @(q_f) obj.ComputeLengthErrorVector(q_f, len_fk, obj.model, cable_indices);
            % Jacobian should be turned off for quaternion joints since the
            % jacobian for the quaternion is not the same as the quaternion
            % of the error
            % TODO: Need to consider the Jacobian of quaternions and length
            % (different number of variables q)
            options = optimoptions(@lsqnonlin, 'Display', 'none', 'Jacobian', 'on');

            % Step 3: Call the least squares non-linear function to
            % determine q
            [q, resnorm, ~, ~, output] = lsqnonlin(func, q_approx, obj.model.bodyModel.q_lb, obj.model.bodyModel.q_ub, options);

            CASPR_log.Debug(sprintf('Function lsqnonlin completed. Fitting error: %f. Number of function calls: %d', resnorm, output.funcCount));

            % Step 4: Determine the value of q_dot
            % TODO: Need to use generic approach to determine q_dot for
            % quaternions
            switch obj.qDotMethod
                case FK_LS_QdotOptionType.FIRST_ORDER_DERIV
                    if delta_t ~= 0
                        q_dot = (q - q_prev)/delta_t;
                    else
                        q_dot = zeros(obj.model.numDofs,1);
                    end
                case FK_LS_QdotOptionType.PSEUDO_INV
                    if delta_t ~= 0
                        obj.model.update(q, zeros(size(q)), zeros(size(q)), zeros(size(q)));
                        q_dot = L_pinv * (len_fk - len_fk_prev)/delta_t;
                    else
                        q_dot = zeros(obj.model.numDofs,1);
                    end
            end
        end
    end

    methods (Static)
        % ComputeInitialLengths serves to compute the initial length and
        % pose of a CDPR given a set of relative lengths, and approximated
        % q and initial length values. This is useful for CDPRs that only
        % have relative length sensors (encoders) and do not have absolute
        % encoders. The algorithm uses the FK Least Squares approach to
        % solve for the initial length.
        %
        % The function takes in the following:
        %   - model: the kinematic model
        %   - l_r: a cell array of relative lengths measure, l_r{k} is the
        %   relative length vector at time sample k
        %   - l0_approx: the approximate l0 vector to help the search
        %   - q_approx: cell array of approximate q, where q_appox{k} is
        %   the approximate q vector at time sample k
        function [l0, q] = ComputeInitialLengths(model, l_r, l0_approx, cable_indices, q_approx)
            numCables = length(l0_approx);
            numSamples = length(l_r);
            numDofs = length(q_approx{1});

            CASPR_log.Assert(model.numCables == numCables, 'The number of cables for the model and input l0_approx do not match');
            CASPR_log.Assert(model.numDofs == numDofs, 'The number of DoFs for the model and input q_approx do not match');

            % X vector contains all variables to optimise for:
            %   X = [l0; q{1}; q{2}; ...; q{numSamples}];
            % X_approx is the approximate version of X to begin the search
            X_approx = zeros(numCables + numSamples * numDofs, 1);
            X_approx(1:numCables) = l0_approx;
            for k = 1:numSamples
                X_approx(numCables + (k-1) * numDofs + 1: numCables + k*numDofs) = q_approx{k};
            end

            % Lower bound of l0 is 0, and lower bound of q is -Inf (no
            % bound)
            lb = -Inf*ones(size(X_approx));
            lb(1:numCables) = 0;

            % The least squares function to solve for X
            func = @(X) FKLeastSquares.ComputeInitialLengthsErrorVector(X, l_r, model, cable_indices);
            options = optimoptions(@lsqnonlin, 'Display', 'none', 'Jacobian', 'on');
            [X, resnorm, ~, ~, output] = lsqnonlin(func, X_approx, lb, [], options);
            CASPR_log.Print(sprintf('Function lsqnonlin completed. Fitting error: %f. Number of function calls: %d', resnorm, output.funcCount),CASPRLogLevel.INFO);

            % Extract the resulting l0 from X
            l0 = X(1:numCables);
            %%q = vec2mat(X(numCables+1:length(X)), numDofs)'; The function
            %%'vec2mat' is not supported in old-versioned matlab.
            q = reshape(X(numCables+1:length(X)),numDofs,[]);
        end

        % Responsible for computing the error vector for the nonlinear
        % least squares in ComputeInitialLengths.
        function [errorVector, jacobian] = ComputeInitialLengthsErrorVector(X, l_r, model, cable_indices)
            numSamples = length(l_r);
            numCables = length(l_r{1});
            numDofs = model.numDofs;
            errorVector = zeros(numSamples * numCables, 1);
            jacobian = zeros(numCables*numSamples, numCables + numDofs*numSamples);

            l0 = X(1:numCables);

            for k = 1:numSamples
                lr_k = l_r{k};
                qk = X(model.numCables + (k-1) * model.numDofs + 1:model.numCables + k * model.numDofs);

                model.update(qk, zeros(size(qk)), zeros(size(qk)), zeros(size(qk)))
                errorVector((k-1)*numCables+1:k*numCables) = lr_k + l0 - model.cableLengths(cable_indices);
                jacobian((k-1)*numCables + 1:k*numCables, 1:numCables) = eye(numCables, numCables);
                jacobian((k-1)*numCables + 1:k*numCables, numCables + (k-1)*numDofs + 1:numCables + k*numDofs) = -model.L(cable_indices, :);
            end
        end
    end
end
