classdef FKLeastSquares < FKFunction
    %IDFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
        approxMethod
        qDotMethod
    end
    
    methods
        function fkls = FKLeastSquares(approxType, qDotType)
            fkls.approxMethod = approxType;
            fkls.qDotMethod = qDotType;            
        end
        
        function [q, q_dot] = compute(obj, len, len_prev_2, q_prev, q_d_prev, delta_t, kin_model)
            switch obj.approxMethod
                case FK_LS_ApproxOptionType.USE_PREVIOUS_Q
                    q_approx = q_prev;
                case FK_LS_ApproxOptionType.FIRST_ORDER_INTEGRATE_QDOT
                    q_approx = kin_model.qIntegrate(q_prev, q_d_prev, delta_t);
                case FK_LS_ApproxOptionType.FIRST_ORDER_INTEGRATE_PSEUDOINV
                    kin_model.update(q_prev, zeros(kin_model.numDofs,1), zeros(kin_model.numDofs,1));
                    if delta_t ~= 0
                        q_approx = kin_model.bodyKinematics.qIntegrate(q_prev, pinv(kin_model.L) * (len - len_prev_2)/(2*delta_t), delta_t);
                    else
                        q_approx = kin_model.bodyKinematics.qIntegrate(q_prev, zeros(kin_model.numDofs,1), 0);
                    end
            end
            func = @(q_f) FKFunction.ComputeLengthErrorVector(q_f, len, kin_model);
            
            % TODO THE JACOBIAN MATRIX IS DIFFERENT FOR SPHERICAL JOINT
            options = optimoptions(@lsqnonlin, 'Display', 'none', 'Jacobian', 'on');
            [q, resnorm, ~, ~, output] = lsqnonlin(func, q_approx, [], [], options);
            
            display(sprintf('Function lsqnonlin completed. Fitting error: %f. Number of function calls: %d', resnorm, output.funcCount));
            
            % TODO HERE TO MAKE DERIVATIVE GENERIC TOO
            switch obj.qDotMethod
                case FK_LS_QdotOptionType.FIRST_ORDER_DERIV
                    if delta_t ~= 0
                        q_dot = (q - q_prev)/delta_t;
                    else
                        q_dot = zeros(kin_model.numDofs,1);
                    end
                case FK_LS_QdotOptionType.PSEUDO_INV
                    if delta_t ~= 0
                        kin_model.update(q, zeros(size(q)), zeros(size(q)));
                        q_dot = pinv(kin_model.L) * (len - len_prev_2)/(2*delta_t);
                    else
                        q_dot = zeros(kin_model.numDofs,1);
                    end
            end
        end        
    end
        
    methods (Static)
        function [l0] = ComputeInitialLengths(model, l_r, l0_approx, q_approx)            
            numCables = length(l0_approx);
            numSamples = length(l_r);
            numDofs = length(q_approx{1});
            
            assert(model.numCables == numCables, 'The number of cables for the model and input l0_approx do not match');
            assert(model.numDofs == numDofs, 'The number of DoFs for the model and input q_approx do not match');
            
            X_approx = zeros(numCables + numSamples * numDofs, 1);
            X_approx(1:numCables) = l0_approx;
            for k = 1:numSamples
                X_approx(numCables + (k-1) * numDofs + 1: numCables + k*numDofs) = q_approx{k};
            end
            lb = -Inf*ones(size(X_approx));
            lb(1:numCables) = 0;
            
            func = @(X) FKLeastSquares.ComputeInitialLengthsErrorVector(X, l_r, model);
            
            options = optimoptions(@lsqnonlin, 'Display', 'none', 'Jacobian', 'on');
            [X, resnorm, ~, ~, output] = lsqnonlin(func, X_approx, lb, [], options);
            display(sprintf('Function lsqnonlin completed. Fitting error: %f. Number of function calls: %d', resnorm, output.funcCount));
            
            l0 = X(1:numCables);
        end
        
        function [errorVector, jacobian] = ComputeInitialLengthsErrorVector(X, l_r, model)
            numSamples = length(l_r);
            numCables = model.numCables;
            numDofs = model.numDofs;
            errorVector = zeros(numSamples * numCables, 1);
            jacobian = zeros(numCables*numSamples, numCables + numDofs*numSamples);
            
            l0 = X(1:model.numCables);
            
            for k = 1:numSamples
                lr_k = l_r{k};
                qk = X(model.numCables + (k-1) * model.numDofs + 1:model.numCables + k * model.numDofs);
                
                model.update(qk, zeros(size(qk)), zeros(size(qk)))
                errorVector((k-1)*numCables+1:k*numCables) = lr_k + l0 - model.cableLengths;
                jacobian((k-1)*numCables + 1:k*numCables, 1:numCables) = eye(numCables, numCables);
                jacobian((k-1)*numCables + 1:k*numCables, numCables + (k-1)*numDofs + 1:numCables + k*numDofs) = -model.L;
            end
        end
    end
end

