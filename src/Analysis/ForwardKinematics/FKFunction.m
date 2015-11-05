classdef FKFunction < handle
    %IDFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected, GetAccess = protected)
        q_previous = []
        l_previous = []
    end
    
    methods (Abstract)
        f = compute(obj, length, kin_model, kin_prev, delta_t);
    end
        
%     methods (Static)
%         function [q, q_dot] = ComputeGeneralisedCoordinates1(model, l, l_dot, q_prev, q_prev_dot, dt)
%             q_approx = q_prev + dt * q_prev_dot;
%             func = @(q) ForwardKinematics.ComputeErrorVector(q, l, model);
%             options = optimoptions(@lsqnonlin, 'Display', 'none', 'Jacobian', 'on');
%             [q, resnorm, residual, exitflag, output] = lsqnonlin(func, q_approx, [], [], options);
%             resnorm
%             output.funcCount
%             model.update(q, zeros(size(q)), zeros(size(q)));
%             q_dot = pinv(model.L) * l_dot;
%         end
%         
%         function [q, q_dot] = ComputeGeneralisedCoordinates2(model, l, l_dot_prev, q_prev, dt)
%             model.update(q_prev, zeros(size(q_prev)), zeros(size(q_prev)));
%             q_approx = q_prev + dt * pinv(model.L) * l_dot_prev;
%             
%             func = @(q) ForwardKinematics.ComputeErrorVector(q, l, model);
%             
%             options = optimoptions(@lsqnonlin, 'Display', 'none', 'Jacobian', 'on');
%             [q, resnorm, residual, exitflag, output] = lsqnonlin(func, q_approx, [], [], options);
%             resnorm
%             output.funcCount
%             if dt ~= 0
%                 q_dot = (q - q_prev)/dt;
%             else
%                 q_dot = zeros(size(q));
%             end
%         end
%         
%         function [q, q_dot] = ComputeGeneralisedCoordinates3(model, l, q_prev, q_prev_dot, dt)
%             q_approx = q_prev + dt * q_prev_dot;
%             func = @(q) ForwardKinematics.ComputeErrorVector(q, l, model);
%             options = optimoptions(@lsqnonlin, 'Display', 'none', 'Jacobian', 'on');
%             [q, resnorm, residual, exitflag, output] = lsqnonlin(func, q_approx, [], [], options);
%             resnorm
%             output.funcCount
%             if dt ~= 0
%                 q_dot = (q - q_prev)/dt;
%             else
%                 q_dot = zeros(size(q));
%             end
%         end
%         
%         function [q, q_dot] = ComputeGeneralisedCoordinates4(model, l, q_prev, dt)
%             q_approx = q_prev;
%             func = @(q) ForwardKinematics.ComputeErrorVector(q, l, model);
%             options = optimoptions(@lsqnonlin, 'Display', 'none', 'Jacobian', 'on');
%             [q, resnorm, residual, exitflag, output] = lsqnonlin(func, q_approx, [], [], options);
%             resnorm
%             output.funcCount
%             if dt ~= 0
%                 q_dot = (q - q_prev)/dt;
%             else
%                 q_dot = zeros(size(q));
%             end
%         end
%         
%         function [q, q_dot] = ComputeGeneralisedCoordinates5(model, l_dot_prev, q_prev, dt)
%             model.update(q_prev, zeros(size(q_prev)), zeros(size(q_prev)));
%             q = q_prev + dt * pinv(model.L) * l_dot_prev;
%             q_dot = (q - q_prev)/dt;
%         end
%         
%         function [errorVector, jacobian] = ComputeErrorVector(q, l, model)
%             model.update(q, zeros(size(q)), zeros(size(q)));
%             errorVector = l - model.cableLengths;
%             jacobian = - model.L;
%         end
%     end
end

