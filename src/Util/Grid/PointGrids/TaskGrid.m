% Grid representation in operational space
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    :
%   This is a work in progress its point will be to take a starting point 
%   and give the next grid point to achieve a uniform spacing in the task 
%   space. TODO: CHANGE task space to op_space
classdef TaskGrid < GridBase
    properties (SetAccess = private)
        q_begin             % The beginning of the generalised coordinates
        q_end               % The ending of the generalised coordinates
        delta_x             % The change in the task space
        x_begin             % The beginning of the task coordinates
        x_end               % The ending of the task coordinates
        x_length            % The length of the task space coordinates
        q_length            % The length of the joint space coordinates
        n_task_dimensions   % The dimension of the task/operational space
    end
    
    methods
        % A constructor for the task grid object
        function id = TaskGrid(q_begin,q_end,x_begin,x_end,delta_x)
            CASPR_log.Assert((size(q_begin,2)==1)&&(size(q_end,2)==1)&&(size(delta_x,2)==1)&&(size(x_begin,2)==1)&&(size(x_end,2)==1),'Input to UniformGrid must be a column vector');
            CASPR_log.Assert((size(q_begin,1)==size(q_end,1)),'Inputs must be of the same dimension');
            CASPR_log.Assert((sum(q_begin > q_end)==0),'Invalid Input Range');
            % Maybe add more checks to ensure
            id.q_begin  =   q_begin;
            id.q_end    =   q_end;
            id.x_begin  =   x_begin;
            id.x_end    =   x_end;
            id.delta_x  =   delta_x;
            id.n_task_dimensions = size(x_begin,1); 
            id.n_dimensions = size(q_begin,1);
            id.x_length = (id.x_end - id.x_begin)./id.delta_x + 1;
            % THIS CAN BE CHANGED LATER
            id.q_length = zeros(id.n_dimensions,1);
            for i=1:id.n_dimensions
                id.q_length(i) = id.x_length;
            end
            id.n_points = prod(id.q_length);
        end
        
        % Get a point in the grid
        function q = getGridPoint(obj,index)
            % GENERALISE LATER - For now the first index is x the second
            % index is parameter
            
            % Convert the index into a column index
            q_index = zeros(obj.n_dimensions,1);
            q_mult = [obj.q_length;1];
            for i = 1:obj.n_dimensions
                q_div = prod(q_mult(i+1:length(q_mult)));
                q_index(i,1) = floor((index-1)/q_div);
                index = index - q_index(i,1)*q_div;
            end
            
            % Target position
            x_target = obj.x_begin + q_index(1)*obj.delta_x;
            if(abs(x_target)==2)
%                 q = pi*ones(2,1)*sign(x_target);
                q(1,1) = pi*sign(x_target);
                q(2,1) = 0;
            else
                if(x_target > 0)
                    q_off = 0;
                    r = acos(x_target/2);
                    % Angle
                    th = -pi + q_index(2)*((2*pi)/(obj.q_length(2)));
                    q0 = r*[cos(th);sin(th)];
                    options = optimset('display','off');
                else
                    q_off = pi;
                    r = pi - acos(x_target/2);
%                     kjfdskjfd
                    % Angle
                    th = -pi + q_index(2)*((2*pi)/(obj.q_length(2)));
                    q0 = -[pi;0] + r*[cos(th);sin(th)];
                    options = optimset('display','off');
                end
                q = fsolve(@(q) target_kinematics(q,x_target,th,q_off),q0,options);
                if((x_target - cos(q(1)) - cos(q(1) + q(2)))^2 + (th- atan2(q(2),q(1)+q_off))^2>1e-5)
                    x_target
                    th
                end
                if(sum(abs(q)>pi))
                    q = q-2*pi.*(sign(q).*(abs(q)>pi));
                end
            end
            
%             q = 0;
        end 
    end
    
end

% An objective for the nonlinear evaluation
function cost = target_kinematics(q,x_target,th_target,q_off)
    cost  = ((x_target - cos(q(1)) - cos(q(1) + q(2)))^2 + (th_target - atan2(q(2),q(1)+q_off))^2);
end

