% Algorithm to to compute the trajectory feasible range under wrench clouse condition
%
% Author        : Paul Cheng
% Created       : 2019
% Description   : The class for evaluation of WCC
classdef TrajecotryWrenchClousreCondition < TrajectoryConditionBase
    properties (Constant)
        % Type of workspace condition (TrajectoryConditionType enum)
        type = TrajectoryConditionType.WRENCH_CLOSURE;
    end
    
    properties
        method
        sample_number            % number of sample point(affect accuracy)
        feasible_time_range = [];% feasible time range under WCC
        feasible_range = [];     % feasible pose range under WCC
        theta = [];              % only for analytical method
    end
    properties (Access = private)
        tolerance = 9;           % error tolerance, digits
        default_time_range = []; % default_time_range from input
        feasible_intervals = []; % local use, sets of calculated solutions
        trajectory = [];         % cfit type functions
        adjust_value = 1;       % Inaccurate result might found if the polynomial has short x-range, adjust this number
        adjust_offset = 1.;       % Inaccurate result might found if the polynomial has short x-range, adjust this number
    end
    
    methods
        % Constructor for wcc
        function t = TrajecotryWrenchClousreCondition(method,sample_number)
            if (isempty(method))
                t.method = TrajectoryAnalysisMethodType.NUMERICAL_ANALYTICAL;
            else
                t.method = method;
            end
            t.sample_number = sample_number;
        end
        
        % Evaluate the Interference Free condition return true if
        % satisified
        function [feasible_range,feasible_time_range,other_info,theta] = evaluateFunction(obj,model,trajectory,time_range,maximum_trajectory_degree)
            
            obj.feasible_time_range = [];
            obj.feasible_range = [];
            obj.tolerance = 9;
            obj.feasible_intervals = [];
            obj.trajectory = [];
            other_info = [];
            theta = [];
            obj.trajectory = trajectory;
            obj.default_time_range = time_range;
            switch obj.method
                case TrajectoryAnalysisMethodType.NUMERICAL_ANALYTICAL                    
                    obj.default_time_range = [0 1];
                    if isempty(obj.sample_number)
                        CASPR_log.Error('Please input sampling number')
                    end
                    poses = PosesData(obj,trajectory,obj.default_time_range);
                    J_d = JacobianData(model,poses);
                    TrajectoryFeasibilityVerification(obj,model,J_d);
                    feasible_time_range = obj.feasible_time_range;
                    feasible_range = obj.feasible_range;
                case TrajectoryAnalysisMethodType.ANALYTICAL                     
                    obj.default_time_range = time_range;
                    obj.default_time_range(2) = obj.default_time_range(2)*obj.adjust_offset;
                    theta = GetTheta(model,trajectory,time_range);  
                    obj.theta = theta;
                    obj.sample_number = model.numDofs*(maximum_trajectory_degree+4)+1;                    
                    Jacobian_Matrics = MatrixData(obj,model,theta,maximum_trajectory_degree);
                    TrajectoryFeasibilityVerification_A(obj,model,Jacobian_Matrics,theta);                    
                    translation_time_interval = obj.feasible_time_range;
                    orientation_time_interval = atan(obj.feasible_time_range*tan(theta/2))*2/theta;
                    other_info = {translation_time_interval,orientation_time_interval};
                    feasible_time_range = obj.feasible_time_range;
                    feasible_range = obj.feasible_range;
                    
                otherwise
                    CASPR_log.Error('Invalid analysis method');
            end
        end
        %% Analytical functions
        %% A function to handle different DOFs and N+M cables robot
        function TrajectoryFeasibilityVerification_A(obj,model,Jacobian_Matrics,theta)    
            obj.default_time_range = obj.adjust_value*obj.default_time_range;
            if model.numDofs + 1 < model.numCables
                check_indices = nchoosek(1:model.numCables,model.numDofs+1);
                for i = 1: size(check_indices,1)
                    for j = 1:size(Jacobian_Matrics,2)
                        remain_Jacobian_Matrics{j} = Jacobian_Matrics{j}(:,check_indices(i,:));
                    end
                    fit_polynomials = PolynomialFitting_A(obj,model,remain_Jacobian_Matrics);
                    
                    if ~isempty(fit_polynomials)
                        [time] = WorkspaceBoundaryTime_A(obj,fit_polynomials);
                        obj.feasible_intervals = [obj.feasible_intervals;time];
                    end
                end
                
            elseif model.numDofs + 1 == model.numCables
                
                fit_polynomials = PolynomialFitting_A(obj,Jacobian_Matrics);
                
                if ~isempty(fit_polynomials)
                    [time] = WorkspaceBoundaryTime_A(obj,model,fit_polynomials);
                    obj.feasible_intervals = [obj.feasible_intervals;time];
                end
            else
                error('Input Model Error');
            end
            
            if ~isempty(obj.feasible_intervals)
                obj.feasible_intervals = sortrows(obj.feasible_intervals,1);
                obj.feasible_intervals = round(obj.feasible_intervals,obj.tolerance);
                current_interval = obj.feasible_intervals(1,:);
                
                for i = 1:size(obj.feasible_intervals,1)
                    
                    if obj.feasible_intervals(i,1) <= current_interval(end,2) && obj.feasible_intervals(i,1) >= current_interval(end,1) && obj.feasible_intervals(i,2) >= current_interval(end,2)
                        current_interval(end,:) = [current_interval(end,1),obj.feasible_intervals(i,2)];
                    elseif obj.feasible_intervals(i,1) <= current_interval(end,2) && obj.feasible_intervals(i,1) >= current_interval(end,1) && obj.feasible_intervals(i,2) <= current_interval(end,2)
                        %
                    elseif obj.feasible_intervals(i,1) >= current_interval(2)
                        current_interval = [current_interval;obj.feasible_intervals(i,:)];
                    end
                    
                end
                
                obj.feasible_time_range = current_interval/obj.adjust_value;
                
                q_ans = [];
                for j = 1:size(obj.feasible_time_range,1)
                    for i = 1:size(obj.feasible_time_range,2)
                        q_ans = [q_ans;InstancePose(model,obj.trajectory,obj.feasible_time_range(j,i),theta,obj.default_time_range)'];                        
                    end
                end
                obj.feasible_range = q_ans;
            end
            if isempty(obj.feasible_time_range)
                Message = 'No feasible range inside the workspace';
                disp(Message)
            end
        end      
        %% A function to find and varify the valid time interval
        function [time] = WorkspaceBoundaryTime_A(obj,model,fcn)
            
            polynomial_ans = [];
            time = [];
            continue_count = 1;                 
           obj.default_time_range = obj.default_time_range/obj.adjust_offset;
            time_ans = obj.default_time_range';
            for i = 1:size(fcn,2)
%                 tmp_ans = roots(fcn{i});
%                 fcn{i} = round(fcn{i},5);
                polynomial_ans = [polynomial_ans,roots(fcn{i})'];
            end
%             polynomial_ans = polynomial_ans/obj.adjust_value;
%             for i = 1:length(polynomial_ans)
%                 if isreal(polynomial_ans(i))
%                     if real(polynomial_ans(i)) >= obj.default_time_range(1) && real(polynomial_ans(i)) <= obj.default_time_range(2)
%                         time_ans = [time_ans;polynomial_ans(i)];
%                     end
%                 end
%             end
           time_ans = sort([time_ans;unique(real(polynomial_ans))']);
           time_ans = time_ans(time_ans>=obj.default_time_range(1));
           time_ans = time_ans(time_ans<=obj.default_time_range(2));
         
            if size(time_ans,1) == 1
                time_ans = obj.default_time_range;
            end
            
            if length(time_ans) >= 2
                
                for k = 1:length(time_ans)-1  
                    mid_point_test = [];lower_bound_test = [];upper_bound_test = [];
               
%                     for h = 1:size(fcn,2)-1
%                         mid_point_test(h) = polyval(fcn{h},(time_ans(k)+time_ans(k+1))/2) * polyval(fcn{end},(time_ans(k)+time_ans(k+1))/2);
%                         lower_bound_test(h) =  polyval(fcn{h},time_ans(k)) * polyval(fcn{end},time_ans(k));
%                         upper_bound_test(h) =  polyval(fcn{h},time_ans(k+1)) * polyval(fcn{end},time_ans(k+1));
%                     end
                    q = InstancePose(model,obj.trajectory,(time_ans(k)+time_ans(k+1))/2,obj.theta,obj.default_time_range);                   
%                     if all(round(mid_point_test,obj.tolerance) < 0) && all(round(lower_bound_test,obj.tolerance) <= 0) && all(round(upper_bound_test,obj.tolerance) <= 0)
                    if wrench_closure_condition_checking(model,q) 
                        if continue_count == 1
                            if ~isempty(time) && time(end) == time_ans(k)
                            time(end,end) = time_ans(k+1);
                            else
                            time = [time,[time_ans(k),time_ans(k+1)]];    
                            end                            
                        else
                            time = [time;[time_ans(k),time_ans(k+1)]];
                            continue_count = 1;
                        end
                    else
                        continue_count = 0;
                    end
                end
            end
        end
        % A function to find out the interference free time intervals
        function Jacobian_Matrics = MatrixData(obj,model,theta,max_deg)
            % get the sampling data
            T = linspace(obj.default_time_range(1),obj.default_time_range(2),obj.sample_number);
%             T = linspace(0,tan(theta/2),obj.sample_number);
            
            for i = 1:length(T)                
            poses(:,i) = InstancePose(model,obj.trajectory,T(i),theta,[T(1),T(end)]);  
            if i == 36
               i 
            end
            end
            Jacobian_Matrics = JacobianData(model,poses);          
            
        end
         %% A function to estimate the possible interpolated curve for the RREF data
        function [fit_equ] = PolynomialFitting_A(obj,Jacobian_Matrics)
            null_data_count = [];
            
            for i = 1:length(Jacobian_Matrics)
                n_n_matrix = Jacobian_Matrics{i}(:,1:end-1);
                n_1_matrix = Jacobian_Matrics{i}(:,end);                
               
                matrix_determinant = det(n_n_matrix);
                
                if round(matrix_determinant,obj.tolerance) ~=0
                    column_vector(i,:) = inv(n_n_matrix)*n_1_matrix*matrix_determinant;
                    det_d(i) =  matrix_determinant;
                else
                    null_data_count = [null_data_count,i];
                    det_d(i) = 0;
                    column_vector(i,:) = zeros(1,size(n_n_matrix,1));
                end
                
            end
            
            time_step = linspace(obj.default_time_range(1),obj.default_time_range(2),obj.sample_number);
            tmp_time_step = time_step;
            
            if size(null_data_count,2) ~= size(Jacobian_Matrics,2)
                if isempty(null_data_count)
                     for i = 1:size(time_step,2)
                        for j = 1:size(time_step,2)
                        y(j,i) = [time_step(j)^(i-1)];
                        end
                     end
                    for i = 1:size(column_vector,2)
                        fit_equ{i} = flipud(y\column_vector(:,i))';
%                         [fit_equ{i},gof(i)]= polyfit(time_step',column_vector(:,i),obj.sample_number-1);
                    end
                    fit_equ{end+1} = flipud(y\det_d')';
                else
                    for delete_step = length(null_data_count):-1:1
                        if all(column_vector(null_data_count(delete_step),:)) ==0
                            column_vector(null_data_count(delete_step),:) = [];
                            tmp_time_step(null_data_count(delete_step)) = [];
                        end
                    end
                   
                    
                    for i = 1:size(column_vector,2)
%                         
                        [fit_equ{i},gof(i)]= polyfit(tmp_time_step',column_vector(:,i),obj.sample_number-1);
                    end
                    [fit_equ{end+1},gof(i)]= polyfit(tmp_time_step',det_d',obj.sample_number-1);
                    
                end
                
%                 for i = 1:size(gof,2)
%                     if round(gof(i).normr,obj.tolerance-4) ~= 0
%                         error('Sample data points are not enough and result in bad fitting. Increase the path_fineness variable');
%                     end
%                 end
            else
                fit_equ = [];
            end
        end
        
        %% Numerical_Analytical functions
        %% A function to get sample points
        function poses = PosesData(obj,trajectory,time_range)
            
            time_range = linspace(time_range(1),time_range(2),obj.sample_number);
            for i = 1:size(trajectory,2)
                poses(i,:) = feval(trajectory{i},time_range)';
            end
            
        end
        
        %% A function to get the pose q
        function [pose] = GetInstantPose(obj,time)
            
            for i = 1:size(obj.trajectory,2)
                pose(i) = feval(obj.trajectory{i},time);
            end
            
        end
        
        %% A function to estimate the possible interpolated curve for the RREF data
        function [interpolfit] = PolynomialFitting(obj,Jacobian_Matrics)
            null_data_count = [];
            
            for i = 1:length(Jacobian_Matrics)
                n_n_matrix = Jacobian_Matrics{i}(:,1:end-1);
                n_1_matrix = Jacobian_Matrics{i}(:,end);
                
                matrix_determinant = det(n_n_matrix);
                
                if round(matrix_determinant,obj.tolerance) ~=0
                    column_vector(i,:) = inv(n_n_matrix)*n_1_matrix*matrix_determinant;
                    det_d(i) =  matrix_determinant;
                else
                    null_data_count = [null_data_count,i];
                    det_d(i) = 0;
                    column_vector(i,:) = zeros(1,size(n_n_matrix,1));
                end
                
            end
            
            time_step = linspace(obj.default_time_range(1),obj.default_time_range(2),obj.sample_number);
            tmp_time_step = time_step;
            
            if size(null_data_count,2) ~= size(Jacobian_Matrics,2)
                if isempty(null_data_count)
                    for i = 1:size(column_vector,2)
                        [interpolfit{i},gof(i),~]= fit(time_step',column_vector(:,i),'cubicinterp');
                    end
                    [interpolfit{end+1},gof(i),~]= fit(time_step',det_d','cubicinterp');
                    
                else
                    for delete_step = length(null_data_count):-1:1
                        if all(column_vector(null_data_count(delete_step),:)) ==0
                            column_vector(null_data_count(delete_step),:) = [];
                            tmp_time_step(null_data_count(delete_step)) = [];
                        end
                    end
                    
                    for i = 1:size(column_vector,2)
                        [interpolfit{i},gof(i),~]= fit(tmp_time_step',column_vector(:,i),'cubicinterp');
                    end
                    [interpolfit{end+1},gof(i),~]= fit(obj.time_step',det_d','cubicinterp');
                    
                end
                
                for i = 1:size(gof,2)
                    if round(gof(i).sse,obj.tolerance) ~= 0
                        error('Sample data points are not enough and result in bad fitting. Increase the path_fineness variable');
                    end
                end
            else
                interpolfit = [];
            end
        end
        
        %% A function to handle different DOFs and N+M cables robot
        function TrajectoryFeasibilityVerification(obj,model,Jacobian_Matrics)
            record_zero = [];
            interpolated_function = [];
            
            if model.numDofs + 1 < model.numCables
                check_indices = nchoosek(1:model.numCables,model.numDofs+1);
                for i = 1: size(check_indices,1)
                    for j = 1:size(Jacobian_Matrics,2)
                        remain_Jacobian_Matrics{j} = Jacobian_Matrics{j}(:,check_indices(i,:));
                    end
                    
                    interpolated_function = PolynomialFitting(obj,remain_Jacobian_Matrics);
                    
                    if ~isempty(interpolated_function)
                        [time] = WorkspaceBoundaryTime(obj,interpolated_function);
                        obj.feasible_intervals = [obj.feasible_intervals;time];
                    end
                    
                    record_zero = [];
                    interpolated_function = [];
                end
                
            elseif model.numDofs + 1 == model.numCables
                
                interpolated_function = PolynomialFitting(obj,Jacobian_Matrics);
                
                if ~isempty(interpolated_function)
                    [time] = WorkspaceBoundaryTime(obj,interpolated_function);
                    obj.feasible_intervals = [obj.feasible_intervals;time];
                end
            else
                error('Input Model Error');
            end
            
            if ~isempty(obj.feasible_intervals)
                obj.feasible_intervals = sortrows(obj.feasible_intervals,1);
                obj.feasible_intervals = round(obj.feasible_intervals,obj.tolerance);
                current_interval = obj.feasible_intervals(1,:);
                
                for i = 1:size(obj.feasible_intervals,1)
                    
                    if obj.feasible_intervals(i,1) <= current_interval(end,2) && obj.feasible_intervals(i,1) >= current_interval(end,1) && obj.feasible_intervals(i,2) >= current_interval(end,2)
                        current_interval(end,:) = [current_interval(end,1),obj.feasible_intervals(i,2)];
                    elseif obj.feasible_intervals(i,1) <= current_interval(end,2) && obj.feasible_intervals(i,1) >= current_interval(end,1) && obj.feasible_intervals(i,2) <= current_interval(end,2)
                        %
                    elseif obj.feasible_intervals(i,1) >= current_interval(2)
                        current_interval = [current_interval;obj.feasible_intervals(i,:)];
                    end
                    
                end
                
                obj.feasible_time_range = current_interval;
                
                q_ans = [];
                for j = 1:size(current_interval,1)
                    for i = 1:size(current_interval,2)
                        q_ans = [q_ans;GetInstantPose(obj,current_interval(j,i))];
                        
                    end
                end
                obj.feasible_range = q_ans;
            end
            if isempty(obj.feasible_time_range)
                Message = 'No feasible range inside the workspace';
                disp(Message)
            end
        end
        
        %% A function to find and varify the valid time interval
        function [time] = WorkspaceBoundaryTime(obj,fcn)
            
            polynomial_ans = [];
            time = [];
            continue_count = 1;
            time_ans = obj.default_time_range';
            for i = 1:size(fcn,2)
                tmp_ans = fnzeros(fcn{i}.p);
                polynomial_ans = [polynomial_ans,unique(tmp_ans)'];
            end
            
            for i = 1:length(polynomial_ans)
                if isreal(polynomial_ans(i))
                    if real(polynomial_ans(i)) >= obj.default_time_range(1) && real(polynomial_ans(i)) <= obj.default_time_range(2)
                        time_ans = [time_ans;polynomial_ans(i)];
                    end
                end
            end
            
            time_ans =  unique(time_ans,'rows');
            time_ans = sort(time_ans);
            if size(time_ans,1) == 1
                time_ans = obj.default_time_range;
            end
            
            if length(time_ans) >= 2
                
                for k = 1:length(time_ans)-1
                    
                    
                    for h = 1:size(fcn,2)-1
                        mid_point_test(h) = feval(fcn{h},(time_ans(k)+time_ans(k+1))/2) * feval(fcn{end},(time_ans(k)+time_ans(k+1))/2);
                        lower_bound_test(h) =  feval(fcn{h},time_ans(k)) * feval(fcn{end},time_ans(k));
                        upper_bound_test(h) =  feval(fcn{h},time_ans(k+1)) * feval(fcn{end},time_ans(k+1));
                    end
                    
                    if all(round(mid_point_test,obj.tolerance) < 0) && all(round(lower_bound_test,obj.tolerance) <= 0) && all(round(upper_bound_test,obj.tolerance) <= 0)
                        
                        if continue_count == 1
                            time = [time,[time_ans(k),time_ans(k+1)]];
                        else
                            time = [time;[time_ans(k),time_ans(k+1)]];
                            continue_count = 1;
                        end
                    else
                        continue_count = 0;
                    end
                end
            end
        end
    end
end

%% A function the handle the RREF data from the poses
function J_d = JacobianData(model,poses)
% Update the model according to the path, return the last column data of the model.L
for i = 1:size(poses,2)
    model.update(poses(:,i), zeros(model.numDofs,1), zeros(model.numDofs,1),zeros(model.numDofs,1));
    %                 cable_norm = model.cableLengths';
    %                 L = model.L';
    %                 norm_o_L = diag(cable_norm);
    L{i} = model.L'*diag(model.cableLengths');
    if all(L{i}(find(ismember(model.bodyModel.q_dofType,'ROTATION')),:)==0,'all')
        %             L{i}(find(ismember(model.bodyModel.q_dofType,'ROTATION')),:) = [];
        CASPR_log.Error('Feasible range of point-mass end-effector is the boundary of the attachment points. Make sure the end-effector type is not point-mass')
    end
end
J_d = L;
end
%% Analytical functions

function pose = InstancePose(model,trajectory,time,theta,time_range)
% for j = 1:size(trajectory,2)
%     if model.bodyModel.q_dofType(j) == 'TRANSLATION'
% %         tau = time/tan(theta/2);
%         pose(j,:) = feval(trajectory{j},time);
%     else
%         if theta == 0
%             t = 0;
%         else            
%         t = atan(time*tan(theta/2))*2/theta;
%         end
%         pose(j,:) = feval(trajectory{j},t);
%     end
% end
for j = 1:size(trajectory,2)
    if model.bodyModel.q_dofType(j) == 'TRANSLATION'
        pose(j,:) = feval(trajectory{j},time);
    else        
        orientation_index = find(ismember(model.bodyModel.q_dofType,'ROTATION'));
        for i = 1:size(orientation_index,2)
            rad_Q(:,i) = feval(trajectory{orientation_index(i)},time_range);
        end        
        q_s = angle2quat(rad_Q(1,1),rad_Q(1,2),rad_Q(1,3),'XYZ');
        q_e = angle2quat(rad_Q(2,1),rad_Q(2,2),rad_Q(2,3),'XYZ');        
        k = tan(theta/2)/time_range(2);        
        if theta~=0
            tau = round(atan(k*time)*2/theta,10);
        else
            tau = 0;
        end        
        q_t = quatinterp(q_s,q_e,tau,'slerp');      
        [r1, r2, r3] = quat2angle(q_t, 'XYZ');        
        pose(orientation_index,:) = [r1, r2, r3]';
        break;
    end
end
end

function theta = GetTheta(model,trajectory,time_range)
% get start and end angles
orientation_index = find(ismember(model.bodyModel.q_dofType,'ROTATION'));
for i = 1:size(orientation_index,2)
    rad_Q(:,i) = feval(trajectory{orientation_index(i)},time_range);
    if any(rad_Q(:,i)>= pi)
        m = ['Orientation over 180',char(176), ' is not appicable for this method'];
        CASPR_log.Error(m)
    end
end

q_s = quatnormalize(angle2quat(rad_Q(1,1),rad_Q(1,2),rad_Q(1,3),'XYZ'));
q_e = quatnormalize(angle2quat(rad_Q(2,1),rad_Q(2,2),rad_Q(2,3),'XYZ'));
theta = acos(q_s*q_e');

end

function situtaion = wrench_closure_condition_checking(model,q) 
model.update(q', zeros(model.numDofs,1), zeros(model.numDofs,1),zeros(model.numDofs,1));
row_C = combnk(1:size(model.L,1),size(model.L,2)+1);
L = model.L.';
for i = 1: size(row_C,1)
    k = rref(L(:,row_C(i,:)));
    if isdiag(k(:,1:size(model.L,2))) &&  all(k(:,end)<=0)
        situtaion = 1;
        return;      
    end
    
end
situtaion = 0;
end