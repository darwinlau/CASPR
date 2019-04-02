% The simulator to run a trajectory simulation for rays
%
% Author        : Zeqing ZHANG
% Created       : 2019
% Description   : ray-based trajectory verification
%   
classdef RayTrajectorySimulator < SimulatorBase
    
    properties
        workspace               % Final Workspace (output)
        conditions = []         % A list of conditions to be evaluated for
        metrics = []            % A list of metrics to be evaluated for
        options                 % The workspace simulator options
        graph_rep = []          % The graph representation for the workspace
        node_list = []          % A list of all nodes
        comp_time               % Computational time structure
        translationTrajectories = []
        quaternionPts
        trajectory = []         % Final Trajectory (output)
    end
    
    methods
        % The constructor for the workspace simulator class.
        function w = RayTrajectorySimulator(model,traj,options)
            w@SimulatorBase(model);
            w.quaternionPts = traj{1};
            w.translationTrajectories  = traj{2};
            w.options       = options;
            w.comp_time     = struct('ray_construction',0,'graph_construction',0,'total',0);
        end
        
        % Implementation of the run function
        function run(obj, w_conditions, w_metrics)
            % First determine how big the metrics previously were
            if(isempty(obj.metrics))
                n_metrics_prev  = 0; 
                obj.metrics     = w_metrics;
            else
                n_metrics_prev  = size(obj.metrics,2);
                obj.metrics = [obj.metrics,w_metrics];
            end
            % Determine how big the conditions previously were
            if(~obj.options.read_mode)                
                % Determine how big the conditions previously were
                if(isempty(obj.conditions))
                    n_conditions_prev = 0; 
                    obj.conditions = w_conditions;
                else
                    n_conditions_prev = size(obj.conditions,2);
                    obj.conditions = [obj.conditions,w_conditions];
                end

                % Store the previous workspace
                workspace_prev = obj.workspace;
                % Test if the metrics have infinite limits
                for i = 1:size(w_metrics,2)
                    if((~isempty(w_metrics{i}.metricMax))&&((abs(w_metrics{i}.metricMax)==Inf)||(abs(w_metrics{i}.metricMin)==Inf)))
                        CASPR_log.Print('A metric with infinite limit values cannot be plotted.  To plot please set the metric limit to be finite or filter the workspace after plotting',CASPRLogLevel.WARNING);
                    end
                end
                
                n_trajectories = length(obj.translationTrajectories);
                obj.trajectory = cell(n_trajectories,1);
                workspace_count = 0;
                n_conditions    = length(obj.conditions);
                n_trajectories  = length(obj.translationTrajectories);
                % Determine translation from workspace_in to current metrics
                % list
                
                
                % For each translation trajectory
                for j_t=1:n_trajectories
                    % construct the trajectory
                    tr = RayTrajectoryElement(n_conditions, obj.quaternionPts, obj.translationTrajectories{j_t});
                    for j_c=1:n_conditions
                        %% THIS NEEDS TO BE FILLED IN
                        if(j_c < n_conditions_prev)

                        else
                            % New condition
                            % [condition_type, condition_intervals, comp_time] = obj.conditions{j_c}.evaluate(obj.model,wr);
                            [condition_type, trajectory_intervals, ~] = obj.conditions{j_c}.evaluate(obj.model,tr);
                            %                                     if(~isempty(condition_intervals))
                            %                                         wr.addCondition(condition_type,condition_intervals,j_c);
                            %                                     end
                            if(~isempty(trajectory_intervals))
                                tr.addTrajectory(condition_type,trajectory_intervals,j_c);
                            end
                        end
                    end
                    %%%%%
                    % Determine whether to add the condition to the trajectory
                    test_conditions = cellfun(@isempty,tr.conditions);
                    % Determine whether to add to trajectory
                    if(obj.options.union)
                        entry_condition = (~isempty(obj.metrics)||(sum(test_conditions(:,1))~=n_conditions));
                    else
                        entry_condition = (sum(test_conditions(:,1))==0);
                    end
                    if(entry_condition)
                        % Add the trajectory point to the
                        % obj.workspace{k} = wr;
                        obj.trajectory{j_t} = tr;
                        workspace_count = workspace_count + 1;
                    end
                    %%%%%
                end
            end
        end

    
        function plotRayWorkspace2Cond(obj,startEndPts,plot_axis)
            % % %             numDofs=obj.grid.n_dimensions;
            if(nargin<2)
                plot_axis=[1,2,3];
            end
            axis1=[];
            axis2=[];
            axis3=[];
            [size_trajectory, ~]=size(obj.trajectory);
            q0_given = obj.quaternionPts(1,:);
            q1_given = obj.quaternionPts(2,:);
            cosAgl = sum(q0_given .* q1_given);
            theta = acos(cosAgl);
            
            for it=1:size_trajectory
                if ~isempty(obj.trajectory{it})
                    %%% w.quaternionPts = traj{1};
                    %%% w.translationTrajectories  = traj{2};
                    C_wrt_T = obj.translationTrajectories{it};
                    %
                    % ---- intersecting set ----
                    L_A = obj.trajectory{it}.conditions{1,2}(:,1);
                    R_A = obj.trajectory{it}.conditions{1,2}(:,2);
                    L_B = obj.trajectory{it}.conditions{2,2}(:,1);
                    R_B = obj.trajectory{it}.conditions{2,2}(:,2);
                    if ~isempty(L_A) && ~isempty(L_B)
                        [L_res, R_res] = or_and_or(obj, L_A, R_A, L_B, R_B);
                        if ~isempty(L_res)
                            tmp = round(L_res - R_res, 10);
                            L_res(tmp==0) =[];
                            R_res(tmp==0) =[];
                            P_IFCWCC = [L_res' R_res']; % 2 columns matrix
                        else
                            P_IFCWCC = [];
                        end
                    else
                        P_IFCWCC = [];
                    end
                    
                    minVal = 0;
                    maxVal = 1;
                    feasiblePath = P_IFCWCC;
                    if isempty(P_IFCWCC)
                        infeasiblePath = [minVal, maxVal];
                    else
                        [L_comp, R_comp] = Comp_interval(obj, P_IFCWCC(:,1), P_IFCWCC(:,2));
                        if ~isempty(L_comp)
                            % results from fun'or_and_or' is closed intervals
                            [resultL, resultR] = or_and_or(obj, minVal, maxVal, L_comp, R_comp);
                            % avoid the adjcent intervals
                            % [resultL, resultR] = Or_interval(resultL, resultR);
                            % modify to open set (delete the isolated point)
                            tmp = round(resultL - resultR, 10);
                            resultL(tmp==0) =[];
                            resultR(tmp==0) =[];
                            infeasiblePath = [resultL', resultR'];
                        else
                            infeasiblePath = [];
                        end
                    end
                    % ----intersecting set ----
                    %
                    
                    %% plot resulting translational paths
                    k = 20;
                    feasiblePath_T = tan(feasiblePath*theta/2);
                    for part = 1:size(feasiblePath_T,1)
                        cc=1;
                        T_start = feasiblePath_T(part,1);
                        T_terminal = feasiblePath_T(part,2);
                        for cnt = T_start:(T_terminal-T_start)/k:T_terminal
                            x(cc) = polyval(C_wrt_T(1,:),cnt);
                            y(cc) = polyval(C_wrt_T(2,:),cnt);
                            z(cc) = polyval(C_wrt_T(3,:),cnt);
                            cc=cc+1;
                        end
                        plot3(x,y,z,'g','LineWidth', 2.5)
                        hold on
                    end
                    
                    %% plot the infeasible paths
                    clear x y z cc
                    infeasiblePath_T = tan(infeasiblePath*theta/2);
                    for part = 1:size(infeasiblePath_T,1)
                        cc=1;
                        T_start = infeasiblePath_T(part,1);
                        T_terminal = infeasiblePath_T(part,2);
                        for cnt = T_start:(T_terminal-T_start)/k:T_terminal
                            x(cc) = polyval(C_wrt_T(1,:),cnt);
                            y(cc) = polyval(C_wrt_T(2,:),cnt);
                            z(cc) = polyval(C_wrt_T(3,:),cnt);
                            cc=cc+1;
                        end
                        % plot3(x,y,z,'r','LineWidth', 2) % in red
                        plot3(x,y,z,'Color',[0.9 0.9 0.9],'LineWidth', 1) % in gray
                        hold on
                    end
                    
                    %% plot the initial pt
                    plot3(startEndPts{it}(1,1),startEndPts{it}(1,2),startEndPts{it}(1,3), '.k', 'MarkerSize',15)
                    hold on
                    % plot3(ones(3,1)*startEndPts{it}(1,1),[ones(2,1)*startEndPts{it}(1,2);0],[startEndPts{it}(1,3);0;0], ':k')
                    % hold on
                    % plot3([0;startEndPts{it}(1,1)],ones(2,1)*startEndPts{it}(1,2),[0;0], ':k')
                    % hold on
                    % text(startEndPts{it}(1,1)+0.2,startEndPts{it}(1,2),startEndPts{it}(1,3)+0.2,'Initial','FontSize',20)
                    
                    %% plot the terminal pt
                    plot3(startEndPts{it}(end,1),startEndPts{it}(end,2),startEndPts{it}(end,3), '.k', 'MarkerSize',15)
                    hold on
                    % plot3(ones(3,1)*startEndPts{it}(end,1),[ones(2,1)*startEndPts{it}(end,2);0],[startEndPts{it}(end,3);0;0], ':k')
                    % hold on
                    % plot3([0;startEndPts{it}(end,1)],ones(2,1)*startEndPts{it}(end,2),[0;0], ':k')
                    % hold on
                    % text(startEndPts{it}(end,1)+0.2,startEndPts{it}(end,2),startEndPts{it}(end,3)+0.2,'Terminal','FontSize',20)
                    str = num2str(it);
                    text(startEndPts{it}(end,1)+0.05,startEndPts{it}(end,2),startEndPts{it}(end,3)+0.2,str)
                end
            end
            axis([0,4,0,4,0,4])
            axis equal
            xlabel('x')
            ylabel('y')
            zlabel('z')
            view(0,90)
            grid on
        end
        
        function plotCoeffWorkspaceinTranslation(obj)
            for i = 1: size(obj.trajectory,1)
                if obj.trajectory{i, 1}.conditions{2,2}(2) == 1 && obj.trajectory{i, 1}.conditions{1,2}(2)  == 1
                    t = linspace(obj.trajectory{i, 1}.conditions{2,2}(1),obj.trajectory{i, 1}.conditions{2,2}(2),100);
                    for j = 1:size(obj.trajectory{i, 1}.translationTrajectory,1)
                        var_t{i,:}(j,:) = polyval(obj.trajectory{i,1}.translationTrajectory(j,:),t);
                    end
                    graph_plot(i) = plot3(var_t{i,:}(1,:),var_t{i,:}(2,:),var_t{i,:}(3,:),'black','LineWidth',1.2);
                    hold on
                    grid on
                else
%                     i
                end               
                
            end
            title('Coefficient Ray Workspace(Translation View)')
        end
        
        function plotCurvedRayWorkspace(obj)
            
            for i = 1: size(obj.trajectory,1)
                total_time_range_ifc = unique([obj.trajectory{i, 1}.conditions{1,2},1]);
                total_time_range_wcc = unique([obj.trajectory{i, 1}.conditions{2,2},1]);
                time_range = sort(unique([total_time_range_ifc,total_time_range_wcc]));
                
                feasible_time_range = linspace(time_range(1),time_range(2),100);
                if size(time_range,2) > 2
                    infeasible_time_range = linspace(time_range(2),time_range(end),100);
                end
                for j = 1:size(obj.trajectory{i,1}.translationTrajectory,1)
                    feasible_coord{i,:}(j,:) = polyval(obj.trajectory{i,1}.translationTrajectory(j,:),feasible_time_range);
                    if exist('infeasible_time_range')
                        infeasible_coord{i,:}(j,:) = polyval(obj.trajectory{i,1}.translationTrajectory(j,:),infeasible_time_range);
                    end
                end
                graph_plot_f(i) = plot3(feasible_coord{i,:}(1,:),feasible_coord{i,:}(2,:),feasible_coord{i,:}(3,:),'g','LineWidth',1.2);
                hold on
                grid on
                if exist('infeasible_time_range')
                    graph_plot_i(i) = plot3(infeasible_coord{i,:}(1,:),infeasible_coord{i,:}(2,:),infeasible_coord{i,:}(3,:),'--','Color',[0.7 0.7 0.7],'LineWidth',1.2);
                end               
                clear infeasible_time_range feasible_time_range time_range total_time_range_wcc total_time_range_ifc
            end
            title('Curved Ray Workspace(Translation View)')
        end
        function [resultL, resultR] = or_and_or(obj, L_A, R_A, L_B, R_B)
            % make sure elements in A and B are non-intersecting
            [AL, AR] = Or_interval(obj, L_A, R_A); % Input of Or_interval fn is not bounded, see 'Or_interval'
            [BL, BR] = Or_interval(obj, L_B, R_B); % Output of Or_interval fn could be null sets, see 'Or_interval'
            % -- V.2.0 --
            if ~isempty(AL) && ~isempty(BL) % if AL is not empty, then AR is not empty as well
                t =1;
                storeL = cell(1, length(AL)*length(BL));
                storeR = cell(1, length(AL)*length(BL));
                for i = 1: length(AL)
                    for j = 1: length(BL) % AL(i), from the output of Or_interval fn, refers to one specific interval and non-null set, so do AR(i),BL(j) and BR(j)
                        [storeL{t}, storeR{t}] = And_interval(obj, [AL(i) BL(j)], [AR(i) BR(j)]); % null sets CAN be inputted into cell{t}
                        t = t +1;
                    end
                end
                [resultL, resultR] = Or_interval(obj, [storeL{1, :}], [storeR{1, :}]);
            else
                resultL = [];
                resultR = [];
            end
        end
        
        function [L_or, R_or] = Or_interval(obj, L, R)
            if length(L) ~= length(R)
                CASPR_log.Print('Invaid Input',CASPRLogLevel.ERROR);
            elseif max(L > R)
                CASPR_log.Print('Plz make sure L <= R',CASPRLogLevel.ERROR);
            elseif isempty(L) % the input is one null interval
                disp('[WARNING] Empty Input') % union(a null space)
                L_or = [];
                R_or = [];
            else  % the input is not one null interval so Note-3 holds
                % sort input in ascending order
                [L, I] = sort(L);
                R_temp = zeros(1, length(L));
                for i = 1: length(L)
                    index = I(i);
                    R_temp(i) = R(index);
                end
                R = R_temp;
                % cal. union interval ONE by ONE
                t = 1;
                L_store(t) = L(1); % store the finial results
                R_store(t) = R(1);
                for i = 1: length(L)-1
                    % L(i+1) and R(i+1) are non-null due to Note-3; L_store(t) and
                    % R_store(t) are also non-null b/c the origin of them as follows
                    temp = And_interval(obj, [L_store(t) L(i+1)], [R_store(t) R(i+1)]); 
                    if isempty(temp) % two intervals are non-intersecting
                        L_store(t+1) = L(i+1); % L(i+1) is non-null, so L_store(t+1) is non-null
                        R_store(t+1) = R(i+1);
                        t = t + 1;
                    else % two intervals have intersections
                        [L_2inter, R_2inter] = Or_interval_2(obj, [L_store(t) L(i+1)], [R_store(t) R(i+1)]); % result is one interval
                        L_store(t) = L_2inter; R_store(t) = R_2inter;
                    end
                end
                L_or = L_store;
                R_or = R_store;
            end
        end

        function [L_or, R_or] = Or_interval_2(~, L, R)
            if length(L) ~= length(R)
                CASPR_log.Print('Invaid Input',CASPRLogLevel.ERROR);
            elseif max(L > R)
                CASPR_log.Print('Plz make sure L <= R',CASPRLogLevel.ERROR);
            elseif isempty(L) % both L and R are empty could happen; one of them is empty would get error from the condition 'length(L) ~= length(R)'
                disp('[WARNING] Empty Input') % union(null space)
                L_or = [];
                R_or = [];
            else 
                l_fix = L(1);
                r_fix = R(1);
                l_var = L(2);
                r_var = R(2);
                if r_var < l_fix      % 2 non-intersecting intervals
                    L_or(1) = l_var;
                    R_or(1) = r_var;
                    L_or(2) = l_fix;
                    R_or(2) = r_fix;
                elseif l_var > r_fix  % 2 non-intersecting intervals
                    L_or(1) = l_fix;
                    R_or(1) = r_fix;
                    L_or(2) = l_var;
                    R_or(2) = r_var;
                else                  % combine 2 intervals into one interval
                    l_temp = min(L);
                    r_temp = max(R);        
                    L_or(1) = l_temp;
                    R_or(1) = r_temp;
                end
            end
        end

        function [L_and, R_and] = And_interval(~, L, R)
            if length(L) ~= length(R)
                CASPR_log.Print('Invaid Input',CASPRLogLevel.ERROR);
            elseif max(L > R)
                CASPR_log.Print('Plz make sure L <= R',CASPRLogLevel.ERROR);
            % elseif isempty(option) || strcmp(option, 'and') % default of option is 'and'
            elseif isempty(L) % One null set could happen; one of them is empty would get error from the condition 'length(L) ~= length(R)'
                disp('[WARNING] Empty Input') % inters(one null set); see Input-2)
                L_and = [];
                R_and = [];
            else % the input is not one null interval
                L_and = max(L);
                R_and = min(R);
                if L_and > R_and
                    L_and = [];
                    R_and = [];
                end
            end
        end
        
        function [L_comp, R_comp] = Comp_interval(obj, L, R)
            if length(L) ~= length(R)
                CASPR_log.Print('Invaid Input',CASPRLogLevel.ERROR);
            elseif max(L > R)
                CASPR_log.Print('Plz make sure L <= R',CASPRLogLevel.ERROR);
            elseif isempty(L) % both L and R are empty could happen; one of them is empty would get error from the condition 'length(L) ~= length(R)'
                disp('[WARNING] Empty Input')  % comp(null)
                L_comp = -inf;
                R_comp = +inf;
            else % the input is not one null interval, then Note 4 holds
                % cal. union intervals and in the ascending order
                [L_ABC, R_ABC] = Or_interval(obj, L, R);   % to make sure L_ABC(i) and R_ABC(i) are non-intersecting non-null
                % initial value of L_accum and R_accum is comp. set of 1st interval
                [L_accum, R_accum] = Comp_interval_12(obj, L_ABC(1), R_ABC(1)); % L_ABC(1) refers to one interval
                % cal. inters(A^c, B^c), where A^c refers to [L_accum, R_accum]
                for i= 2: length(L_ABC)
                    temp1 = Comp_interval_12(obj, L_ABC(i), R_ABC(i));
                    if ~isempty(temp1) && ~isempty(L_accum)
                        % cal. B^c, which is [L_comp_B, R_comp_B]
                        [L_comp_B, R_comp_B] = Comp_interval_12(obj, L_ABC(i), R_ABC(i));
                        % cal. inters(A^c, B^c), the results go to L_accum etc.;
                        % Note that A^c and B^c could be more than one intervals respectively, use the cell instead of the array
                        L_mat = {L_accum; L_comp_B};
                        R_mat = {R_accum; R_comp_B};
                        L_A = L_mat{1, :};
                        R_A = R_mat{1, :};
                        for n = 1 : size(L_mat, 1)-1
                            [resultL, resultR] = or_and_or(obj, L_A, R_A, L_mat{n+1, :}, R_mat{n+1, :});
                            L_A = resultL;
                            R_A = resultR;
                        end
                        L_accum = L_A;
                        R_accum = R_A;
                    else % once A^c or B^c are null sets, then terminate the loop and final result is the null set
                        L_accum = [];
                        R_accum = [];
                        break;
                    end
                end
                L_comp = L_accum;
                R_comp = R_accum;
            end
        end
        
        function [L_comp, R_comp] = Comp_interval_12(obj, L, R)
            if length(L) ~= length(R)
                CASPR_log.Print('Invaid Input',CASPRLogLevel.ERROR);
            elseif max(L > R)
                CASPR_log.Print('Plz make sure L <= R',CASPRLogLevel.ERROR);
            elseif isempty(L) % both L and R are empty could happen; one of them is empty would get error from the condition 'length(L) ~= length(R)'
                disp('[WARNING] Empty Input')  % comp(null set)
                L_comp = -inf;
                R_comp = +inf;
            elseif length(L) == 1
                % 4 cases for an interval
                if L == -inf && R ~= +inf
                    L_comp(1) = R;
                    R_comp(1) = +inf;
                elseif L~= -inf && R == +inf
                    L_comp(1) = -inf;
                    R_comp(1) = L;
                elseif L == -inf && R == +inf
                    L_comp = [];
                    R_comp = [];
                else
                    L_comp(1) = -inf;
                    R_comp(1) = L;
                    L_comp(2) = R;
                    R_comp(2) = +inf;
                end
            elseif length(L) == 2 % two non-null intervals
                temp = And_interval(obj, L, R); % L and R are non-null
                if isempty(temp) % non-intersecting
                    % suppose L(1) < L(2)
                    if L(1) > L(2)
                        temp = L(1);
                        L(1) = L(2);
                        L(2) = temp;
                        temp = R(1);
                        R(1) = R(2);
                        R(2) = temp;
                    end
                    % 4 cases for 2 non-intersecting intervals
                    if L(1) ~= -inf && R(2) ~= +inf
                        L_comp(1) = -inf;
                        R_comp(1) = L(1);
                        L_comp(2) = R(1);
                        R_comp(2) = L(2);
                        L_comp(3) = R(2);
                        R_comp(3) = +inf;
                    elseif L(1) ~= -inf && R(2) == +inf
                        L_comp(1) = -inf;
                        R_comp(1) = L(1);
                        L_comp(2) = R(1);
                        R_comp(2) = L(2);
                    elseif L(1) == -inf && R(2) ~= +inf
                        L_comp(1) = R(1);
                        R_comp(1) = L(2);
                        L_comp(2) = R(2);
                        R_comp(2) = +inf;
                    elseif  L(1) == -inf && R(2) == +inf
                        L_comp(1) = R(1);
                        R_comp(1) = L(2);
                        
                    end
                else
                    CASPR_log.Print('Make sure 2 non-intersecting intervals',CASPRLogLevel.ERROR);
                end
            else
                CASPR_log.Print('Make sure the number of inputs <= 2',CASPRLogLevel.ERROR);
            end
        end
        
    end
end
