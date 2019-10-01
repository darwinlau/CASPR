% Algorithm to to compute the trajectory feasible range under cable interference free condition
%
% Author        : Paul Cheng
% Created       : 2019
% Description   : The class for evaluation of IFC
classdef TrajectoryCableInterferenceFreeCondition< TrajectoryConditionBase
    properties (Constant)
        % Type of workspace condition (TrajectoryConditionType enum)
        type = TrajectoryConditionType.CABLE_INTERFERENCE_FREE;
    end
    
    properties
        method
        sample_number            % number of sample point(affect accuracy)
        interference_free_time_range = [];      % feasible time range under IFC
        interference_free_range = [];           % feasible pose range under IFC
        min_cable_dist = [];     % minimum cable distance
        theta = [];              % only for analytical method
    end
    properties (Access = private)
        tolerance = 9;           % error tolerance, digits
        default_time_range = []; % default_time_range from calculated input
        intersected_time = [];   % local use, sets of calculated solutions
        trajectory = [];         % cfit type functions
    end
    
    methods
        % Constructor for Interference Free WS
        function t = TrajectoryCableInterferenceFreeCondition(method,sample_number,min_cable_dist)
            if (isempty(method))
                t.method = TrajectoryAnalysisMethodType.NUMERICAL_ANALYTICAL;
            else
                t.method = method;
            end
            t.sample_number = sample_number;
            t.min_cable_dist = min_cable_dist;
        end
        
        % Evaluate the Interference Free condition return true if
        % satisified
        function [feasible_range,feasible_time_range,other_info,theta] = evaluateFunction(obj,model,trajectory,time_range,maximum_trajectory_degree)
            obj.interference_free_time_range = [];
            obj.interference_free_range = [];
            obj.tolerance = 9;
            obj.default_time_range = [0 1];
            obj.trajectory = [];
            obj.intersected_time = [];
            obj.trajectory = trajectory;
            obj.default_time_range = time_range;
            theta = [];
            
            switch obj.method
                case TrajectoryAnalysisMethodType.NUMERICAL_ANALYTICAL
                    obj.default_time_range = [0 1];
                    poses = PosesData(obj,trajectory,obj.default_time_range);
                    cable_seg_data = CableSegmentData(model,poses);
                    obj.intersected_time = InterferenceTimeCalculation(obj,model,cable_seg_data);
                    [obj.interference_free_time_range,intersection_info] = InterferenceTimeVerify_NA(obj,model,obj.intersected_time);
                    feasible_time_range = obj.interference_free_time_range;
                    feasible_range = obj.interference_free_range;
                    other_info = intersection_info;
                case TrajectoryAnalysisMethodType.ANALYTICAL
                    [R,theta] = RotationMatrixCoefficient(model,obj.trajectory,obj.default_time_range,obj.tolerance);
                    T = TranslationTrajectoryCoefficient(model,obj.trajectory,obj.default_time_range,maximum_trajectory_degree,theta);
                    [orientation_time_interval,translation_time_interval,obj.interference_free_time_range,feasible_range,intersection_info] = ...
                        IntervalCompuation(obj,model,R,T,obj.min_cable_dist,theta,maximum_trajectory_degree);
                    feasible_time_range = obj.interference_free_time_range;
                    obj.interference_free_range = feasible_range;
                    other_info = intersection_info;
                    obj.theta = theta;
                otherwise
                    CASPR_log.Error('Invalid analysis method');
            end
        end
        %% Analytical functions
        % A function to find out the interference free time intervals
        function [orientation_time_interval,translation_time_interval,t_interval,feasible_range,intersected_info] = IntervalCompuation(obj,model,R,T,min_dis,theta,max_deg)
%             C_D = [1 0 2 0 1]; % common_denominator
                k = tan(theta/2)/obj.default_time_range(2);
                C_D = [k^4 0 2*k^2 0 1]; % common_denominator
            t_ans = obj.default_time_range(:);feasible_range = [];
            for i = 1:model.numCables
                r_GA_i = model.cableModel.cables{1,i}.attachments{1,2}.r_GA';
                r_OA_i = model.cableModel.cables{1,i}.attachments{1,1}.r_OA';
                for j = i+1:model.numCables
                    r_GA_j = model.cableModel.cables{1,j}.attachments{1,2}.r_GA';
                    r_OA_j = model.cableModel.cables{1,j}.attachments{1,1}.r_OA';
                    % S_i = T(T) + R(T)*r_GA - r_OA
                    Si(1,:) = SumCoeff({conv(T(1,:),C_D);(R(1,:)*r_GA_i(1) + R(2,:)*r_GA_i(2) + R(3,:)*r_GA_i(3));-C_D*r_OA_i(1)});
                    Si(2,:) = SumCoeff({conv(T(2,:),C_D);(R(4,:)*r_GA_i(1) + R(5,:)*r_GA_i(2) + R(6,:)*r_GA_i(3));-C_D*r_OA_i(2)});
                    Si(3,:) = SumCoeff({conv(T(3,:),C_D);(R(7,:)*r_GA_i(1) + R(8,:)*r_GA_i(2) + R(9,:)*r_GA_i(3));-C_D*r_OA_i(3)});
                    
                    Sj(1,:) = SumCoeff({conv(T(1,:),C_D);(R(1,:)*r_GA_j(1) + R(2,:)*r_GA_j(2) + R(3,:)*r_GA_j(3));-C_D*r_OA_j(1)});
                    Sj(2,:) = SumCoeff({conv(T(2,:),C_D);(R(4,:)*r_GA_j(1) + R(5,:)*r_GA_j(2) + R(6,:)*r_GA_j(3));-C_D*r_OA_j(2)});
                    Sj(3,:) = SumCoeff({conv(T(3,:),C_D);(R(7,:)*r_GA_j(1) + R(8,:)*r_GA_j(2) + R(9,:)*r_GA_j(3));-C_D*r_OA_j(3)});
                    
                    Sij = r_OA_j' - r_OA_i';
                    % cross product of Si Sj
                    SiXSj(1,:) = SumCoeff({conv(Si(2,:),Sj(3,:)); -conv(Si(3,:),Sj(2,:))});
                    SiXSj(2,:) = SumCoeff({conv(Si(3,:),Sj(1,:)); -conv(Si(1,:),Sj(3,:))});
                    SiXSj(3,:) = SumCoeff({conv(Si(1,:),Sj(2,:)); -conv(Si(2,:),Sj(1,:))});
                    % non-parallel case
                    % determinants in equation (4)
                    n_ti = SumCoeff({Sij(1)*conv(-Sj(2,:),-SiXSj(3,:)) ;
                        Sij(3)*conv(-Sj(1,:),-SiXSj(2,:)) ;
                        Sij(2)*conv(-Sj(3,:),-SiXSj(1,:)) ;
                        -Sij(3)*conv(-Sj(2,:),-SiXSj(1,:)) ;
                        -Sij(2)*conv(-Sj(1,:),-SiXSj(3,:)) ;
                        -Sij(1)*conv(-Sj(3,:),-SiXSj(2,:)) });
                    n_ti(1:(length(n_ti)-max_deg*2-13)) = [];
                    n_tj = SumCoeff({Sij(2)*conv(Si(1,:),-SiXSj(3,:)) ;
                        Sij(1)*conv(Si(3,:),-SiXSj(2,:)) ;
                        Sij(3)*conv(Si(2,:),-SiXSj(1,:)) ;
                        -Sij(2)*conv(Si(3,:),-SiXSj(1,:)) ;
                        -Sij(1)*conv(Si(2,:),-SiXSj(3,:)) ;
                        -Sij(3)*conv(Si(1,:),-SiXSj(2,:)) });
                    n_tj(1:(length(n_tj)-max_deg*2-13)) = [];
                    n_t = SumCoeff({Sij(3)*conv(Si(1,:),-Sj(2,:));
                        Sij(2)*conv(Si(3,:),-Sj(1,:)) ;
                        Sij(1)*conv(Si(2,:),-Sj(3,:)) ;
                        -Sij(1)*conv(Si(3,:),-Sj(2,:)) ;
                        -Sij(3)*conv(Si(2,:),-Sj(1,:)) ;
                        -Sij(2)*conv(Si(1,:),-Sj(3,:)) });
                    n_t(1:(length(n_t)-max_deg-9)) = [];
                    d =  SumCoeff({conv(-SiXSj(3,:),conv(Si(1,:),-Sj(2,:))) ;
                        conv(-SiXSj(2,:),conv(Si(3,:),-Sj(1,:))) ;
                        conv(-SiXSj(1,:),conv(Si(2,:),-Sj(3,:))) ;
                        conv(+SiXSj(1,:),conv(Si(3,:),-Sj(2,:))) ;
                        conv(+SiXSj(3,:),conv(Si(2,:),-Sj(1,:))) ;
                        conv(+SiXSj(2,:),conv(Si(1,:),-Sj(3,:))) });
                    d(1:(length(d)-max_deg*2-17)) = [];
                    % parallel case
                    % Si cross Sij
                    SiXSij(1,:) = SumCoeff({conv(Si(2,:),Sij(3,:)); -conv(Si(3,:),Sij(2,:))});
                    SiXSij(2,:) = SumCoeff({conv(Si(3,:),Sij(1,:)); -conv(Si(1,:),Sij(3,:))});
                    SiXSij(3,:) = SumCoeff({conv(Si(1,:),Sij(2,:)); -conv(Si(2,:),Sij(1,:))});
                    p_c = SumCoeff({obj.min_cable_dist^2*SumCoeff({conv(Si(1,:),Si(1,:));conv(Si(2,:),Si(2,:));conv(Si(3,:),Si(3,:))});...
                        SumCoeff({conv(SiXSij(1,:),SiXSij(1,:));conv(SiXSij(2,:),SiXSij(2,:));conv(SiXSij(3,:),SiXSij(3,:))})});
                    p_c_roots = roots(p_c);p_c_roots = p_c_roots(imag(p_c_roots)==0);
                    n_ti_roots = roots(n_ti);n_ti_roots = n_ti_roots(imag(n_ti_roots)==0);
                    n_tj_roots = roots(n_tj);n_tj_roots = n_tj_roots(imag(n_tj_roots)==0);
                    nti_d_roots = roots(SumCoeff({d;-n_ti}));nti_d_roots = nti_d_roots(imag(nti_d_roots)==0);
                    ntj_d_roots = roots(SumCoeff({d;-n_ti}));ntj_d_roots = ntj_d_roots(imag(ntj_d_roots)==0);
                    eplsi_roots = roots(SumCoeff({min_dis^2*d;-conv(n_t,n_t)}));eplsi_roots = eplsi_roots(imag(eplsi_roots)==0);
                    
                    intersected_time_instance = [p_c_roots;n_ti_roots;n_tj_roots;nti_d_roots;ntj_d_roots;eplsi_roots];
%                     intersected_time_instance = [unique(real(roots(p_c)));...
%                         unique(real(roots(n_ti)));...
%                         unique(real(roots(n_tj)));...
%                         unique(real(roots(SumCoeff({d;-n_ti}))));...
%                         unique(real(roots(SumCoeff({d;-n_tj}))));...
%                         unique(real(roots(SumCoeff({min_dis^2*d;-conv(n_t,n_t)}))))];
                    intersected_time_instance = intersected_time_instance(intersected_time_instance>=obj.default_time_range(1));
                    intersected_time_instance = intersected_time_instance(intersected_time_instance<=obj.default_time_range(2));
                    t_ans = [t_ans;intersected_time_instance];
                    
                end
            end
            
            t_ans = sort(unique(t_ans));
            [t_interval,intersected_info] = InterferenceTimeVerify_A(obj,model,t_ans,theta);
            orientation_time_interval = atan(t_interval*tan(theta/2)).*2./theta;
            translation_time_interval = t_interval;
            for i = 1:size(t_interval,1)
                feasible_range = [feasible_range;InstancePose(model,obj.trajectory,t_interval(i,1),theta,obj.default_time_range)';InstancePose(model,obj.trajectory,t_interval(i,2),theta,obj.default_time_range)'];
            end
        end
        % A functin to verifiy to feasible range
        function [IFC_intervals,Interference_interval] = InterferenceTimeVerify_A(obj,model,intersected_time,theta)
            Interference_interval = []; IFC_intervals = [];
            for i = 1:size(intersected_time,1)-1
                mid_time = 0.5*(intersected_time(i) + intersected_time(i+1));
                mid_pose = InstancePose(model,obj.trajectory,mid_time,theta,[intersected_time(1),intersected_time(end)]);

                model.update(mid_pose, zeros(model.numDofs,1), zeros(model.numDofs,1),zeros(model.numDofs,1));
                
                epsilon = [];
                for j = 1:model.numCables
                    for k = j+1:model.numCables
                        cable_j_seg_end_pt(:,1) = model.cableModel.cables{1,j}.attachments{1,1}.r_OA;
                        cable_j_seg_end_pt(:,2) = model.cableModel.cables{1,j}.attachments{1,2}.r_OA;
                        cable_k_seg_end_pt(:,1) = model.cableModel.cables{1,k}.attachments{1,1}.r_OA;
                        cable_k_seg_end_pt(:,2) = model.cableModel.cables{1,k}.attachments{1,2}.r_OA;
                        epsilon = [epsilon;DistBetween2Segment(cable_j_seg_end_pt(:,1), cable_j_seg_end_pt(:,2),...
                            cable_k_seg_end_pt(:,1), cable_k_seg_end_pt(:,2)),j,k];
                        epsilon = round(epsilon,5);
                        if epsilon(end,1) <= obj.min_cable_dist
                            if isempty(Interference_interval)
                                Interference_interval = [Interference_interval;intersected_time(i),intersected_time(i+1),j,k];
                            elseif Interference_interval(end,end-2) == intersected_time(i)
                                Interference_interval(end,end-2) = intersected_time(i+1);
                            else
                                Interference_interval = [Interference_interval;intersected_time(i),intersected_time(i+1),j,k];
                            end
                        end
                    end
                end
                if all(epsilon(:,1) >= obj.min_cable_dist)
                    if isempty(IFC_intervals)
                        IFC_intervals = [IFC_intervals;intersected_time(i),intersected_time(i+1)];
                    elseif IFC_intervals(end,end) == intersected_time(i)
                        IFC_intervals(end,end) = intersected_time(i+1);
                    else
                        IFC_intervals = [IFC_intervals;intersected_time(i),intersected_time(i+1)];
                    end
                end
            end
        end
        
        %% Numerical_Analytical functions
        % A functin to verifiy to feasible range
        function [IFC_intervals,Interference_interval] = InterferenceTimeVerify_NA(obj,model,intersected_time)
            Interference_interval = []; IFC_intervals = [];
            for i = 1:size(intersected_time,1) - 1
                if intersected_time(i) >= 0 && intersected_time(i) <= 1
                    mid_time = 0.5*(intersected_time(i) + intersected_time(i+1));
                    for j = 1:size(obj.trajectory,2)
                        mid_pose(j,:) = feval(obj.trajectory{j},mid_time);
                    end
                    model.update(mid_pose, zeros(model.numDofs,1), zeros(model.numDofs,1),zeros(model.numDofs,1));
                    
                    %                     for j = 1:model.numCables
                    %                         cable_seg{j,:} = model.cableModel.cables{1,j}.segments{1,1}.segmentVector;
                    %                     end
                    epsilon = [];
                    for j = 1:model.numCables
                        for k = j+1:model.numCables
                            cable_j_seg_end_pt(:,1) = model.cableModel.cables{1,j}.attachments{1,1}.r_OA;
                            cable_j_seg_end_pt(:,2) = model.cableModel.cables{1,j}.attachments{1,2}.r_OA;
                            cable_k_seg_end_pt(:,1) = model.cableModel.cables{1,k}.attachments{1,1}.r_OA;
                            cable_k_seg_end_pt(:,2) = model.cableModel.cables{1,k}.attachments{1,2}.r_OA;
                            epsilon = [epsilon;DistBetween2Segment(cable_j_seg_end_pt(:,1), cable_j_seg_end_pt(:,2),...
                                cable_k_seg_end_pt(:,1), cable_k_seg_end_pt(:,2)),j,k];
                            
                            if epsilon(end,1) <= obj.min_cable_dist
                                if isempty(Interference_interval)
                                    Interference_interval = [Interference_interval;intersected_time(i),intersected_time(i+1),j,k];
                                elseif Interference_interval(end,end-2) == intersected_time(i)
                                    Interference_interval(end,end-2) = intersected_time(i+1);
                                else
                                    Interference_interval = [Interference_interval;intersected_time(i),intersected_time(i+1),j,k];
                                end
                            end
                        end
                    end
                    
                else
                    % for the case where the min distance is not in the cable seg, but is that usefull to know?
                end
                if all(epsilon(:,1) >= obj.min_cable_dist)
                    if isempty(IFC_intervals)
                        IFC_intervals = [IFC_intervals;intersected_time(i),intersected_time(i+1)];
                    elseif IFC_intervals(end,end) == intersected_time(i)
                        IFC_intervals(end,end) = intersected_time(i+1);
                    else
                        IFC_intervals = [IFC_intervals;intersected_time(i),intersected_time(i+1)];
                    end
                end
            end
            for i = 1:size(IFC_intervals,1)*2
                [ind_1,ind_2] = ind2sub(size(IFC_intervals),i);
                for j = 1:size(obj.trajectory,2)
                    obj.interference_free_range(i,j) = feval(obj.trajectory{j},IFC_intervals(ind_1,ind_2));
                end
            end
        end
        
        %% A function to find out the condition's polynomial and it
        % solution
        function t_ans = InterferenceTimeCalculation(obj,model,cable_seg_data)
            t_ans = obj.default_time_range(1); epsilon = [];
            equ_num = 1;
            time_steps = linspace(obj.default_time_range(1),obj.default_time_range(2),obj.sample_number);
            for i = 1:model.numCables
                for j = i+1:model.numCables
                    for k = 1:size(cable_seg_data{i,:},2)
                        S_ij = model.cableModel.cables{1,i}.attachments{1,1}.r_OA -  model.cableModel.cables{1,j}.attachments{1,1}.r_OA;
                        
                        % Benji's method but in numerical approach
                        %                         M = [cable_seg_data{i,:}(:,k), -cable_seg_data{j,:}(:,k), -cross(cable_seg_data{i,:}(:,k),cable_seg_data{j,:}(:,k))];
                        %                         sampling the value of the
                        %                         five condtions in the
                        %                         paper
                        %
                        %                         d(k) = det(M);
                        %                         tmp = inv(M)*det(M)*S_ij;
                        %                         n_ti(k) = tmp(1);
                        %                         n_tj(k) = tmp(2);
                        %                         n_t(k) = tmp(3);
                        %                                   n_ti(k) = det([S_ij,-cable_seg_data{j,:}(:,k),cross(cable_seg_data{i,:}(:,k),cable_seg_data{j,:}(:,k))]);
                        %                                   n_tj(k) = det([cable_seg_data{i,:}(:,k),S_ij,cross(cable_seg_data{i,:}(:,k),cable_seg_data{j,:}(:,k))]);
                        %                                   n_t(k) = det([cable_seg_data{i,:}(:,k),-cable_seg_data{j,:}(:,k),S_ij]);
                        %
                        %                         d_n_ti(k) = d(k) - n_ti(k);
                        %                         d_n_tj(k) = d(k) - n_tj(k);
                        %                         epd_n(k) = obj.min_cable_dist^2*d(k) - n_t(k)^2;
                        %
                        
                        % this is ref to http://geomalgorithms.com/a07-_distance.html
                        
                        Sij = S_ij;
                        Si = cable_seg_data{i,:}(:,k);
                        Sj = cable_seg_data{j,:}(:,k);
                        a = Si'*Si;
                        b = Si'*Sj;
                        c = Sj'*Sj;
                        d = Si'*Sij;
                        e = Sj'*Sij;
                        t_term = Sij + ((b*e-c*d)*Si - (a*e-b*d)*Sj)/(a*c-b^2);
                        epsilon_data(k) = norm(t_term) - obj.min_cable_dist;
                        
                    end
                    
                    k_cable_equ = fit(time_steps',epsilon_data','cubicinterp');
                    t_ans = unique([t_ans;unique(fnzeros(k_cable_equ.p),'rows')']);
                    %polyfit the conditions and find the intersected time
                    %                     estimating_equ = [n_ti;n_tj;d_n_ti;d_n_tj;epd_n];
                    %                     for k = 1:size(estimating_equ,1)
                    %                         equ{equ_num} = fit(time_steps',estimating_equ(k,:)','cubicinterp');
                    %                         t_ans = unique([t_ans;unique(fnzeros(equ{equ_num}.p),'rows')']);
                    %                         equ_num = equ_num + 1;
                    %                     end
                end
            end
            t_ans(end+1) = obj.default_time_range(2);
            t_ans = sort(t_ans);
        end
        %% A function to get sample pose data
        function poses = PosesData(obj,trajectory,time_range)
            
            time_range = linspace(time_range(1),time_range(2),obj.sample_number);
            for i = 1:size(trajectory,2)
                poses(i,:) = feval(trajectory{i},time_range)';
            end
            
        end
        
    end
end

%% A function the handle the cable segment data from the poses
function cable_seg_data = CableSegmentData(model,poses)
% Update the model according to the path, return the last column data of the model.L
for i = 1:size(poses,2)
    model.update(poses(:,i), zeros(model.numDofs,1), zeros(model.numDofs,1),zeros(model.numDofs,1));
    for j = 1:model.numCables
        cable_seg_data{j,:}(:,i) = model.cableModel.cables{1,j}.segments{1,1}.segmentVector;
    end
end
end

%% A function to calculation minimum distance between two line segment
function distance = DistBetween2Segment(p1, p2, p3, p4)

u = p1 - p2;
v = p3 - p4;
w = p2 - p4;

a = dot(u,u);
b = dot(u,v);
c = dot(v,v);
d = dot(u,w);
e = dot(v,w);
D = a*c - b*b;
sD = D;
tD = D;

SMALL_NUM = 0.00000001;

% compute the line parameters of the two closest points
if (D < SMALL_NUM)  % the lines are almost parallel
    sN = 0.0;       % force using point P0 on segment S1
    sD = 1.0;       % to prevent possible division by 0.0 later
    tN = e;
    tD = c;
else                % get the closest points on the infinite lines
    sN = (b*e - c*d);
    tN = (a*e - b*d);
    if (sN < 0.0)   % sc < 0 => the s=0 edge is visible
        sN = 0.0;
        tN = e;
        tD = c;
    elseif (sN > sD)% sc > 1 => the s=1 edge is visible
        sN = sD;
        tN = e + b;
        tD = c;
    end
end

if (tN < 0.0)            % tc < 0 => the t=0 edge is visible
    tN = 0.0;
    % recompute sc for this edge
    if (-d < 0.0)
        sN = 0.0;
    elseif (-d > a)
        sN = sD;
    else
        sN = -d;
        sD = a;
    end
elseif (tN > tD)       % tc > 1 => the t=1 edge is visible
    tN = tD;
    % recompute sc for this edge
    if ((-d + b) < 0.0)
        sN = 0;
    elseif ((-d + b) > a)
        sN = sD;
    else
        sN = (-d + b);
        sD = a;
    end
end

% finally do the division to get sc and tc
if(abs(sN) < SMALL_NUM)
    sc = 0.0;
else
    sc = sN / sD;
end

if(abs(tN) < SMALL_NUM)
    tc = 0.0;
else
    tc = tN / tD;
end

% get the difference of the two closest points
dP = w + (sc * u) - (tc * v);  % = S1(sc) - S2(tc)
Pt1 = sc*u+p2;
Pt2 = tc*v+p4;
distance = norm(dP);
outV = dP;

varargout(1) = {outV};      % vector connecting the closest points
varargout(2) = {p2+sc*u};   % Closest point on object 1
varargout(3) = {p4+tc*v};   % Closest point on object 2

end


%% Analytical functions

function [Rotation_Matrix_Coefficient,theta] = RotationMatrixCoefficient(model,trajectory,time_range,tolerance)
% get start and end angles
orientation_index = find(ismember(model.bodyModel.q_dofType,'ROTATION'));
for i = 1:size(orientation_index,2)
    rad_Q(:,i) = feval(trajectory{orientation_index(i)},time_range);
      if any(rad_Q(:,i)>= pi)
        m = ['Orientation over 180',char(176), ' is not appicable for this method'];
        CASPR_log.Error(m)
    end
end

% q_s = quatnormalize(angle2quat(rad_Q(1,1),rad_Q(1,2),rad_Q(1,3),'XYZ'));
% q_e = quatnormalize(angle2quat(rad_Q(2,1),rad_Q(2,2),rad_Q(2,3),'XYZ'));

q_s = angle2quat(rad_Q(1,1),rad_Q(1,2),rad_Q(1,3),'XYZ');
q_e = angle2quat(rad_Q(2,1),rad_Q(2,2),rad_Q(2,3),'XYZ');
theta = acos(q_s*q_e');

%% sampling the matrix by 5 times since the max degree is 4
% tau = linspace(time_range(1),time_range(2),5);
k = tan(theta/2)/time_range(2);
t = linspace(time_range(1),time_range(2),5);
if theta~=0
    tau = atan(k*t)*2/theta;
else
    tau = zeros(1,size(t,2));
end
for i = 1:length(tau)
    q_t = quatinterp(q_s,q_e,tau(i),'slerp');
    sample_R = quat2rotm(q_t)';
    denominator_R(i) = polyval([1 0 2 0 1],k*t(i));    
    numerator_R{i} = denominator_R(i) * sample_R;
end
for i = 1:9 % number of elements of rotation matrix
    for j = 1:5 % number of samples
        sample_element(i,j) = numerator_R{j}(i);
        M(j,:) = [t(j)^4 t(j)^3 t(j)^2 t(j) 1];
    end
    Rotation_Matrix_Coefficient(i,:) = M\sample_element(i,:)';
end
Rotation_Matrix_Coefficient = round(Rotation_Matrix_Coefficient,tolerance);
Rotation_Matrix_Coefficient(isnan(Rotation_Matrix_Coefficient))=0;
end

function Translation_Trajectory_Coefficient = TranslationTrajectoryCoefficient(model,trajectory,time_range,maximum_trajectory_degree,theta)
translation_index = find(ismember(model.bodyModel.q_dofType,'TRANSLATION'));
% polynomials degree should be known
t = linspace(time_range(1),time_range(2),maximum_trajectory_degree+1);
% T = linspace(0,tan(theta/2),maximum_trajectory_degree+1);
for i = 1:size(translation_index,2)
    data_in_tau = feval(trajectory{translation_index(i)},t);
    Translation_Trajectory_Coefficient(i,:) = polyfit(t',data_in_tau,maximum_trajectory_degree);
end
Translation_Trajectory_Coefficient = round(Translation_Trajectory_Coefficient,9);
end

function output = SumCoeff(P)
output = P{1,:};
for i = 2:size(P,1)
    output =  [zeros(1, size(P{i,:},2)-size(output,2)) output] + [zeros(1, size(output,2)-size(P{i,:},2)) P{i,:}];
end
% output = [zeros(1, size(P_1,2)-size(P_2,2)) P_2] + [zeros(1, size(P_2,2)-size(P_1,2)) P_1];

end

function pose = InstancePose(model,trajectory,time,theta,time_range)
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
            tau = atan(k*time)*2/theta;
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

