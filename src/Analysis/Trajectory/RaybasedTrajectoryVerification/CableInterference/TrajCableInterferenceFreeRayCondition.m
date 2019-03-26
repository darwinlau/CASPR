% Class to compute whether a a trajectory satisfying the interference 
% free workspace (IFW) between cables
%
% Author        : Zeqing ZHANG
% Created       : 2019
% Description   : 

classdef TrajCableInterferenceFreeRayCondition < TrajectoryRayConditionBase
    properties (SetAccess = protected, GetAccess = protected)
        min_ray_percentage          % The minimum percentage of the ray at which it is included
        safe_dis                    % Safe distance between two cables
    end
    
    methods
        % Constructor for interference free worksapce
        function w= TrajCableInterferenceFreeRayCondition(min_ray_percent,safe_distance)
            w.min_ray_percentage = min_ray_percent;
            w.safe_dis = safe_distance;
        end
            
        % Evaluate the interference free intervals 
        function P_IFC =  evaluateFunction(obj,model,traj)
            % quaternion points
            q0_given = traj.quaternionPts(1,:);
            q1_given = traj.quaternionPts(2,:);
            cosAgl = sum(q0_given .* q1_given);
            theta = acos(cosAgl);
            
            % cal. the coeff. of rotation matrix
            A = cal_coeff_numer_element_numerical(obj,q0_given, q1_given);
            coeff_denom = [1 0 2 0 1];
            collision2Cables = [];
            
            % coefficients of translation trajectory w.r.t T
            C_wrt_T = traj.translationTrajectory;

            %%%%%%%%% --- END - interference parts of cables i and j --- %%%%%%%%%
            for i = 1:model.numCables
                Erie = model.cableModel.cables{1, i}.attachments{1, 2}.r_GA';
                ris = model.cableModel.cables{1, i}.attachments{1, 1}.r_OA';
                for j = i+1:model.numCables
                    % relative position in local frames [row vector]
                    % Erje = Er_e(j,:);
                    Erje = model.cableModel.cables{1, j}.attachments{1, 2}.r_GA';
                    % absolute postion in the base frame [row vector]
                    % rjs = r_s(j,:);
                    rjs = model.cableModel.cables{1, j}.attachments{1, 1}.r_OA';
                    %% cal. \tilde(li(T)), \tilde(lj(T)) = lj(T)*(1+T^2)^2
                    %%% lix~ liy~ liz~(T)
                    lix = AddPolyCoeff_2P(obj,conv(C_wrt_T(1,:),coeff_denom), (A(1, :)*Erie(1) + A(2, :)*Erie(2) + A(3, :)*Erie(3) - ris(1)*coeff_denom));
                    liy = AddPolyCoeff_2P(obj,conv(C_wrt_T(2,:),coeff_denom), (A(4, :)*Erie(1) + A(5, :)*Erie(2) + A(6, :)*Erie(3) - ris(2)*coeff_denom));
                    liz = AddPolyCoeff_2P(obj,conv(C_wrt_T(3,:),coeff_denom), (A(7, :)*Erie(1) + A(8, :)*Erie(2) + A(9, :)*Erie(3) - ris(3)*coeff_denom));

                    %%% ljx~ ljy~ ljz~(T)
                    ljx = AddPolyCoeff_2P(obj,conv(C_wrt_T(1,:),coeff_denom), (A(1, :)*Erje(1) + A(2, :)*Erje(2) + A(3, :)*Erje(3) - rjs(1)*coeff_denom));
                    ljy = AddPolyCoeff_2P(obj,conv(C_wrt_T(2,:),coeff_denom), (A(4, :)*Erje(1) + A(5, :)*Erje(2) + A(6, :)*Erje(3) - rjs(2)*coeff_denom));
                    ljz = AddPolyCoeff_2P(obj,conv(C_wrt_T(3,:),coeff_denom), (A(7, :)*Erje(1) + A(8, :)*Erje(2) + A(9, :)*Erje(3) - rjs(3)*coeff_denom));

                    %%%
                    SegEndpt{1,2} = [AddPolyCoeff_2P(obj,conv(C_wrt_T(1,:),coeff_denom), (A(1, :)*Erie(1) + A(2, :)*Erie(2) + A(3, :)*Erie(3)));
                        AddPolyCoeff_2P(obj,conv(C_wrt_T(2,:),coeff_denom), (A(4, :)*Erie(1) + A(5, :)*Erie(2) + A(6, :)*Erie(3)));
                        AddPolyCoeff_2P(obj,conv(C_wrt_T(3,:),coeff_denom), (A(7, :)*Erie(1) + A(8, :)*Erie(2) + A(9, :)*Erie(3)))];
                    SegEndpt{1,1} = ris'*coeff_denom;
                    SegEndpt{2,2} = [AddPolyCoeff_2P(obj,conv(C_wrt_T(1,:),coeff_denom), (A(1, :)*Erje(1) + A(2, :)*Erje(2) + A(3, :)*Erje(3)));
                        AddPolyCoeff_2P(obj,conv(C_wrt_T(2,:),coeff_denom), (A(4, :)*Erje(1) + A(5, :)*Erje(2) + A(6, :)*Erje(3)));
                        AddPolyCoeff_2P(obj,conv(C_wrt_T(3,:),coeff_denom), (A(7, :)*Erje(1) + A(8, :)*Erje(2) + A(9, :)*Erje(3)))];
                    SegEndpt{2,1} = rjs'*coeff_denom;
                    L{1} = AddPolyCoeff_2P(obj,SegEndpt{1,2}, -SegEndpt{1,1});
                    L{2} = AddPolyCoeff_2P(obj,SegEndpt{2,2}, -SegEndpt{2,1});
                    %% cal. nti/ntj/nhatcp_2/den(T) (??? Deg Polynomial of T)
                    W = rjs - ris;
                    Wx = W(1);
                    Wy = W(2);
                    Wz = W(3);

                    %%% nti
                    nti = conv((Wy*liy+Wz*liz),conv(ljx,ljx))+conv((conv((-Wx*liy-Wy*lix),ljy)...
                        -conv(ljz,(Wx*liz+Wz*lix))),ljx)+conv((Wx*lix+Wz*liz),conv(ljy,ljy))...
                        -conv(ljz,conv((Wy*liz+Wz*liy),ljy))+conv(conv(ljz,ljz),(Wx*lix+Wy*liy));
                    % coeff_nti = CoeffUniPoly(nti, coeff_denom);
                    tilde_nti = nti;

                    %%% ntj
                    ntj = conv((-Wy*ljy-Wz*ljz),conv(lix,lix))+conv((conv((Wx*ljy+Wy*ljx),liy)...
                        +conv(liz,(Wx*ljz+Wz*ljx))),lix)+conv((-Wx*ljx-Wz*ljz),conv(liy,liy))...
                        +conv(liz,conv((Wy*ljz+Wz*ljy),liy))-conv(conv(liz,liz),(Wx*ljx+Wy*ljy));
                    % coeff_ntj = CoeffUniPoly(ntj, coeff_denom);
                    tilde_ntj = ntj;

                    %%% nhatcp_2
                    nhatcp_temp = (-conv(lix,ljy)+conv(liy,ljx))*Wz-Wx*conv(liy,ljz)...
                        +conv(liz,(Wx*ljy-Wy*ljx))+Wy*conv(lix,ljz);
                    nhatcp_2 = conv(nhatcp_temp,nhatcp_temp);
                    % coeff_nhatcp_2 = CoeffUniPoly(nhatcp_2, coeff_denom);
                    tilde_nt_square = nhatcp_2;

                    %%% den
                    den = conv((conv(ljy,ljy)+conv(ljz,ljz)),conv(lix,lix))...
                        -2*conv(ljx,conv((conv(liy,ljy)+conv(liz,ljz)),lix))...
                        +conv((conv(ljx,ljx)+conv(ljz,ljz)),conv(liy,liy))...
                        -2*conv(conv(liy,liz),conv(ljy,ljz))+conv(conv(liz,liz),(conv(ljx,ljx)...
                        +conv(ljy,ljy)));
                    % coeff_den = CoeffUniPoly(den, coeff_denom);
                    tilde_d = den;

                    %% 5 polynomials of sufficient & necessary condtions of collisions
                    CoUniPol{1}  = conv(coeff_denom,tilde_nti);
                    CoUniPol{2}  = conv(coeff_denom,tilde_ntj);
                    CoUniPol{3}  = AddPolyCoeff_2P(obj,tilde_d, -CoUniPol{1});
                    CoUniPol{4}  = AddPolyCoeff_2P(obj,tilde_d, -CoUniPol{2});
                    CoUniPol{5}  = (obj.safe_dis^2)*tilde_d-tilde_nt_square;

                    coeff_nti = CoUniPol{1};
                    coeff_ntj = CoUniPol{2};
                    dnti = CoUniPol{3};
                    dntj = CoUniPol{4};

                    %% solve conditions
                    inequal = '>=';
                    % intv_CP = RealRoots(CoUniPol, curflxvar, bcurflxvar, inequal);
                    intv_CP = RealRootsIntersectSet(obj, theta, CoUniPol, inequal);
                    clear CoUniPol;

                    % ---------(MinDis is the distance between one end-pt and the other segment)-start-----------------*
                    % ti<0 & 0<tj & tj<1 & invt(pt,seg)
                    % ti<0 & 0<tj & tj<1
                    CoUniPol{1} = -coeff_nti;
                    CoUniPol{2} = coeff_ntj;
                    % CoUniPol{3} = coeff_den - coeff_ntj;
                    CoUniPol{3} = dntj;

                    % --- v2.0 ---
                    % invt(pt,seg): pt=Ai, seg = lj
                    pt = SegEndpt{1,1};
                    segendpt = SegEndpt(2,:);
                    seg = L{2};
                    intv_Ailj = IntersectInvPtSeg(obj, obj.safe_dis, theta, coeff_denom, CoUniPol, inequal, pt, segendpt, seg);
                    clear CoUniPol;

                    % 0<ti & ti<1 & tj<0 & invt(pt,seg)
                    % 0<ti & ti<1 & tj<0
                    CoUniPol{1} = coeff_nti;
                    % CoUniPol{2} = coeff_den - coeff_nti;
                    CoUniPol{2} = dnti;
                    CoUniPol{3} = -coeff_ntj;
                    % --- v2.0 ---
                    % invt(pt,seg): pt=Aj, seg = li
                    pt = SegEndpt{2,1};
                    segendpt = SegEndpt(1,:);
                    seg = L{1};
                    intv_Ajli = IntersectInvPtSeg(obj, obj.safe_dis, theta, coeff_denom, CoUniPol, inequal, pt, segendpt, seg);
                    clear CoUniPol;

                    % 0<ti & ti<1 & tj>1 & invt(pt,seg)
                    % 0<ti & ti<1 & tj>1
                    CoUniPol{1} = coeff_nti;
                    % CoUniPol{2} = coeff_den - coeff_nti;
                    CoUniPol{2} = dnti;
                    % CoUniPol{3} = -(coeff_den - coeff_ntj);
                    CoUniPol{3} = -dntj;
                    % --- v2.0 ---
                    % invt(pt,seg): pt=Bj, seg = li
                    pt = SegEndpt{2,2};
                    segendpt = SegEndpt(1,:);
                    seg = L{1};
                    intv_Bjli = IntersectInvPtSeg(obj, obj.safe_dis, theta, coeff_denom, CoUniPol, inequal, pt, segendpt, seg);
                    clear CoUniPol;


                    % 1<ti & 0<tj & tj<1 & invt(pt,seg)
                    % 1<ti & 0<tj & tj<1
                    % CoUniPol{1} = -(coeff_den - coeff_nti);
                    CoUniPol{1} = -dnti;
                    CoUniPol{2} = coeff_ntj;
                    % CoUniPol{3} = coeff_den - coeff_ntj;
                    CoUniPol{3} = dntj;
                    % --- v2.0 ---
                    % invt(pt,seg): pt=Bi, seg = lj
                    pt = SegEndpt{1,2};
                    segendpt = SegEndpt(2,:);
                    seg = L{2};
                    intv_Bilj = IntersectInvPtSeg(obj, obj.safe_dis, theta, coeff_denom, CoUniPol, inequal, pt, segendpt, seg);
                    clear CoUniPol;
                    % ---------(MinDis is the distance between one end-pt and the other segment)-end-------------------*

                    intv = [intv_CP; intv_Ailj; intv_Ajli; intv_Bjli; intv_Bilj];
                    if ~isempty(intv)
                        Left = intv(:, 1);
                        R = intv(:, 2);
                        [L_intv, R_intv] = Or_interval(obj,Left, R); % L_intv \in row vector form
                        % modify to open set (delete the isolated point)
                        tmp = round(L_intv - R_intv, 10);
                        L_intv(tmp==0) =[];
                        R_intv(tmp==0) =[];
                        out_temp = [L_intv', R_intv']; % n*2 matrix
                        % T->t_Q
                        out_temp = 2*atan(out_temp)/theta;
                    else
                        out_temp = [];
                    end
                    intvInterf = out_temp; % (t:\in [0,1])

                    % Union set of collsion intervals
                    collision2Cables = [collision2Cables; intvInterf];
                    %%% check codes %%%
                    % %         test_t = .8;
                    % %         [mindis, position_ij] =  checkFun(theta,q0,q1,test_t,ris,rjs,Erie,Erje,C_wrt_T)
                end
            end
            %
            %%%%%%%%% --- END - interference parts of cables i and j --- %%%%%%%%%

            %%%%%%
            %
            %% Union sets of collision rays for all cables
            if ~isempty(collision2Cables)
                Left = collision2Cables(:, 1);
                R = collision2Cables(:, 2);
                [L_intv, R_intv] = Or_interval(obj,Left, R); % L_intv \in row vector form
                % modify to open set (delete the isolated point)
                tmp = round(L_intv - R_intv, 10);
                L_intv(tmp==0) =[];
                R_intv(tmp==0) =[];
                out_temp = [L_intv', R_intv']; % n*2 matrix
            else
                out_temp = [];
            end
            UnionCollisionRays = out_temp;

            %% IFW rays for all cables: Complement sets:
            minVal = 0;
            maxVal = 1;
            if ~isempty(UnionCollisionRays)
                [L_comp, R_comp] = Comp_interval(obj,UnionCollisionRays(:,1), UnionCollisionRays(:,2));
                if ~isempty(L_comp)
                    % results from fun'or_and_or' is closed intervals
                    [resultL, resultR] = or_and_or(obj,minVal, maxVal, L_comp, R_comp);
                    tmp = round(resultL - resultR, 10);
                    resultL(tmp==0) =[];
                    resultR(tmp==0) =[];
                    intvifw_temp = [resultL', resultR'];
                else
                    intvifw_temp = [];
                end
            else
                intvifw_temp = [minVal, maxVal];
            end
            P_IFC = intvifw_temp;
        end
        
        %% Support Functions
        function A = cal_coeff_numer_element_numerical(obj, q0_given, q1_given)
            % normalized to unit quaternions
            q0 = quatnormalize(q0_given);
            q1 = quatnormalize(q1_given);
            cosAgl = sum(q0 .* q1);
            % the angle between two quaternions
            Agl = acos(cosAgl);
            %     sinAgl = sin(Agl);
            
            % take 5 unique samples
            t = zeros(1, 5);
            T = zeros(1, 5);
            quat_sample = cell(1, 5);
            rotat_sample = cell(1, 5);
            denom_val = zeros(1, 5);
            numer_val = cell(1, 5);
            for i = 1:5
                t(i) = 0.25*(i-1);
                T(i) = tan(0.5*(Agl*t(i)));
                % quaternion by slerp
                quat_sample{i} = round(quatinterp(q0,q1,t(i),'slerp'), 10);
                % convert to the rotation matrix
                rotat_sample{i} = quat2rotm_1D(obj,quat_sample{i});
                % calculate the common denominator of the rotation matrix
                %        [denom_val(i), ~] = cal_common_denom(q0, q1, T(i));
                [denom_val(i), ~] = cal_common_denom_v2(obj, q0, q1, T(i));
                
                % the numerator of the rotation matrix
                numer_val{i} = rotat_sample{i}*denom_val(i);
            end
            
            % cal. coefficients of each element in rotation matrix
            % store nr values
            B = zeros(5,1);
            % store T^4 .. 1
            M = zeros(5, 5);
            % store coefficients \in R^{9*5}
            A = zeros(9, 5);
            for cnt = 1:9
                for i = 1:5
                    mat_temp = numer_val{i}';
                    B(i, :) = mat_temp(cnt);
                    M(i, :) = [T(i)^4 T(i)^3 T(i)^2 T(i) 1];
                end
                A(cnt, :) = round((M\B)',6);
                clear B M
            end
        end
        
        function rotat = quat2rotm_1D(~,q)
            % normalize the input quaternion
            n = quatnormalize(q);
            s = n(1);
            x = n(2);
            y = n(3);
            z = n(4);
            % cal. the rotation matrix
            rotat = [1 - 2*(y.^2 + z.^2),   2*(x.*y - s.*z),   2*(x.*z + s.*y);...
                2*(x.*y + s.*z), 1 - 2*(x.^2 + z.^2),   2*(y.*z - s.*x);...
                2*(x.*z - s.*y),   2*(y.*z + s.*x), 1 - 2*(x.^2 + y.^2)];
            rotat = round(rotat, 12);
        end
        
        function [den_val, coeff_high_precision] = cal_common_denom_v2(~, q0_given, q1_given, T)
            q0 = quatnormalize(q0_given);
            q1 = quatnormalize(q1_given);
            cosAgl = sum(q0 .* q1);
            coeff_high_precision = [1 0 2 0 1];
            % cal. the value of denominator
            den_val = polyval(coeff_high_precision, T);

        end
            
        function coeff_sum = AddPolyCoeff_2P(~,x1, x2)
            x1_order = length(x1);
            x2_order = length(x2);
            if x1_order > x2_order
                max_order = size(x1);
            else
                max_order = size(x2);
            end
            new_x1 = padarray(x1,max_order-size(x1),0,'pre');
            new_x2 = padarray(x2,max_order-size(x2),0,'pre');
            coeff_sum = new_x1 + new_x2;
        end
        
        function Real_invt = RealRootsIntersectSet(obj, theta, CoUniPol, inequal)
            %%% range of T
            %     bcurflxvar = workspace_ray.free_variable_range;
            min = 0;
            max = tan(0.5*theta);
            
            % solve for roots
            real_root_pts = [];
            numIneq = length(CoUniPol); % num of inequalities
            for i = 1:numIneq
                %     root_pts_temp = round(roots(CoUniPol{i}), 12); % column vector
                root_pts_temp = roots(CoUniPol{i}); % column vector
                % leave real soln, eliminate complex soln of EACH polynomials
                real_root_pts_temp = root_pts_temp(imag(root_pts_temp)==0);
                if ~isempty(real_root_pts_temp) % have real solns
                    real_root_pts = unique([real_root_pts; real_root_pts_temp]);  % (T)
                else % no real solns
                    midpts = (min+max)*0.5;
                    polyval_temp = round(polyval(CoUniPol{i}, midpts), 10);
                    if strcmp(inequal, '<=')
                        if polyval_temp <= 0
                            % disp(strcat('The soln of ', num2str(i), '-th inequality is [min max]'));
                        else
                            Real_invt = [];return;
                        end
                    elseif strcmp(inequal, '>=')
                        if polyval_temp >= 0
                            % disp(strcat('The soln of ', num2str(i), '-th inequality is [min max]'));
                        else
                            Real_invt = [];return;
                        end
                        % -- testing for open interval
                    elseif strcmp(inequal, '<')
                        if polyval_temp < 0
                            % disp(strcat('The soln of ', num2str(i), '-th inequality is [min max]'));
                        else
                            Real_invt = [];return;
                        end
                    elseif strcmp(inequal, '>')
                        if polyval_temp > 0
                            % disp(strcat('The soln of ', num2str(i), '-th inequality is [min max]'));
                        else
                            Real_invt = [];return;
                        end
                    else
                        CASPR_log.Print('Invalid Sign or solution is zero',CASPRLogLevel.ERROR);
                    end
                end
            end
            
            if isempty(real_root_pts) % all inequality solns are [-inf inf]
                Real_invt = [min max];
            else
                if min>= max
                    CASPR_log.Print('Invalid limits of flexible variable',CASPRLogLevel.ERROR);
                end
                Rel_root_pts = real_root_pts((real_root_pts > min) & (real_root_pts < max));
                
                % judge the positive/negative intervals
                % if ~isempty(Rel_root_pts) % optimize code for empty result case
                Rel_root_pts = sort(Rel_root_pts);
                
                % S2 add 2 boundary pts
                % check_pts = [min; Rel_root_pts; max];
                
                % S3 pick up the MidPts
                low = [min; Rel_root_pts];
                up = [Rel_root_pts; max];
                MidPts = (low+up)*0.5; % vector
                
                % S4 calculate the polynomial valus according to the MidPts
                PolyVal = zeros(length(MidPts), numIneq);
                for i = 1:numIneq
                    polyval_temp = round(polyval(CoUniPol{i}, MidPts), 10); % same as the form of MidPts
                    % polyval_temp2 = polyval(CoUniPol{i}, check_pts)
                    %     if strcmp(inequal, '<')
                    if strcmp(inequal, '<=')
                        %         polyval_temp_ind = (polyval_temp<0); % 1 refers to negative ploy-value
                        polyval_temp_ind = (polyval_temp<=0);
                        %     elseif strcmp(inequal, '>')
                    elseif strcmp(inequal, '>=')
                        %         polyval_temp_ind = (polyval_temp>0); % 1 refers to positive ploy-value
                        polyval_temp_ind = (polyval_temp>=0);
                        % ---- testing for open interval
                    elseif strcmp(inequal, '<')
                        polyval_temp_ind = (polyval_temp<0);
                    elseif strcmp(inequal, '>')
                        polyval_temp_ind = (polyval_temp>0);
                        % -----
                    else
                        CASPR_log.Print('Invaid inequality',CASPRLogLevel.ERROR);
                    end
                    PolyVal(:, i) = polyval_temp_ind;
                end
                
                % S5 collect the neagtive(-) intervals (i.e. IFW intervals) and store in
                %    the form of n*2 matrix
                logic = all(PolyVal, 2); % INTERSECTION set of all inequalities
                k = find(logic > 0);
                if isempty(k)
                    out_temp = [];
                else
                    Left = low(k); % column vector
                    Right = up(k); % column vector
                    [L_or, R_or] = Or_interval(obj, Left, Right); % avoid adjacent intervals
                    out_temp = [L_or', R_or']; % n*2 matrix
                end
                Real_invt = out_temp; % n*2 matrix or null
            end
        end
        
        function intvPtSeg = IntersectInvPtSeg(obj, safe_dis, theta, denom, CoUniPol, inequal, pt, segendpt, seg)
            intv_temp1 = RealRootsIntersectSet(obj, theta, CoUniPol, inequal);
            if ~isempty(intv_temp1)
                intv_temp2 =  MinDisPtSeg(obj, safe_dis, theta, denom, pt, segendpt, seg);
                % intersetion
                if ~isempty(intv_temp2)
                    [resultL, resultR] = or_and_or(obj, intv_temp1(:,1), intv_temp1(:,2), intv_temp2(:,1), intv_temp2(:,2));
                    intvPtSeg = [resultL', resultR'];
                else
                    intvPtSeg =[];
                end
            else
                intvPtSeg = [];
            end
        end
        
        function intvPtSeg =  MinDisPtSeg(obj, safe_dis, theta, denom, pt, segendpt, seg)
            % bcurflxvar = [obj.model.bodyModel.q_min(flxvar); obj.model.bodyModel.q_max(flxvar)];
            %% specify the transfer operator
            denomi = denom;
            if size(pt,2) == 1 % if pt is a constant column vector
                pt = pt* transf; % pt \in 3*2(translation) or 3*3(rotation) matrix
            end
            
            %     APt = pt - segendpt{1};
            %     BPt = pt - segendpt{2};
            APt = AddPolyCoeff_2P(obj, pt, -segendpt{1});
            BPt = AddPolyCoeff_2P(obj, pt, -segendpt{2});
            
            %% potential MinDis < diameter
            % norm(APt) < dia
            temp1 = SquarePoly(obj, APt);
            % temp2 = -conv(conv(denomi,denomi), diameter^2);
            temp2 = -(safe_dis)^2 * (conv(denomi,denomi));
            mindisPoly{1} = AddPolyCoeff_2P(obj, temp1, temp2);
            % cross(APt, l)/norm(l) < dia
            temp1 = CrossProductPoly(obj, APt,seg);
            temp12 = SquarePoly(obj, temp1);
            temp22 = -(safe_dis)^2 * conv(conv(denomi, denomi),SquarePoly(obj, seg)); % <<<<<<<<<<<<
            mindisPoly{2} = AddPolyCoeff_2P(obj, temp12, temp22);
            % norm(BPt) < dia
            temp1 = SquarePoly(obj, BPt);
            % temp2 = -conv(conv(denomi,denomi), diameter^2);
            temp2 = -(safe_dis)^2 * (conv(denomi,denomi));
            mindisPoly{3} = AddPolyCoeff_2P(obj, temp1, temp2);
            
            
            %% form the condition polynomial
            % dot(APt, l)
            tempdot = conv(APt(1,:), seg(1,:))+conv(APt(2,:), seg(2,:))+conv(APt(3,:), seg(3,:));
            % dot(APt, l) = 0
            CondPoly{1} = tempdot;
            % dot(APt, l) = norm(l)^2
            temp_norm2 = -SquarePoly(obj, seg);
            % CondPoly{2} = tempdot - temp_norm2;
            CondPoly{2} = AddPolyCoeff_2P(obj, tempdot, temp_norm2);
            
            
            %% interval calculation
            % MinDisPtSeg is a piece-wise function with 3 cases
            inequal = '<';
            % inequal = '<=';
            % i.e., Union_(i,j)(mindisPoly{i} & CondPoly{j})
            % intv1 (case1)
            CoUniPol_temp{1} = mindisPoly{1};
            CoUniPol_temp{2} = CondPoly{1};
            intv1 = RealRootsIntersectSet(obj, theta, CoUniPol_temp, inequal);
            clear CoUniPol_temp;
            
            % intv2 (case2)
            CoUniPol_temp{1} = mindisPoly{2};
            CoUniPol_temp{2} = -CondPoly{1};
            CoUniPol_temp{3} = CondPoly{2};
            intv2 = RealRootsIntersectSet(obj, theta, CoUniPol_temp, inequal);
            clear CoUniPol_temp;
            
            % intv3 (case3)
            CoUniPol_temp{1} = mindisPoly{3};
            CoUniPol_temp{2} = -CondPoly{2};
            intv3 = RealRootsIntersectSet(obj, theta, CoUniPol_temp, inequal);
            clear CoUniPol_temp;
            
            % union set of (intv1, intv2, intv3)
            intv = [intv1; intv2; intv3];
            if ~isempty(intv)
                L_temp = intv(:,1);
                R_temp = intv(:,2);
                [L_intv, R_intv] = Or_interval(obj, L_temp, R_temp); % L_intv \in row vector form
                intv_temp = [L_intv', R_intv'];
            else
                intv_temp = [];
            end
            
            % intervals of MinDis from pt v.s. seg
            intvPtSeg = intv_temp; % \in n*2 or null
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
                        % Since L(i+1) is non-null, then the input of Or_interval_2 fn. below
                        % cannot be one null set, thus the outputs, L_2inter and
                        % R_2inter, are non-null due to Note-2-2), which inplies
                        % L_store(t) and R_store(t) are non-null
                        [L_2inter, R_2inter] = Or_interval_2(obj, [L_store(t) L(i+1)], [R_store(t) R(i+1)]); % result is one interval
                        L_store(t) = L_2inter; R_store(t) = R_2inter;
                    end
                end
                L_or = L_store;
                R_or = R_store;
            end
        end

        function [L_or, R_or] = Or_interval_2(~,L, R)
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
        
        function output = SquarePoly(~, input)
            % output = input^2
            % input \in 3*n matrix
            x = input(1,:);
            y = input(2,:);
            z = input(3,:);
            output = conv(x,x)+conv(y,y)+conv(z,z);
        end
        
        function y = CrossProductPoly(~,x1, x2)
            % input should be of column 'vector' form
            lix = x1(1, :);
            liy = x1(2, :);
            liz = x1(3, :);
            ljx = x2(1, :);
            ljy = x2(2, :);
            ljz = x2(3, :);
            y_x = conv(liy, ljz)-conv(liz, ljy);
            y_y = conv(liz, ljx)-conv(lix, ljz);
            y_z = conv(lix, ljy)-conv(liy, ljx);
            y = [y_x; y_y; y_z];
        end
        
        
        
    end
    
end

