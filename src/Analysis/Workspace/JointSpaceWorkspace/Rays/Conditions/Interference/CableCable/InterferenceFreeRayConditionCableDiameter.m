frti8u% Class to compute whether a pose (dynamics) is within the interference 
% free workspace (IFW)
%
% Author        : Benji
% Created       : 2017
% Description   : 

classdef InterferenceFreeRayConditionCableDiameter < WorkspaceRayConditionBase
    properties (SetAccess = protected, GetAccess = protected)
        min_ray_percentage          % The minimum percentage of the ray at which it is included
        safe_dis                    % Safe distance between two cables
    end
    
    methods
        % Constructor for interference free worksapce
        function w= InterferenceFreeRayConditionCableDiameter(min_ray_percent,~)
            w.min_ray_percentage = min_ray_percent;
        end
            
        % Evaluate the interference free intervals 
        function intvifwmc =  evaluateFunction(obj,model,workspace_ray)
            % generate the Lm (m=1:numCables) before reviewing each pair of
            % cables
            SegEndpt = CoeffLm(obj, model, workspace_ray);
            %% select 2 cables to get non-interset intervals
            flag   = 0; % flag for non-null 
            ncable = model.numCables;
            L_A = -inf;
            R_A = inf;
            % in the form of lower triangular matrix
            for ithcable = 1: ncable % i-th cable
                for jthcable = 1: (ithcable - 1) % j-th cable
                    % -- new --
                    twocables = [ithcable; jthcable];
                    SegEndpt_ij = {SegEndpt{ithcable,:}; SegEndpt{jthcable,:}};
                    intvifw2c = IFW_HyB_2C_safebuf(obj, model, workspace_ray, SegEndpt_ij, twocables);
                    
                    % update non-intersect intervals 
                    if ~isempty(intvifw2c)
                        L_B = intvifw2c(:, 1);
                        R_B = intvifw2c(:, 2);
                        if ~isempty(L_A)
                            [L_A, R_A] = or_and_or(obj, L_A, R_A, L_B, R_B);
                        else
                            flag = 1;
                            break;
                        end
                    else
                        % intvifwmc_temp = [];
                        flag = 1;
                        break;
                    end
                end
                if flag == 1
                    break;
                end
            end
            if flag == 1
                intvifwmc_temp = [];
            else
                intvifwmc_temp = [L_A', R_A'];                
            end
            %% generate non-intersect intervals of multicables
            % avoid isolated pts occur i.e., L_A == R_A
%                 if round(intvifwmc_temp(:,1)-intvifwmc_temp(:,2),7)==0
%                     intvifwmc_temp(round(intvifwmc_temp(:,1)-intvifwmc_temp(:,2),7)==0,:)=[];
%                 end
            if ~isempty(intvifwmc_temp)
                if abs(intvifwmc_temp(:,1)-intvifwmc_temp(:,2)) <= obj.min_ray_percentage
                    intvifwmc_temp(abs(intvifwmc_temp(:,1)-intvifwmc_temp(:,2))<=obj.min_ray_percentage,:)=[];
                end
            end
            intvifwmc = intvifwmc_temp;
        end
        
        %% --v2.0 to calcualte Lm (m=1:numSeg)
        function SegEndpt = CoeffLm(~, model, workspace_ray)
            curflxvar = workspace_ray.freeVariableIndex;
            magconvar = workspace_ray.fixedVariables';
            % ---- the bound of curflxvar ----------
            curValmin = workspace_ray.freeVariableRange(1);
            curValmax = workspace_ray.freeVariableRange(2);
            % just for cables collisions
            nseg = model.numCables;
            
            SegEndpt = cell(nseg,2);
            curtypevar = model.bodyModel.q_dofType;            
            if curtypevar(curflxvar)==DoFType.TRANSLATION 
                % -- q1, q2 ----
                curVal1 = curValmin+(curValmax-curValmin)*0.3; 
                curVal2 = curValmin+(curValmax-curValmin)*0.6;
                q_temp = zeros(model.numDofs,2);
                q_temp(:,1) = [magconvar(1:curflxvar-1) curVal1 magconvar(curflxvar:end)]';
                q_temp(:,2) = [magconvar(1:curflxvar-1) curVal2 magconvar(curflxvar:end)]';
                M = [curVal1 1; curVal2 1];
                trsf = ones(1,2);
            else % -- T1, T2, T3 -- 
                angle2 = (curValmin+curValmax)*0.5; T2 = tan(0.5*angle2);
                angle1 = (curValmin+angle2)*0.5; T1 = tan(0.5*angle1);
                angle3 = (angle2+curValmax)*0.5; T3 = tan(0.5*angle3);
                q_temp = zeros(model.numDofs,3);
                q_temp(:,1) = [magconvar(1:curflxvar-1) angle1 magconvar(curflxvar:end)]';
                q_temp(:,2) = [magconvar(1:curflxvar-1) angle2 magconvar(curflxvar:end)]';
                q_temp(:,3) = [magconvar(1:curflxvar-1) angle3 magconvar(curflxvar:end)]';
                M = [T1^2 T1 1; T2^2 T2 1; T3^2 T3 1];
                trsf1 = 1+(T1)^2; trsf2 = 1+(T2)^2; trsf3 = 1+(T3)^2;
                trsf = [trsf1 trsf2 trsf3];
            end
            % -- li & lj ( _1,_2,_3 )
            q_dot = zeros(model.numDofs,1);
            q_ddot = zeros(model.numDofs,1);
            w_ext = zeros(model.numDofs,1);
            Ep_iax = zeros(size(q_temp,2),1); Ep_iay = zeros(size(q_temp,2),1); Ep_iaz = zeros(size(q_temp,2),1);
            Ep_ibx = zeros(size(q_temp,2),1); Ep_iby = zeros(size(q_temp,2),1); Ep_ibz = zeros(size(q_temp,2),1);
            for rmflg = 1:nseg
                for cnt = 1:size(q_temp,2)
                    model.update(q_temp(:,cnt), q_dot, q_ddot,w_ext);
                    endpt_ia = model.cableModel.cables{rmflg}.attachments{1}.r_OA;
                    endpt_ib = model.cableModel.cables{rmflg}.attachments{2}.r_OA;
                    Ep_iax(cnt) = trsf(cnt)*endpt_ia(1); Ep_iay(cnt) = trsf(cnt)*endpt_ia(2); Ep_iaz(cnt) = trsf(cnt)*endpt_ia(3);
                    Ep_ibx(cnt) = trsf(cnt)*endpt_ib(1); Ep_iby(cnt) = trsf(cnt)*endpt_ib(2); Ep_ibz(cnt) = trsf(cnt)*endpt_ib(3);
                end

                % > Liendpt \in 1*2 cell
                % >> Liendpt{1} \in 3*3 matrix; each row refers to x or y or z
                row1 = (M\Ep_iax)'; row2 = (M\Ep_iay)'; row3 = (M\Ep_iaz)';
                Liendpt{1} = round([row1; row2; row3], 12);
                % >> Liendpt{2} \in 3*3 matrix
                row1 = (M\Ep_ibx)'; row2 = (M\Ep_iby)'; row3 = (M\Ep_ibz)';
                Liendpt{2} = round([row1; row2; row3], 12);
                % ref  SegEndpt = [Liendpt; Ljendpt]; Liendpt \in 1*2 cell
                SegEndpt(rmflg,:) = Liendpt;
            end
        end
        
        function intvifw2c = IFW_HyB_2C_safebuf(obj, model, workspace_ray, SegEndpt, flagCable)
            L{1} = SegEndpt{1,2} - SegEndpt{1,1};
            L{2} = SegEndpt{2,2} - SegEndpt{2,1};  
            obj.safe_dis = 0.5*(model.cableModel.cables{flagCable(1)}.diameter+...
                model.cableModel.cables{flagCable(2)}.diameter);
            % parallel or not by cross product
            cropro = CrossProductPoly(obj, L{1}, L{2});
            isparallel = all(round(cropro(:),10)==0); % test if all the elements are zero
            % 1 refers to parallel cables (all elements are zero)
            if isparallel == 1
                %% parallel cables
                % NEED TO BE FILLED IN
%                 % MinDis is MinDisPtSeg e.g. Ai v.s. lj
%                 pt = SegEndpt{1,1};
%                 segendpt = SegEndpt(2,:);
%                 seg = L{2};
%                 intvInterf = MinDisPtSeg(obj, model, workspace_ray, pt, segendpt, seg);
                warning('Parallel segments')
            else
                %% NON-parallel cables [C-C]
                % coefficients of nti/ntj/nhatcp_2/den(3*4 or 5*4 matrix)
                Coeff4Poly = Coeff_ntijdenhat_2(obj, model, workspace_ray, flagCable);
                temp_coeff{1} = Coeff4Poly(:,1)'; % nti      (row vector)
                temp_coeff{2} = Coeff4Poly(:,2)'; % ntj      (row vector)
                temp_coeff{3} = Coeff4Poly(:,3)'; % den      (row vector)
                temp_coeff{4} = Coeff4Poly(:,4)'; % nhatcp_2 (row vector)

                %% obtain the coefficient of polynomial w.r.t nti/ntj/den/hatcp_2
                % simplify & reshape to uniform form (ROW Vectors)
                coeff = cell(1,4);
                for i = 1:4
                    coeff{i} = CoeffUniPoly(obj, temp_coeff{i});
                end
                coeff_nti = coeff{1};
                coeff_ntj = coeff{2};
                coeff_den = coeff{3};
                coeff_nhatcp_2 = coeff{4};              
              
                %% solve the univariable polynomial equations
                % reshape to 5th degree (ROW Vectors) to addication
                coeff_nti = padarray(coeff_nti, [0 5-length(coeff_nti)], 'pre');
                coeff_ntj = padarray(coeff_ntj, [0 5-length(coeff_ntj)], 'pre');
                coeff_den = padarray(coeff_den, [0 5-length(coeff_den)], 'pre');
                coeff_nhatcp_2 = padarray(coeff_nhatcp_2, [0 5-length(coeff_nhatcp_2)], 'pre');
                
                % ---------(MinDis is the length of Common Perpendicular Segment (C.P.S))-start-----------------------
                % i.e. 0<ti & ti<1 & 0<tj & tj<1 & norm(C.P.S.)^2<dia^2
                % obtain the Coefficient of 5 Univariable Polynomials
                CoUniPol{1}  = coeff_nti;
                CoUniPol{2}  = coeff_ntj;
                CoUniPol{3}  = coeff_den - coeff_nti;
                CoUniPol{4}  = coeff_den - coeff_ntj;
                CoUniPol{5}  = (obj.safe_dis^2) * coeff_den - coeff_nhatcp_2;
                % ----------------- v2.0 ----------------------
                inequal = '>=';
                % intv_CP = RealRoots(CoUniPol, curflxvar, bcurflxvar, inequal);
                intv_CP = RealRootsIntersectSet(obj, model, workspace_ray, CoUniPol, inequal);
                clear CoUniPol;
                % ---------(MinDis is the length of common perpendicular segment)-end-------------------------

                % ---------(MinDis is the distance between one end-pt and the other segment)-start-----------------*
                % ti<0 & 0<tj & tj<1 & invt(pt,seg)
                % ti<0 & 0<tj & tj<1
                CoUniPol{1} = -coeff_nti;
                CoUniPol{2} = coeff_ntj;
                CoUniPol{3} = coeff_den - coeff_ntj;
                % --- v2.0 ---
                % invt(pt,seg): pt=Ai, seg = lj
                pt = SegEndpt{1,1};
                segendpt = SegEndpt(2,:);
                seg = L{2};
                intv_Ailj = IntersectInvPtSeg(obj, model, workspace_ray, CoUniPol, inequal, pt, segendpt, seg);
                clear CoUniPol;

                % 0<ti & ti<1 & tj<0 & invt(pt,seg)
                % 0<ti & ti<1 & tj<0
                CoUniPol{1} = coeff_nti;
                CoUniPol{2} = coeff_den - coeff_nti;
                CoUniPol{3} = -coeff_ntj;
                % --- v2.0 ---
                % invt(pt,seg): pt=Aj, seg = li
                pt = SegEndpt{2,1};
                segendpt = SegEndpt(1,:);
                seg = L{1};
                intv_Ajli = IntersectInvPtSeg(obj, model, workspace_ray, CoUniPol, inequal, pt, segendpt, seg);
                clear CoUniPol;

                % 0<ti & ti<1 & tj>1 & invt(pt,seg)
                % 0<ti & ti<1 & tj>1
                CoUniPol{1} = coeff_nti;
                CoUniPol{2} = coeff_den - coeff_nti;
                CoUniPol{3} = -(coeff_den - coeff_ntj);
                % --- v2.0 ---
                % invt(pt,seg): pt=Bj, seg = li
                pt = SegEndpt{2,2};
                segendpt = SegEndpt(1,:);
                seg = L{1};
                intv_Bjli = IntersectInvPtSeg(obj, model, workspace_ray, CoUniPol, inequal, pt, segendpt, seg);
                clear CoUniPol;


                % 1<ti & 0<tj & tj<1 & invt(pt,seg)
                % 1<ti & 0<tj & tj<1
                CoUniPol{1} = -(coeff_den - coeff_nti);
                CoUniPol{2} = coeff_ntj;
                CoUniPol{3} = coeff_den - coeff_ntj;
                % --- v2.0 ---
                % invt(pt,seg): pt=Bi, seg = lj
                pt = SegEndpt{1,2};
                segendpt = SegEndpt(2,:);
                seg = L{2};
                intv_Bilj = IntersectInvPtSeg(obj, model, workspace_ray, CoUniPol, inequal, pt, segendpt, seg);
                clear CoUniPol;
                % ---------(MinDis is the distance between one end-pt and the other segment)-end-------------------*


                % ---------(MinDis occurs when 2 foot pts of C.P.S. are both beyond (0,1))-start-----------------#
                % ti>1 & tj>1 
                tiflag = 1; tjflag = 1; tflag = [tiflag; tjflag];
                CoUniPol{1} = -(coeff_den - coeff_nti);
                CoUniPol{2} = -(coeff_den - coeff_ntj);
                % p: positive(>1); n: negative(<0)
                % intv_ipjp = IntersectInvPP(obj, tflag, CoUniPol, curflxvar, inequal, SegEndpt);
                intv_ipjp = IntersectInvPP(obj, model, workspace_ray, CoUniPol, tflag, inequal, SegEndpt);
                clear CoUniPol;

                % ti<0 & tj>1 
                tiflag = -1; tjflag = 1; tflag = [tiflag; tjflag];
                CoUniPol{1} = - coeff_nti;
                CoUniPol{2} = -(coeff_den - coeff_ntj);
                % intv_tt_1 = RealRootsIntersectSet(CoUniPol, curflxvar, bcurflxvar, inequal);
                % p: positive(>1); n: negative(<0)
                intv_injp = IntersectInvPP(obj, model, workspace_ray, CoUniPol, tflag, inequal, SegEndpt);
                clear CoUniPol;

                % ti>1 & tj<0
                tiflag = 1; tjflag = -1; tflag = [tiflag; tjflag];
                CoUniPol{1} = -(coeff_den - coeff_nti);
                CoUniPol{2} = - coeff_ntj;
                % intv_tt_1 = RealRootsIntersectSet(CoUniPol, curflxvar, bcurflxvar, inequal);
                % p: positive(>1); n: negative(<0)
                intv_ipjn = IntersectInvPP(obj, model, workspace_ray, CoUniPol, tflag, inequal, SegEndpt);
                clear CoUniPol;

                % ti<0 & tj<0
                tiflag = -1; tjflag = -1; tflag = [tiflag; tjflag];
                CoUniPol{1} = - coeff_nti;
                CoUniPol{2} = - coeff_ntj;
                % intv_tt_1 = RealRootsIntersectSet(CoUniPol, curflxvar, bcurflxvar, inequal);
                % p: positive(>1); n: negative(<0)
                intv_injn = IntersectInvPP(obj, model, workspace_ray, CoUniPol, tflag, inequal, SegEndpt);
                clear CoUniPol;
                % ---------(MinDis occurs when 2 foot pts of C.P.S. are both beyond (0,1))-end-----------------#


                % union sets of (intv_CP & intvPtSeg & intvPP)
                % 1*intv_CP and 4*intvPtSeg and 4*intvPtPt\in n*2 form
                intv = [intv_CP; intv_Ailj; intv_Ajli; intv_Bjli; intv_Bilj; intv_ipjp; intv_injp; intv_ipjn; intv_injn];
                if ~isempty(intv)
                    Left = intv(:, 1);
                    R = intv(:, 2);
                    [L_intv, R_intv] = Or_interval(obj, Left, R); % L_intv \in row vector form
                    % modify to open set (delete the isolated point)
                    tmp = round(L_intv - R_intv, 10);
                    L_intv(tmp==0) =[];
                    R_intv(tmp==0) =[];
                    out_temp = [L_intv', R_intv']; % n*2 matrix
                    % (option) change from T to angle (rad)
                    curtypevar= model.bodyModel.q_dofType;
                    curflxvar = workspace_ray.freeVariableIndex;
                    if curtypevar(curflxvar)==DoFType.ROTATION
                        out_temp = 2*atan(out_temp);
                    %     % rad -> deg
                    %     deg = out_temp/pi*180
                    end
                else
                    out_temp = [];
                end

                intvInterf = out_temp; % (m) or (rad)
            end

            %% to obtain IFW from Interference Intervals
            min = workspace_ray.freeVariableRange(1);
            max = workspace_ray.freeVariableRange(2);
            if ~isempty(intvInterf)
                [L_comp, R_comp] = Comp_interval(obj, intvInterf(:,1), intvInterf(:,2));
                if ~isempty(L_comp)
                    % results from fun'or_and_or' is closed intervals 
                    [resultL, resultR] = or_and_or(obj, min, max, L_comp, R_comp);
                    % avoid the adjcent intervals
                    % [resultL, resultR] = Or_interval(resultL, resultR);
                    % modify to open set (delete the isolated point)
                    tmp = round(resultL - resultR, 10);
                    resultL(tmp==0) =[];
                    resultR(tmp==0) =[];
                    intvifw_temp = [resultL', resultR'];
                else
                    intvifw_temp = [];
                end
            else
                intvifw_temp = [min, max];
            end
            intvifw2c = intvifw_temp; % (m) or (rad)
        end
        
        function coeff = Coeff_ntijdenhat_2(~, model, workspace_ray, flagCable)
            riflg = flagCable(1); rjflg = flagCable(2);
            
            curflxvar = workspace_ray.freeVariableIndex;
            magconvar = workspace_ray.fixedVariables';
            % ---- the bound of curflxvar ----------
            curValmin = workspace_ray.freeVariableRange(1);
            curValmax = workspace_ray.freeVariableRange(2);
            
            curtypevar= model.bodyModel.q_dofType;
            if curtypevar(curflxvar)==DoFType.TRANSLATION 
                % -- q1, q2, q3 ----
                curVal2 = (curValmin+curValmax)*0.5; 
                curVal1 = (curValmin+curVal2)*0.5;
                curVal3 = (curVal2+curValmax)*0.5;
                q_temp = zeros(model.numDofs,3);
                q_temp(:,1) = [magconvar(1:curflxvar-1) curVal1 magconvar(curflxvar:end)]';
                q_temp(:,2) = [magconvar(1:curflxvar-1) curVal2 magconvar(curflxvar:end)]';
                q_temp(:,3) = [magconvar(1:curflxvar-1) curVal3 magconvar(curflxvar:end)]';
                M = [curVal1^2 curVal1 1; curVal2^2 curVal2 1; curVal3^2 curVal3 1];
                trsf = ones(1,3);
            else % -- T1, T2, T3, T4, T5 -- 
                delta = (curValmax - curValmin)/6;
                angle1 = curValmin+delta*1; T1 = tan(0.5*angle1);
                angle2 = curValmin+delta*2; T2 = tan(0.5*angle2);
                angle3 = (curValmax+curValmin)/2; T3 = tan(0.5*angle3);
                angle4 = curValmin+delta*4; T4 = tan(0.5*angle4);
                angle5 = curValmin+delta*5; T5 = tan(0.5*angle5);
                q_temp = zeros(model.numDofs,5);
                q_temp(:,1) = [magconvar(1:curflxvar-1) angle1 magconvar(curflxvar:end)]';
                q_temp(:,2) = [magconvar(1:curflxvar-1) angle2 magconvar(curflxvar:end)]';
                q_temp(:,3) = [magconvar(1:curflxvar-1) angle3 magconvar(curflxvar:end)]';
                q_temp(:,4) = [magconvar(1:curflxvar-1) angle4 magconvar(curflxvar:end)]';
                q_temp(:,5) = [magconvar(1:curflxvar-1) angle5 magconvar(curflxvar:end)]';
                M = [T1^4 T1^3 T1^2 T1 1; T2^4 T2^3 T2^2 T2 1; T3^4 T3^3 T3^2 T3 1; T4^4 T4^3 T4^2 T4 1; T5^4 T5^3 T5^2 T5 1];
                trsf1 = (1+(T1)^2)^2; trsf2 = (1+(T2)^2)^2; trsf3 = (1+(T3)^2)^2; trsf4 = (1+(T4)^2)^2; trsf5 = (1+(T5)^2)^2;
                trsf = [trsf1 trsf2 trsf3 trsf4 trsf5];
            end
            
            % calculate W (constant vector)
            % Up to now, attachment pts come from the BASE
            ra_1 = model.cableModel.cables{riflg}.attachments{1}.r_OA;
            ra_2 = model.cableModel.cables{rjflg}.attachments{1}.r_OA;
            W = ra_2 - ra_1;
            Wx = W(1); Wy = W(2); Wz = W(3);
            
            % -- li & lj ( _1,_2,_3 )
            q_dot = zeros(model.numDofs,1);
            q_ddot = zeros(model.numDofs,1);
            w_ext = zeros(model.numDofs,1);
            nti = zeros(size(q_temp,2),1);
            ntj = zeros(size(q_temp,2),1);
            nhatcp_2 = zeros(size(q_temp,2),1);
            den = zeros(size(q_temp,2),1);
            for cnt = 1:size(q_temp,2)
                model.update(q_temp(:,cnt), q_dot, q_ddot,w_ext);
                li = model.cableModel.cables{riflg}.segments{1}.segmentVector;
                lj = model.cableModel.cables{rjflg}.segments{1}.segmentVector;
                lix = li(1); liy = li(2); liz = li(3);
                ljx = lj(1); ljy = lj(2); ljz = lj(3);
                % calculate the nti/ntj/nhatcp_2/den
                nti(cnt) = trsf(cnt)*((Wy*liy+Wz*liz)*ljx^2+((-Wx*liy-Wy*lix)*ljy-ljz*(Wx*liz+Wz*lix))*ljx+(Wx*lix+Wz*liz)*ljy^2-ljz*(Wy*liz+Wz*liy)*ljy+ljz^2*(Wx*lix+Wy*liy));
                ntj(cnt) = trsf(cnt)*((-Wy*ljy-Wz*ljz)*lix^2+((Wx*ljy+Wy*ljx)*liy+liz*(Wx*ljz+Wz*ljx))*lix+(-Wx*ljx-Wz*ljz)*liy^2+liz*(Wy*ljz+Wz*ljy)*liy-liz^2*(Wx*ljx+Wy*ljy));
                nhatcp_2(cnt) = trsf(cnt)*(((lix*ljy-liy*ljx)*Wz+Wx*liy*ljz+(-Wx*ljy+Wy*ljx)*liz-Wy*lix*ljz)^2);
                den(cnt) = trsf(cnt)*((ljy^2+ljz^2)*lix^2-2*ljx*(liy*ljy+liz*ljz)*lix+(ljx^2+ljz^2)*liy^2-2*liy*liz*ljy*ljz+liz^2*(ljx^2+ljy^2));
            end
            % coefficient of nti (denote: a') \in 3*1 or 5*1 vector
            coeff_nti = M\nti;
            % coefficient of ntj (denote: b')
            coeff_ntj = M\ntj;
            % coefficient of nhatcp_2 (denote: c')
            coeff_nhatcp_2 = M\nhatcp_2;
            % coefficient of den (denote: d')
            coeff_den = M\den;
            % finial results in the order of [nti ntj den nhatcp_2]
            coeff = [coeff_nti coeff_ntj coeff_den coeff_nhatcp_2]; % \in 3*4 or 5*4 matrix
        end
        
        function y = CrossProductPoly(~, x1, x2)
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
        
        function  coeff = CoeffUniPoly(~, temp_coeff)
            % simplify twice
            cnt = 1;
            while cnt <= 2
                [q, r] = deconv(temp_coeff, [1 0 1]);
                if round(r, 10) == 0 % just for check
                    % disp('remainder is zero!');
                    temp_coeff = q;
                else
                    break;
                end        
                cnt = cnt + 1;
            end
            % reshape to uniform form (make 1st entry (from left) is nonzero)
            temp_coeff = round(temp_coeff, 12);
            k = find(temp_coeff ~= 0, 1); 
            coeff = temp_coeff(k:end);
        end
        
        function Real_invt = RealRootsIntersectSet(obj, model, workspace_ray, CoUniPol, inequal)
            curflxvar = workspace_ray.freeVariableIndex;
            % Real_invt \in % n*2 matrix
            curtypevar= model.bodyModel.q_dofType;
            bcurflxvar = workspace_ray.freeVariableRange;
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
                    if curtypevar(curflxvar)==DoFType.TRANSLATION 
                        min = bcurflxvar(1);
                        max = bcurflxvar(2);
                    else
                        min = tan(0.5*(bcurflxvar(1)));
                        max = tan(0.5*(bcurflxvar(2)));
                    end
                    midpts = (min+max)*0.5;
                    polyval_temp = round(polyval(CoUniPol{i}, midpts), 12);
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
                    % ----
                    else
                        CASPR_log.Error('Invalid Sign or solution is zero');
                    end
                end
            end

            if isempty(real_root_pts) % all inequality solns are [-inf inf]
                Real_invt = [min max];
            else
                if curtypevar(curflxvar)==DoFType.TRANSLATION 
                    min = bcurflxvar(1);
                    max = bcurflxvar(2);
                else
                    if bcurflxvar(1) == -pi
                        min = -inf;
                    else
                        min = tan(0.5*(bcurflxvar(1))); % --> T
                    end
                    if bcurflxvar(2) == pi
                        max = inf;
                    else
                        max = tan(0.5*(bcurflxvar(2))); % --> T
                    end
                end
                if min>= max
                    CASPR_log.Error('Invalid limits of flexible variable');
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
                    polyval_temp = round(polyval(CoUniPol{i}, MidPts),12); % same as the form of MidPts
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
                        CASPR_log.Error('Invaid inequality');
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
        
        function [L_or, R_or] = Or_interval(obj, L, R)
            if length(L) ~= length(R)
                CASPR_log.Error('Invaid Input');
            elseif max(L > R)                
                CASPR_log.Error('Please ensure that L <= R');
            elseif isempty(L) % the input is one null interval
                CASPR_log.Warn('Empty Input'); % union(a null space)
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

        function [L_or, R_or] = Or_interval_2(~, L, R)
            if length(L) ~= length(R)
                CASPR_log.Errer('Invaid Input');
            elseif max(L > R)
                CASPR_log.Error('Please ensure that L <= R');
            elseif isempty(L) % both L and R are empty could happen; one of them is empty would get error from the condition 'length(L) ~= length(R)'
                CASPR_log.Warn('Empty Input'); % union(a null space)
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
                CASPR_log.Errer('Invaid Input');
            elseif max(L > R)
                CASPR_log.Error('Please ensure that L <= R');
            % elseif isempty(option) || strcmp(option, 'and') % default of option is 'and'
            elseif isempty(L) % One null set could happen; one of them is empty would get error from the condition 'length(L) ~= length(R)'
                    CASPR_log.Warn('Empty Input'); % inters(one null set); see Input-2)
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
        
        function intvPtSeg = IntersectInvPtSeg(obj, model, workspace_ray, CoUniPol, inequal, pt, segendpt, seg)
            intv_temp1 = RealRootsIntersectSet(obj, model, workspace_ray, CoUniPol, inequal);
            if ~isempty(intv_temp1)
                intv_temp2 =  MinDisPtSeg(obj, model, workspace_ray, pt, segendpt, seg);
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

        function intvPP = IntersectInvPP(obj, model, workspace_ray, CoUniPol, tflag, inequal, SegEndpt)
        % function intvPP = IntersectInvPP(obj, tflag, CoUniPol, curflxvar, inequal, SegEndpt)
            intv_tt_1 = RealRootsIntersectSet(obj, model, workspace_ray, CoUniPol, inequal);
            if ~isempty(intv_tt_1)
                intv_tt_2 = MinDisPtPt(obj, model, workspace_ray, tflag, SegEndpt);
                % intersection
                if ~isempty(intv_tt_2)
                    [resultL, resultR] = or_and_or(obj, intv_tt_1(:,1), intv_tt_1(:,2), intv_tt_2(:,1), intv_tt_2(:,2));
                    intvPP = [resultL', resultR'];
                else
                    intvPP = [];
                end    
            else
                intvPP = [];
            end
        end

        function intvPtPt = MinDisPtPt(obj, model, workspace_ray, tflag, SegEndpt)
            flxvar = workspace_ray.freeVariableIndex;
            % specify the transfer operator
            curtypevar= model.bodyModel.q_dofType;
            if curtypevar(flxvar)==DoFType.TRANSLATION 
            %     transf = [0 1]; % transfer to 3*2 matrix
                denomi = 1; % denominator of univariable polynomial coefficients
            else 
            %     transf = [1 0 1]; % transfer to 3*3 matrix
                denomi = [1 0 1];
            end

           %% form the condition polynomial
            % --- v 2.0 -----
            % set up the pseudo seg end pts and the according pseudo segment
            switch tflag(1)
                case -1 % ti<0
                    pt1 = SegEndpt{1,2}; pt2 = SegEndpt{1,1};
                case 1 % ti>1
                    pt1 = SegEndpt{1,1}; pt2 = SegEndpt{1,2};
                otherwise
                    CASPR_log.Error('Invalid ti_flag');
            end
            switch tflag(2)
                case -1 % tj<0
                    pt3 = SegEndpt{2,2}; pt4 = SegEndpt{2,1};
                case 1 % tj>1
                    pt3 = SegEndpt{2,1}; pt4 = SegEndpt{2,2};
                otherwise
                    CASPR_log.Error('Invalid tj_flag');
            end
            % P1 = dot(AiBj, li)/norm(li)
            pt1pt4 = pt4 - pt1;
            litemp = pt2 - pt1;
            tempdot = conv(pt1pt4(1,:), litemp(1, :))+conv(pt1pt4(2,:), litemp(2, :))+conv(pt1pt4(3,:), litemp(3, :)); % dot(AiBj, li)
            % P1 = 0
            P1CondCoeff{1} = tempdot;
            % P1 = norm(li)
            % temp = -SquarePoly(L{1}); % -norm(li)^2
            temp = -SquarePoly(obj, litemp);
            P1CondCoeff{2} = AddPolyCoeff_2P(obj, tempdot, temp);

            % P2 = dot(AjBi, lj)/norm(lj)
            pt3pt2 = pt2 - pt3;
            ljtemp = pt4 - pt3;
            tempdot = conv(pt3pt2(1,:),ljtemp(1, :))+conv(pt3pt2(2,:), ljtemp(2, :))+conv(pt3pt2(3,:),ljtemp(3, :)); % dot(AjBi, lj)
            % P2 = 0
            P2CondCoeff{1} = tempdot;
            % P2 = norm(lj)
            temp = -SquarePoly(obj, ljtemp); % norm(lj)^2
            P2CondCoeff{2} = AddPolyCoeff_2P(obj, tempdot, temp);

            SudoL = {litemp, ljtemp}; % Pseudo vector of 2 segments \\ 2*2 cell array
            SudoSegEndpt = [{pt1}, {pt2}; {pt3}, {pt4}]; % Pseudo end point of 2 segments \\ 2*2 cell array
            PCondPoly = [P1CondCoeff; P2CondCoeff]; % 2*2 cell matrix

            %% interval calculation 
            % MinDisPtPt is a piece-wise function with 8 Pce(pieces)
            intv = [];
            for P1_flag = -1:1:1
                for P2_flag = -1:1:1
                    intvTemp = MinDdiPP_8Pce(obj, model, workspace_ray, P1_flag, P2_flag, PCondPoly, SudoL, SudoSegEndpt, denomi);
                    intv = [intv; intvTemp]; % \in n*2 matrix or null
                end    
            end
            % union set of (intv)
            if ~isempty(intv)
                L_temp = intv(:,1);
                R_temp = intv(:,2);
                [L_intv, R_intv] = Or_interval(obj, L_temp, R_temp); % L_intv \in row vector form
                intv_temp = [L_intv', R_intv'];
            else
                intv_temp = [];
            end
            intvPtPt = intv_temp;
        end
        
        function intvPtSeg =  MinDisPtSeg(obj, model, workspace_ray, pt, segendpt, seg)
            % bcurflxvar = [obj.model.bodyModel.q_min(flxvar); obj.model.bodyModel.q_max(flxvar)];
            %% specify the transfer operator
            flxvar = workspace_ray.freeVariableIndex;
            curtypevar= model.bodyModel.q_dofType;
            if curtypevar(flxvar)==DoFType.TRANSLATION
                transf = [0 1]; % transfer to 3*2 matrix
                denomi = 1;
            else 
                transf = [1 0 1]; % transfer to 3*3 matrix
                denomi = transf;
            end
            if size(pt,2) == 1 % if pt is a constant column vector
                pt = pt* transf; % pt \in 3*2(translation) or 3*3(rotation) matrix
            end

            APt = pt - segendpt{1};
            BPt = pt - segendpt{2};

            %% potential MinDis < diameter 
            % norm(APt) < dia
            temp1 = SquarePoly(obj, APt);
            % temp2 = -conv(conv(denomi,denomi), diameter^2);
            temp2 = -(obj.safe_dis)^2 * (conv(denomi,denomi));
            mindisPoly{1} = AddPolyCoeff_2P(obj, temp1, temp2);
            % cross(APt, l)/norm(l) < dia
            temp1 = CrossProductPoly(obj,APt,seg);
            temp12 = SquarePoly(obj, temp1);
            temp22 = -(obj.safe_dis)^2 * conv(conv(denomi, denomi),SquarePoly(obj, seg)); % <<<<<<<<<<<<
            mindisPoly{2} = AddPolyCoeff_2P(obj, temp12, temp22);
            % norm(BPt) < dia
            temp1 = SquarePoly(obj, BPt);
            % temp2 = -conv(conv(denomi,denomi), diameter^2);
            temp2 = -(obj.safe_dis)^2 * (conv(denomi,denomi));
            mindisPoly{3} = AddPolyCoeff_2P(obj, temp1, temp2);


            %% form the condition polynomial
            % dot(APt, l)
            tempdot = conv(APt(1,:), seg(1,:))+conv(APt(2,:), seg(2,:))+conv(APt(3,:), seg(3,:));
            % dot(APt, l) = 0
            CondPoly{1} = tempdot;
            % dot(APt, l) = norm(l)^2
            temp_norm2 = -SquarePoly(obj,seg);
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
            intv1 = RealRootsIntersectSet(obj, model, workspace_ray, CoUniPol_temp, inequal);
            clear CoUniPol_temp;

            % intv2 (case2)
            CoUniPol_temp{1} = mindisPoly{2};
            CoUniPol_temp{2} = -CondPoly{1};
            CoUniPol_temp{3} = CondPoly{2};
            intv2 = RealRootsIntersectSet(obj, model, workspace_ray, CoUniPol_temp, inequal);
            clear CoUniPol_temp;

            % intv3 (case3)
            CoUniPol_temp{1} = mindisPoly{3};
            CoUniPol_temp{2} = -CondPoly{2};
            intv3 = RealRootsIntersectSet(obj, model, workspace_ray, CoUniPol_temp, inequal);
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

        function coeff_sum = AddPolyCoeff_2P(~, x1, x2)
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
        
        function intvPP = MinDdiPP_8Pce(obj, model, workspace_ray, P1_flag, P2_flag, PCondPoly, SudoL, SudoSegEndpt, denomi)
            % Assumption
            %  inequal = '<';
            inequal = '<=';
            cnt = 1;
            switch P1_flag % Condition 1 flag
                case -1 % <0
                    cartp{1} = SudoSegEndpt{1,1}; % Ai
                    CoUniPol{cnt} = PCondPoly{1,1}; cnt=cnt+1;
                case 0 % (0, norm(li))
                    cartp{1} = {SudoL{1}; SudoSegEndpt(1,:)}; % li cell
                    CoUniPol{cnt} = -PCondPoly{1,1}; cnt=cnt+1;
                    CoUniPol{cnt} =  PCondPoly{1,2}; cnt=cnt+1;
                case +1 % > norm(li)
                    cartp{1} = SudoSegEndpt{1,2}; % Bi
                    CoUniPol{cnt} = -PCondPoly{1,2}; cnt=cnt+1;
            end
            switch P2_flag % Condition 2 flag
                case -1 % <0
                    cartp{2} = SudoSegEndpt{2,1}; % Aj
                    CoUniPol{cnt} = PCondPoly{2,1};
                case 0 % (0, norm(lj))
                    cartp{2} = {SudoL{2}; SudoSegEndpt(2,:)}; % lj cell
                    CoUniPol{cnt} = -PCondPoly{2,1}; cnt=cnt+1;
                    CoUniPol{cnt} =  PCondPoly{2,2}; 
                case +1 % > norm(lj)
                    cartp{2} = SudoSegEndpt{2,2}; % Bj
                    CoUniPol{cnt} = -PCondPoly{2,2};
            end
            % intv of (P1 & P2)
            intv2Cond = RealRootsIntersectSet(obj, model, workspace_ray, CoUniPol, inequal);
            clear CoUniPol;
            if ~isempty(intv2Cond)
                % intv of MinDis
                Pflag = [P1_flag, P2_flag];
                if all(Pflag) % MinDis is norm(pt1,pt2) i.e., no zero
                    pt1 = cartp{1}; pt2 = cartp{2};
                    pt1pt2 = pt2 - pt1;
                    temp1 = SquarePoly(obj, pt1pt2);
                    temp2 = -(obj.safe_dis)^2 * (conv(denomi,denomi));
                    mindisPoly = {round(AddPolyCoeff_2P(obj, temp1, temp2),8)};
                    % mindisPoly = {AddPolyCoeff_2P(obj, temp1, temp2)};
                    % mindisPoly = {[0 0 0 0 0]};
                    intvMinDis = RealRootsIntersectSet(obj, model, workspace_ray, mindisPoly, inequal);
                    % intvMinDis = RealRootsIntersectSet(obj, mindisPoly, flxvar, '<');
                elseif (P1_flag+P2_flag)~=0 % MinDis is MinDisPtSeg
                    ind_seg = find(Pflag==0);
                    % ind_pt  = find(Pflag);% to determine who is Pt (i.e.,flag~=0)
                    ind_pt = 3-ind_seg;
                    pt  = cartp{ind_pt};
                    seg = cartp{ind_seg}{1};
                    segendpt = cartp{ind_seg}{2};
                    intvMinDis =  MinDisPtSeg(obj, model, workspace_ray, pt, segendpt, seg);
                else % P1_flag==0, P2_flag==0
                    intvMinDis = [];
                end
                % intersetion
                if ~isempty(intvMinDis)
                    [resultL, resultR] = or_and_or(obj, intv2Cond(:,1), intv2Cond(:,2), intvMinDis(:,1), intvMinDis(:,2));
                    intvPP = [resultL', resultR']; % \in n*2 matrix
                else
                    intvPP = [];
                end
            else
                intvPP = [];
            end
        end

        function [L_comp, R_comp] = Comp_interval(obj, L, R)
            if length(L) ~= length(R)
                CASPR_log.Error('Invaid Input');
            elseif max(L > R)
                CASPR_log.Error('Please ensure that L <= R');
            elseif isempty(L) % both L and R are empty could happen; one of them is empty would get error from the condition 'length(L) ~= length(R)'
                CASPR_log.Warn('Empty input')  % comp(null)
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
                CASPR_log.Error('Invaid Input');
            elseif max(L > R)
                CASPR_log.Error('Please ensure that L <= R');
            elseif isempty(L) % both L and R are empty could happen; one of them is empty would get error from the condition 'length(L) ~= length(R)'
                CASPR_log.Warn('Empty input');  % comp(null set)
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
                    CASPR_log.Error('Make sure 2 non-intersecting intervals');
                end
            else
                CASPR_log.Error('Make sure the number of inputs <= 2');
            end
        end

        
        
    end
    
end

