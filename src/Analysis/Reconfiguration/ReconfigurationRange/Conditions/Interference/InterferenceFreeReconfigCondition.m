% Class to compute
% Author        : Paul Cheng
% Created       : 2020
% Description   :

classdef InterferenceFreeReconfigCondition < ReconfigConditionBase
    properties (Constant)
        ROUNDING_DIGIT = 4;
        OBSTACLE_DIST_TOL = 0.002; % the tol should be handle carefully for detection of inside the obstacle
        root_tol = 1e-4;
        
        type = ReconfigConditionType.INTERFERENCE_CABLE_OBSTACLE;
        
    end
    properties (SetAccess = protected)
        % Set constants
        SweptSurfaceDegree;           % Array for the cable-swept surface: 1-> linear 2-> quad ...
        
        numCables;                  % The number of cables
        numSegment;                 % The number of cable segments
        
        FittingForHighDegBoundary   % For high degree boundaries only
        
        is_compiled_mode;
        
        needImplicitSweptSurface = 0;
        
        verifyInterior;            % For consider the hollow(interior of) obstacle -> 1
        
        Obstacles;                 % all obstacles
        NumObstacle;
        u_range;
        G_uv
        G_xyz;
        G_coeffs;
        OA_u;
        OB;
        OA_0;
        OA_1;
        OA_at_u;
        OA_i_hat;
        
        Case1time = 0;
        Case2time = 0;
        Case3time = 0;
        Verifytime = 0;
        Case4time = 0;
        total_time = 0;
        CableSegmentInterRange;
        NumSol = 0;
        
        model
    end
    
    methods
        % Constructor for interference free worksapce
        function w = InterferenceFreeReconfigCondition(model, Obstacles)
            
            %             w.SurfaceTypeIndex = SurfaceTypeIndex;
            w.model = model;
            w.numCables = model.numCables;
            w.NumObstacle = numel(Obstacles);
            w.Obstacles = Obstacles;
            w.is_compiled_mode = (model.modelMode == ModelModeType.COMPILED);
            w.numSegment = size(model.cableModel.r_OAs,2);
            
            field1 = 'u';  value1 = [];
            field2 = 'v';  value2 = [];
            field3 = 'ObstacleNum';  value3 = [];
            field4 = 'SurfaceNum';  value4 = [];
            CableSegmentInterRange(w.numSegment) = struct(field1,value1,field2,value2,field3,value3,field4,value4);
            w.CableSegmentInterRange = CableSegmentInterRange;
            for OI = 1:w.NumObstacle
                OBS_i = w.Obstacles(OI);
                if OBS_i.NumBoundary ~= 0
                    w.needImplicitSweptSurface = 1;
                end
                NeglectInterior(OI) = w.Obstacles(OI).NeglectInterior;
            end
            if all(NeglectInterior)
                w.verifyInterior = false;
            else
                w.verifyInterior = true;
            end
            w.G_uv = @(u,v,OB,OA)(OA*[u.^2;u;1] ./ (1+u.^2) - OB)*v +  OB;
            
            
        end
        
        function TotalInterval =  evaluateFunction(obj, reconfig_element)
            intervals = cell(obj.model.numCables,1);
            U_intervals = cell(obj.model.numCables,1);
            q_zero = zeros(size(obj.model.q));
            obj.SweptSurfaceDegree = reconfig_element.SweptSurfaceDegree;
            if obj.model.modelMode == ModelModeType.COMPILED
                cable_attachment = obj.model.cableModel.compiled_r_OAs_fn(obj.model.q ,q_zero,q_zero,q_zero);
            else
                %                 obj.model.update(obj.model.q ,q_zero,q_zero,q_zero);
                cable_attachment = obj.model.cableModel.r_OAs;
                
            end
            
            obj.OB =  cable_attachment(4:6,:);
            
            if obj.needImplicitSweptSurface
                [obj.G_coeffs,obj.G_xyz,obj.OA_u] = obj.GetGxyz(reconfig_element);
            end
            
            
            for CSI = 1:obj.model.numCables
                
                if ismember(CSI,reconfig_element.Reconfig_Cable_ID)
                    
                    ID = find(ismember(reconfig_element.Reconfig_Cable_ID,CSI));
                    
                    var_range = reconfig_element.Info(ID).VariableRange;
                    var = reconfig_element.Info(ID).Variable;
                    
                    obj.OA_1(CSI,:) = reconfig_element.Info(ID).Path(var_range(end));
                    obj.OA_0(CSI,:) = reconfig_element.Info(ID).Path(var_range(1));
                    u_range = reconfig_element.Info(ID).VariableRange;
                    FeasibleRange = u_range;
                    
                    obj.CableSegmentInterRange(CSI).u = [];
                    obj.CableSegmentInterRange(CSI).v = [];
                    obj.CableSegmentInterRange(CSI).ObstacleNum = [];
                    obj.CableSegmentInterRange(CSI).SurfaceNum = [];
                    obj.NumSol = 0;
                    
                    for OI = 1:obj.NumObstacle
                        
                        for SI = 1:obj.Obstacles(OI).NumSurfaces
                            tic
                            obj.TangentToSurface(ID,CSI,OI,SI,u_range);
                            obj.Case3time = obj.Case3time + toc;
                            tic
                            obj.CurveCutSurface(CSI,OI,SI,u_range);
                            obj.Case4time = obj.Case4time + toc;
                            tic
                            obj.DetectStartEndIntersection(CSI,OI,SI,u_range);
                            obj.Case2time = obj.Case2time + toc;
                        end
                        
                        tic
                        for BI = 1:obj.Obstacles(OI).NumBoundary
                            obj.SegmentIntersection_Gxyz(ID,CSI,OI,BI,u_range);
                        end
                        obj.Case1time = obj.Case1time + toc;
                    end
                    
                    
                    tic
                    u = obj.CableSegmentInterRange(CSI).u;
                    v = obj.CableSegmentInterRange(CSI).v;
                    if ~isempty(u)
                        
                        if obj.verifyInterior
                            if obj.SweptSurfaceDegree(ID) == 1
                                IntersectPoint = (obj.OA_u{ID}*[u;ones(1,numel(u))] - obj.OB(:,CSI)).*v + obj.OB(:,CSI); % [x;y;z,x;y;z]
                            else
                                IntersectPoint = (obj.OA_u{ID}*[u.^2;u;ones(1,numel(u))]./[1+u.^2] - obj.OB(:,CSI)).*v + obj.OB(:,CSI);
                            end
                            
                            inHullu = [];
                            for OI = 1:obj.NumObstacle
                                isInHull = obj.DectectPointInObstacle(OI,IntersectPoint);
                                inHullu = [inHullu,u(isInHull)];
                                
                            end
                            
                            if ~isempty(inHullu)
                                FeasibleRangeSI = [];
                                last_is_feasible = 0;
                                u_unique = unique([inHullu,u_range']);
                                for UI = 1:numel(u_unique) - 1
                                    u_middle = 0.5*(u_unique(UI) + u_unique(UI+1));
                                    if obj.SweptSurfaceDegree(ID) == 1
                                        OA_u_s = obj.OA_u{ID}*[u_middle;ones(1,numel(u_middle))]; % [x;y;z,x;y;z]
                                    else
                                        OA_u_s = obj.OA_u{ID}*[u_middle.^2;u;ones(1,numel(u_middle))];
                                    end
                                    
                                    OB_u_s = obj.OB(:,CSI);
                                    
                                    
                                    isFeasibleRange = true;
                                    for OI = 1:obj.NumObstacle
                                        for SI = 1:obj.Obstacles(OI).NumSurfaces
                                            isInfeasible = obj.VerifyIntersection(OB_u_s,OA_u_s,OI,SI);
                                            if isInfeasible
                                                isFeasibleRange = false;
                                                break
                                            end
                                        end
                                    end
                                    
                                    
                                    if isFeasibleRange
                                        if ~last_is_feasible
                                            FeasibleRangeSI = [FeasibleRangeSI, u_unique(UI) , u_unique(UI+1)];
                                        else
                                            FeasibleRangeSI(end) = u_unique(UI+1);
                                        end
                                        last_is_feasible = 1;
                                    else
                                        last_is_feasible = 0;
                                    end
                                end
                                
                                
                            else
                                FeasibleRangeSI = u_range;
                            end
                            
                        else
                            
                            FeasibleRangeSI = u_range;
                            if obj.SweptSurfaceDegree(ID) == 1
                                IntersectPoint = (obj.OA_u{ID}*[u;ones(1,numel(u))] - obj.OB(:,CSI)).*v + obj.OB(:,CSI); % [x;y;z,x;y;z]
                            else
                                IntersectPoint = (obj.OA_u{ID}*[u.^2;u;ones(1,numel(u))]./[1+u.^2] - obj.OB(:,CSI)).*v + obj.OB(:,CSI);
                            end
                            
                            
                            for OI = 1:obj.NumObstacle
                                isInHull = obj.DectectPointInObstacle(OI,IntersectPoint);
                                inHullu = u(isInHull);
                                FeasibleRangeOI = u_range;
                                if ~isempty(inHullu)
                                    tmp_u = unique(inHullu);
                                    InfeasibleRange = [tmp_u(1),tmp_u(end)];
                                    tmp_CurrentFeasibleRange = obj.FindFeasibleRange(u_range,InfeasibleRange)';
                                    tmp_CurrentFeasibleRange = tmp_CurrentFeasibleRange(:);
                                    CurrentFeasibleRange = tmp_CurrentFeasibleRange';
                                    FeasibleRangeOI = obj.range_intersection(CurrentFeasibleRange,FeasibleRangeOI);
                                    
                                end
                                FeasibleRangeSI = obj.range_intersection(FeasibleRangeSI,FeasibleRangeOI);
                            end
                        end
                        obj.Verifytime = obj.Verifytime + toc;
                        FeasibleRange = obj.range_intersection(FeasibleRange,FeasibleRangeSI);
                        %                     FeasibleRange = reshape(FeasibleRange,size(FeasibleRange,2)/2,2)';
                        count = 0;
                        for FID = 1:size(FeasibleRange,2)
                            if rem(FID, 2)
                                count = count + 1;
                                if obj.SweptSurfaceDegree(ID) == 1
                                    intervals{CSI}{count}(:,1) = obj.OA_u{ID}*[FeasibleRange(FID);1];
                                else
                                    intervals{CSI}{count}(:,1) = obj.OA_u{ID}*[FeasibleRange(FID)^2;FeasibleRange(FID);1];
                                end
                            else
                                if obj.SweptSurfaceDegree(ID) == 1
                                    intervals{CSI}{count}(:,2) = obj.OA_u{ID}*[FeasibleRange(FID);1];
                                else
                                    intervals{CSI}{count}(:,2) = obj.OA_u{ID}*[FeasibleRange(FID)^2;FeasibleRange(FID);1];
                                end
                                %                         intervals{CSI}{count} = [obj.OA_u{ID}*[FeasibleRange(FID,1);1] obj.OA_u{ID}*[FeasibleRange(FID,2);1]];
                            end
                        end
                        r_count =  1:2:numel(FeasibleRange);
                        c_count =  2:2:numel(FeasibleRange);                        
                        U_intervals{CSI}= [FeasibleRange(r_count)',FeasibleRange(c_count)'];
                    end
                    
                else
                    intervals{CSI}{:,1} = [obj.model.cableModel.r_OAs(1:3,CSI),obj.model.cableModel.r_OAs(1:3,CSI)];
                    U_intervals{CSI} = [0,1];
                end
            end
            TotalInterval = U_intervals;
%             TotalInterval.FeasibleInterval = U_intervals;
%             TotalInterval.FeasibleOARange = intervals;
        end
        
        %% %function to find intersection between Obstacles and cable swept surface
        function TangentToSurface(obj,ID,CSI,OI,SI,u_range)
            
            if obj.Obstacles(OI).SurfaceDegree(SI) == 1
                return;
            end
            Delta_n_coeff = [];
            if obj.SweptSurfaceDegree(ID) == 2
                if obj.Obstacles(OI).SurfaceDegree(SI) == 4
                    a = H4_O(obj.OA_u{ID},obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI});
                    b = H3_O(obj.OA_u{ID},obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI});
                    c = H2_O(obj.OA_u{ID},obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI});
                    d = H1_O(obj.OA_u{ID},obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI});
                    e = H0_O(obj.OA_u{ID},obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI});
                    Delta_n_coeff = Delta4u(a,b,c,d,e);
                elseif obj.Obstacles(OI).SurfaceDegree(SI) == 3
                    a = H3_O(obj.OA_u{ID},obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI});
                    b = H2_O(obj.OA_u{ID},obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI});
                    c = H1_O(obj.OA_u{ID},obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI});
                    d = H0_O(obj.OA_u{ID},obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI});
                    Delta_n_coeff = Delta3u(a,b,c,d);
                elseif obj.Obstacles(OI).SurfaceDegree(SI) == 2
                    a = H2_O(obj.OA_u{ID},obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI});
                    b = H1_O(obj.OA_u{ID},obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI});
                    c = H0_O(obj.OA_u{ID},obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI});
                    Delta_n_coeff = Delta2u(a,b,c);
                end
            elseif obj.SweptSurfaceDegree(ID) == 1
                if obj.Obstacles(OI).SurfaceDegree(SI) == 4
                    a = zeros(1,9); b = zeros(1,7); c = zeros(1,5); d = zeros(1,3); e = zeros(1,1);
                    a(5:end) = H4_T(obj.OA_u{ID},obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI});
                    b(4:end) = H3_T(obj.OA_u{ID},obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI});
                    c(3:end) = H2_T(obj.OA_u{ID},obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI});
                    d(2:end) = H1_T(obj.OA_u{ID},obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI});
                    e(1:end) = H0_T(obj.OA_u{ID},obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI});
                    Delta_n_coeff = Delta4u(a,b,c,d,e);
                elseif obj.Obstacles(OI).SurfaceDegree(SI) == 3
                    a = zeros(1,7); b = zeros(1,5); c = zeros(1,3); d = zeros(1,1);
                    a(4:end) = H3_T(obj.OA_u{ID},obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI});
                    b(3:end) = H2_T(obj.OA_u{ID},obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI});
                    c(2:end) = H1_T(obj.OA_u{ID},obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI});
                    d(1:end) = H0_T(obj.OA_u{ID},obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI});
                    Delta_n_coeff = Delta3u(a,b,c,d);
                elseif obj.Obstacles(OI).SurfaceDegree(SI) == 2
                    a = zeros(1,5); b = zeros(1,3); c = zeros(1,1);
                    a(3:end) = H2_T(obj.OA_u{ID},obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI});
                    b(2:end) = H1_T(obj.OA_u{ID},obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI});
                    c = H0_T(obj.OA_u{ID},obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI});
                    Delta_n_coeff = Delta2u(a,b,c);
                end
            end
            
            if any(isnan(Delta_n_coeff))
                Delta_n_coeff = zeros(size(Delta_n_coeff));
            end
            
            u_roots = obj.polyRootsTol(Delta_n_coeff,obj.root_tol);
            
            u_roots(u_roots>u_range(2)) = [];
            u_roots(u_roots<u_range(1)) = [];
            if ~isempty(u_roots)
                for UI = 1:size(u_roots,1)
                    
                    if obj.SweptSurfaceDegree(ID) == 1
                        FoGv = FoGvT(obj.OA_u{ID},obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI},u_roots(UI));
                    else
                        FoGv = FoGvO(obj.OA_u{ID},obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI},u_roots(UI));
                    end
                    
                    v_roots = unique(obj.polyRootsTol(FoGv,4e-2)); % this tolerance need to be handle carefully, high degree need bigger tol
                    %                     v_lower = norm((obj.OB(:,CSI) - obj.OA_u(:,ID)))/norm((obj.OB_at_u{CSI}(u_roots(UI))-obj.OB(:,CSI)));
                    v_roots(v_roots>1) = [];
                    v_roots(v_roots<0) = [];
                    for VI = 1:numel(v_roots)
                        obj.NumSol = obj.NumSol + 1;
                        obj.CableSegmentInterRange(CSI).u(obj.NumSol) =  u_roots(UI);
                        obj.CableSegmentInterRange(CSI).v(obj.NumSol) =  v_roots(VI);
                        obj.CableSegmentInterRange(CSI).ObstacleNum(obj.NumSol) = OI;
                        obj.CableSegmentInterRange(CSI).SurfaceNum(obj.NumSol) = SI;
                    end
                end
            end
        end
        
        %%
        function isInHull = DectectPointInObstacle(obj,OI,Points)
            % POINTS - > N X 3
            % inhull approach
%             tic
            isInHull = inhull(Points',obj.Obstacles(OI).ObstacleHull.SimplifiedVertices,[],1e-3);
%             toc
            % analytical approach
            %             tic
            %             for i = 1:size(Points,2)
            %                 IntersectPoint = Points(:,i);
            %                 %                 if obj.Obstacles(OI).NumSurfaces == 1
            %                 %                     % A lazy way for one surface obstacles =P
            %                                     XYZ_Range = obj.Obstacles(OI).SurfacesXYZRange{1};
            %                                    Range_Detection(i,:) = [IntersectPoint(1) >= XYZ_Range(1) &  IntersectPoint(1) <= XYZ_Range(2) & ...
            %                                      IntersectPoint(2) >= XYZ_Range(3) &  IntersectPoint(2) <= XYZ_Range(4) & ...
            %                                       IntersectPoint(3) >= XYZ_Range(5) &  IntersectPoint(3) <= XYZ_Range(6)];
            %                 %                 else
            %                 for K = 1:obj.Obstacles(OI).NumSurfaces
            %                     PointToSurfaceDist = obj.Obstacles(OI).ImplicitEqu{K}(IntersectPoint(1),IntersectPoint(2),IntersectPoint(3));
            %                     if obj.Obstacles(OI).ObstacleTol >= abs(PointToSurfaceDist)
            %                         PointToSurfaceDist = 0;
            %                     end
            %                     Sign = sign(PointToSurfaceDist);
            %                     if Sign == 0
            %                         Sign = obj.Obstacles(OI).SurfacesDirection(K);
            %                     end
            %                     Surfacesign(K) = Sign;
            %                 end
            %
            %
            %                 if all(Surfacesign == obj.Obstacles(OI).SurfacesDirection)
            %                     isInHull(i,:) = true;
            %                 else
            %                     isInHull(i,:) = false;
            %                 end
            %             end
            %             end
            %             toc
            
        end
        %%
        
        function isInfeasible = VerifyIntersection(obj,OB,OA,OI,SI)
            % check start point have intersection
            %             FoGu_coeff = FoGuT(obj.OB_0(CSI,:)',obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
            FoGu_coeff = FoGuT(OB,OA,obj.Obstacles(OI).SurfaceCoeffs{SI});
            t_roots = obj.polyRootsTol(FoGu_coeff,obj.root_tol);
            t_roots(t_roots > 1) = [];t_roots(t_roots < 0) = [];
            if ~isempty(t_roots)
                IntersectPoint = (OB - OA).*t_roots' + OA;
                isInHull = obj.DectectPointInObstacle(OI,IntersectPoint);
                t_roots = (t_roots(isInHull));
                if ~isempty(t_roots)
                    isInfeasible = true;
                else
                    isInfeasible = false;
                end
            else
                isInfeasible = false;
            end
            
        end
        
        function DetectStartEndIntersection(obj,CSI,OI,SI,u_range)
            % check start point have intersection
            %             FoGu_coeff = FoGuT(obj.OB_0(CSI,:)',obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
            FoGu_coeff = FoGuT(obj.OA_0(CSI,:)',obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI});
            t_roots = obj.polyRootsTol(FoGu_coeff,obj.root_tol);
            t_roots(t_roots > 1) = [];t_roots(t_roots < 0) = [];
            if ~isempty(t_roots)
                IntersectPoint = (obj.OA_0(CSI,:)' - obj.OB(:,CSI)).*t_roots' + obj.OB(:,CSI);
                %             isInHull = inhull(IntersectPoint',obj.Obstacles(OI).ObstacleHull.SimplifiedVertices,[],1e-3);
                isInHull = obj.DectectPointInObstacle(OI,IntersectPoint);
                t_roots = (t_roots(isInHull));
                if ~isempty(t_roots)
                    for TI = 1:numel(t_roots)
                        obj.NumSol = obj.NumSol + 1;
                        obj.CableSegmentInterRange(CSI).u(obj.NumSol) = u_range(1);
                        obj.CableSegmentInterRange(CSI).v(obj.NumSol) = t_roots(TI);
                        obj.CableSegmentInterRange(CSI).ObstacleNum(obj.NumSol) = OI;
                        obj.CableSegmentInterRange(CSI).SurfaceNum(obj.NumSol) = SI;
                    end
                end
            end
            % check end point have intersection
            %             FoGu_coeff = FoGuT(obj.OB_1(CSI,:)',obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
            
            FoGu_coeff = FoGuT(obj.OA_1(CSI,:)',obj.OB(:,CSI),obj.Obstacles(OI).SurfaceCoeffs{SI});
            t_roots = obj.polyRootsTol(FoGu_coeff,obj.root_tol);
            t_roots(t_roots > 1) = [];t_roots(t_roots < 0) = [];
            if ~isempty(t_roots)
                IntersectPoint = (obj.OA_1(CSI,:)' - obj.OB(:,CSI)).*t_roots' + obj.OB(:,CSI);
                isInHull = obj.DectectPointInObstacle(OI,IntersectPoint);
                %             isInHull = inhull(IntersectPoint',obj.Obstacles(OI).ObstacleHull.SimplifiedVertices,[],1e-3);
                t_roots = (t_roots(isInHull));
                if ~isempty(t_roots)
                    for TI = 1:numel(t_roots)
                        obj.NumSol = obj.NumSol + 1;
                        obj.CableSegmentInterRange(CSI).u(obj.NumSol) = u_range(end);
                        obj.CableSegmentInterRange(CSI).v(obj.NumSol) = t_roots(TI);
                        obj.CableSegmentInterRange(CSI).ObstacleNum(obj.NumSol) = OI;
                        obj.CableSegmentInterRange(CSI).SurfaceNum(obj.NumSol) = SI;
                    end
                end
            end
        end
        
        %% function to find intersection between OB_u and Obstacles surface
        function CurveCutSurface(obj,CSI,OI,SI,u_range)
            %             seg_surf_ind = [];
            
            if obj.SweptSurfaceDegree == 1
                FoGu_coeff =  FoGuT(obj.OA_1(CSI,:)',obj.OA_0(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
            else
                FoGu_coeff = FoGuO(obj.OB_u{CSI},obj.Obstacles(OI).SurfaceCoeffs{SI});
            end
            
            u_roots = obj.polyRootsTol(FoGu_coeff,obj.root_tol);
            u_roots(u_roots>u_range(2)) = [];
            u_roots(u_roots<u_range(1)) = [];
            if ~isempty(u_roots)
                for UI = 1:numel(u_roots)
                    obj.NumSol = obj.NumSol + 1;
                    obj.CableSegmentInterRange(CSI).u(obj.NumSol) = u_roots(UI);
                    obj.CableSegmentInterRange(CSI).v(obj.NumSol) = 1;
                    obj.CableSegmentInterRange(CSI).ObstacleNum(obj.NumSol) = OI;
                    obj.CableSegmentInterRange(CSI).SurfaceNum(obj.NumSol) = SI;
                    
                end
            end
            
        end
        
        %% function to find intersection between boundary of implicit surface to cable surface
        function SegmentIntersection_Gxyz(obj,ID,CSI,OI,BI,u_range)
            P2SIndex = obj.Obstacles(OI).P2SIndex(obj.Obstacles(OI).P2SIndex(:,1) == BI,2:end);
            IntersectPoint = [];
            t_roots = []; t_ans = [];
            t_range = obj.Obstacles(OI).ParametricRange{BI};
            ParametricDegree = obj.Obstacles(OI).ParametricDeg(BI);
            
            % determine the intersection points
            if obj.Obstacles(OI).ParametricDeg(BI) >=3
                obj.FittingForHighDegBoundary = 1;
            else
                obj.FittingForHighDegBoundary = 0;
            end
            
            if obj.FittingForHighDegBoundary == 0
                if ParametricDegree == 1
                    N = obj.Obstacles(OI).ParametricCoeffs{BI}.N;
                    D = obj.Obstacles(OI).ParametricCoeffs{BI}.D;
                    
                    Goft_coeff = GoftD1(N./D(:,end),obj.G_coeffs(CSI,:)');
                elseif ParametricDegree == 2
                    D = obj.Obstacles(OI).ParametricCoeffs{BI}.D;
                    D(:,[1,2]) = [];
                    N = obj.Obstacles(OI).ParametricCoeffs{BI}.N;
                    N(:,[1,2]) = [];
                    Goft_coeff = GoftD2(D,N,obj.G_coeffs(CSI,:)');
                elseif ParametricDegree == 3
                    % analtycial method
                    D = obj.Obstacles(OI).ParametricCoeffs{BI}.D;
                    D(:,1) = [];
                    N = obj.Obstacles(OI).ParametricCoeffs{BI}.N;
                    N(:,1) = [];
                    Goft_coeff = GoftD3(D,N,obj.G_coeffs(CSI,:)');
                    
                elseif ParametricDegree == 4
                    D = obj.Obstacles(OI).ParametricCoeffs{BI}.D;
                    N = obj.Obstacles(OI).ParametricCoeffs{BI}.N;
                    Goft_coeff = GoftD4(D,N,obj.G_coeffs(CSI,:)');
                    
                else % add your own degree of surface
                    error('Please generate the close form solution')
                end
                
                t_roots = obj.polyRootsTol(Goft_coeff,obj.root_tol);
                
            else
                % fitting method
                t_step = linspace(obj.Obstacles(OI).ParametricRange{BI}(1),...
                    obj.Obstacles(OI).ParametricRange{BI}(2),30);
                for i = 1:30
                    XYZ = obj.Obstacles(OI).ParametricEqu{BI}(t_step(i));
                    DistData(i) = obj.G_xyz{CSI}(XYZ(1),XYZ(2),XYZ(3));
                end
                [Goft_coeff_fit,~,~]= fit(t_step',DistData','cubicinterp');
                t_roots = unique(fnzeros(Goft_coeff_fit.p));
            end
            t_roots(t_roots > t_range(2)) = [];t_roots(t_roots < t_range(1)) = [];
            
            if ~isempty(t_roots)
                t_ans = [t_ans;t_roots];
                for j = 1:size(t_roots,1)
                    IntersectPoint = [IntersectPoint,obj.Obstacles(OI).ParametricEqu{BI}(t_roots(j))];
                    %                     IntersectSegmentID = [IntersectSegmentID; [t_roots(j),CSI]];
                end
                
            end
            
            % determine the u,v
            for i = 1:size(t_ans,1)
                [u_roots,v_roots] = obj.Finduv(obj.OA_u{ID},obj.OB(:,CSI),IntersectPoint(:,i));
                
                
                if ~isempty(u_roots) && u_roots <= u_range(2) &&  u_roots >= u_range(1) &&...
                        v_roots <= 1 &&  v_roots >= 0
                    
                    for UI = 1:numel(u_roots)
                        obj.NumSol = obj.NumSol + 1;
                        obj.CableSegmentInterRange(CSI).u(obj.NumSol) = u_roots(UI);
                        obj.CableSegmentInterRange(CSI).v(obj.NumSol) = v_roots(UI);
                        obj.CableSegmentInterRange(CSI).ObstacleNum(obj.NumSol) = OI;
                        obj.CableSegmentInterRange(CSI).SurfaceNum(obj.NumSol) = P2SIndex(1);
                    end
                    
                end
            end
            
        end
        
        function [cable_implicit_coeff,cable_implicit_fun,OA_u] = GetGxyz(obj,reconfig_element)
            v = zeros(obj.numCables,10);
            
            for i = 1:reconfig_element.Element_Count
                if reconfig_element.SweptSurfaceDegree(i) == 1
                    sample_size = 2;
                elseif reconfig_element.SweptSurfaceDegree(i) == 2
                    sample_size = 7;
                else
                    error('Only Support 1 and 2 degree surfaces currently...')
                end
                
                var_range = reconfig_element.Info(i).VariableRange;
                u_sample = linspace(var_range(1),var_range(end),sample_size);
                
                for j = 1:sample_size
                    surface_data{i}(:,j) = reconfig_element.Info(i).Path(u_sample(j));
                end
                
                
                for j = 1:3
                    OA_u{i}(j,:) = GeneralMathOperations.PolynomialFit(u_sample', surface_data{i}(j,:)', reconfig_element.SweptSurfaceDegree(i))';
                end
                surface_data{i}(:,end+1) = obj.OB(:,reconfig_element.Reconfig_Cable_ID(i));
                
                if reconfig_element.SweptSurfaceDegree(i) == 1
                    v(reconfig_element.Reconfig_Cable_ID(i),7:end) = obj.Plane_fit(surface_data{i}');
                    cable_implicit_fun{i} =@(x,y,z) v(reconfig_element.Reconfig_Cable_ID(i),7).*x +  v(reconfig_element.Reconfig_Cable_ID(i),8).*y +  v(reconfig_element.Reconfig_Cable_ID(i),9).*z +  v(reconfig_element.Reconfig_Cable_ID(i),10);
                else
                    
                    [v(reconfig_element.Reconfig_Cable_ID(i),:),center(i,:)] = obj.Cone_fit(surface_data{i}');
                    cable_implicit_fun{i} =@(x,y,z) v(reconfig_element.Reconfig_Cable_ID(i),1).*x.^2 + v(reconfig_element.Reconfig_Cable_ID(i),2).*y.^2 + v(reconfig_element.Reconfig_Cable_ID(i),3).*z.^2 + ...
                        v(reconfig_element.Reconfig_Cable_ID(i),4).*x.*y + v(reconfig_element.Reconfig_Cable_ID(i),5).*x.*z + v(reconfig_element.Reconfig_Cable_ID(i),6).*y.*z + v(reconfig_element.Reconfig_Cable_ID(i),7).*x + v(reconfig_element.Reconfig_Cable_ID(i),8).*y +...
                        v(reconfig_element.Reconfig_Cable_ID(i),9).*z + v(reconfig_element.Reconfig_Cable_ID(i),10);
                    
                end
                
            end
            cable_implicit_coeff = v;
            %             %fit Gxyz plane equation
            %             surface_data = [OB,OA]';
            %             vec_1 = surface_data(2,:) - surface_data(1,:);
            %             vec_2 = surface_data(end,:) - surface_data(1,:);
            %             v(1) = vec_1(2)*vec_2(3) - vec_1(3)*vec_2(2);
            %             v(2) = vec_1(3)*vec_2(1) - vec_1(1)*vec_2(3);
            %             v(3) = vec_1(1)*vec_2(2) - vec_1(2)*vec_2(1);
            %             v(4) = -v(1:3)*surface_data(1,:)';
            %             Gxyz_coeff = v;
            %             %             Gxyz = @(x,y,z) v*[x;y;z;1];
        end
        %% function to calculate the u,v where points on the G(u,v)
        
        function [u,v] = Finduv(obj,OA_u,OB,P)
            
            if size(OA_u,2) == 2
                
                A0 = OA_u*[0;1];
                A1 = OA_u*[1;1];
                
                StartPt = [OB,A0]';
                EndPt   = [P ,A1]';
                PAB = obj.LineIntersection3D(StartPt,EndPt)';
                u = (PAB - A0)'*(A1-A0)/norm(A1-A0);
                v = norm(P - OB)/norm(PAB - OB);
                
                %                 ((A1 - A0)/norm(A1-A0)*u +A0- OB)*v + OB;
            elseif size(OA_u,2) == 3
                
                a1 = OA_u(1,1); b1 = OA_u(1,2); c1 = OA_u(1,3);  d1 = OB(1);
                a2 = OA_u(2,1); b2 = OA_u(2,2); c2 = OA_u(2,3);  d2 = OB(2);
                %             a3 = OB_u(3,1); b3 = OB_u(3,2); c3 = OB_u(3,3);  d3 = OA(3);
                x = P(1); y = P(2); z = P(3);
                u = [-(b1*d2 - b2*d1 + b2*x - b1*y + (b1^2*d2^2 + b2^2*d1^2 + b2^2*x^2 + b1^2*y^2 - 4*d2^2*x^2 - 4*d1^2*y^2 - 4*a1*c1*d2^2 - 4*a2*c2*d1^2 - 4*a2*c2*x^2 - 4*a1*c1*y^2 + 4*a1*d2^2*x + 4*a2*d2*x^2 + 4*a1*d1*y^2 + 4*a2*d1^2*y - 2*b2^2*d1*x + 4*c1*d2^2*x - 2*b1^2*d2*y + 4*c2*d2*x^2 + 4*c1*d1*y^2 + 4*c2*d1^2*y + 8*a1*c1*d2*y - 4*a1*c2*d1*y - 4*a2*c1*d1*y + 2*b1*b2*d1*y - 4*a2*d1*d2*x - 4*a1*d1*d2*y - 4*c2*d1*d2*x - 4*c1*d1*d2*y + 4*a1*c2*x*y + 4*a2*c1*x*y - 2*b1*b2*x*y - 4*a1*d2*x*y - 4*a2*d1*x*y - 4*c1*d2*x*y - 4*c2*d1*x*y + 8*d1*d2*x*y + 4*a1*c2*d1*d2 + 4*a2*c1*d1*d2 - 2*b1*b2*d1*d2 - 4*a1*c2*d2*x - 4*a2*c1*d2*x + 8*a2*c2*d1*x + 2*b1*b2*d2*x)^(1/2))/(2*(a1*d2 - a2*d1 + a2*x - a1*y - d2*x + d1*y));
                    (b2*d1 - b1*d2 - b2*x + b1*y + (b1^2*d2^2 + b2^2*d1^2 + b2^2*x^2 + b1^2*y^2 - 4*d2^2*x^2 - 4*d1^2*y^2 - 4*a1*c1*d2^2 - 4*a2*c2*d1^2 - 4*a2*c2*x^2 - 4*a1*c1*y^2 + 4*a1*d2^2*x + 4*a2*d2*x^2 + 4*a1*d1*y^2 + 4*a2*d1^2*y - 2*b2^2*d1*x + 4*c1*d2^2*x - 2*b1^2*d2*y + 4*c2*d2*x^2 + 4*c1*d1*y^2 + 4*c2*d1^2*y + 8*a1*c1*d2*y - 4*a1*c2*d1*y - 4*a2*c1*d1*y + 2*b1*b2*d1*y - 4*a2*d1*d2*x - 4*a1*d1*d2*y - 4*c2*d1*d2*x - 4*c1*d1*d2*y + 4*a1*c2*x*y + 4*a2*c1*x*y - 2*b1*b2*x*y - 4*a1*d2*x*y - 4*a2*d1*x*y - 4*c1*d2*x*y - 4*c2*d1*x*y + 8*d1*d2*x*y + 4*a1*c2*d1*d2 + 4*a2*c1*d1*d2 - 2*b1*b2*d1*d2 - 4*a1*c2*d2*x - 4*a2*c1*d2*x + 8*a2*c2*d1*x + 2*b1*b2*d2*x)^(1/2))/(2*(a1*d2 - a2*d1 + a2*x - a1*y - d2*x + d1*y))];
                %             u2 = [-(b1*d3 - b3*d1 + b3*x - b1*z + (b1^2*d3^2 + b3^2*d1^2 + b3^2*x^2 + b1^2*z^2 - 4*d3^2*x^2 - 4*d1^2*z^2 - 4*a1*c1*d3^2 - 4*a3*c3*d1^2 - 4*a3*c3*x^2 + 4*a1*d3^2*x + 4*a3*d3*x^2 - 4*a1*c1*z^2 - 2*b3^2*d1*x + 4*a1*d1*z^2 + 4*a3*d1^2*z + 4*c1*d3^2*x + 4*c3*d3*x^2 - 2*b1^2*d3*z + 4*c1*d1*z^2 + 4*c3*d1^2*z - 4*a3*d1*d3*x + 8*a1*c1*d3*z - 4*a1*c3*d1*z - 4*a3*c1*d1*z + 2*b1*b3*d1*z - 4*a1*d1*d3*z - 4*c3*d1*d3*x - 4*c1*d1*d3*z + 4*a1*c3*x*z + 4*a3*c1*x*z - 2*b1*b3*x*z - 4*a1*d3*x*z - 4*a3*d1*x*z - 4*c1*d3*x*z - 4*c3*d1*x*z + 8*d1*d3*x*z + 4*a1*c3*d1*d3 + 4*a3*c1*d1*d3 - 2*b1*b3*d1*d3 - 4*a1*c3*d3*x - 4*a3*c1*d3*x + 8*a3*c3*d1*x + 2*b1*b3*d3*x)^(1/2))/(2*(a1*d3 - a3*d1 + a3*x - a1*z - d3*x + d1*z));
                %                 (b3*d1 - b1*d3 - b3*x + b1*z + (b1^2*d3^2 + b3^2*d1^2 + b3^2*x^2 + b1^2*z^2 - 4*d3^2*x^2 - 4*d1^2*z^2 - 4*a1*c1*d3^2 - 4*a3*c3*d1^2 - 4*a3*c3*x^2 + 4*a1*d3^2*x + 4*a3*d3*x^2 - 4*a1*c1*z^2 - 2*b3^2*d1*x + 4*a1*d1*z^2 + 4*a3*d1^2*z + 4*c1*d3^2*x + 4*c3*d3*x^2 - 2*b1^2*d3*z + 4*c1*d1*z^2 + 4*c3*d1^2*z - 4*a3*d1*d3*x + 8*a1*c1*d3*z - 4*a1*c3*d1*z - 4*a3*c1*d1*z + 2*b1*b3*d1*z - 4*a1*d1*d3*z - 4*c3*d1*d3*x - 4*c1*d1*d3*z + 4*a1*c3*x*z + 4*a3*c1*x*z - 2*b1*b3*x*z - 4*a1*d3*x*z - 4*a3*d1*x*z - 4*c1*d3*x*z - 4*c3*d1*x*z + 8*d1*d3*x*z + 4*a1*c3*d1*d3 + 4*a3*c1*d1*d3 - 2*b1*b3*d1*d3 - 4*a1*c3*d3*x - 4*a3*c1*d3*x + 8*a3*c3*d1*x + 2*b1*b3*d3*x)^(1/2))/(2*(a1*d3 - a3*d1 + a3*x - a1*z - d3*x + d1*z))];
                v = -((u.^2 + 1)*(d1 - x))./(c1 - d1 + b1.*u + a1*u.^2 - d1*u.^2);
                OB = [polyval(OA_u(1,:),u),...
                    polyval(OA_u(2,:),u),...
                    polyval(OA_u(3,:),u)]./(1+u.^2);
                m1 = [OB - OB]'./vecnorm([OB - OB]');
                m2 = (P-OB')/ norm(P-OB');
                cor_ind = all(round(m1 - m2,obj.ROUNDING_DIGIT)==0);
                
                %
                u = u(cor_ind);
                v = v(cor_ind);
            end
        end
        
        %% function to find 2 line intersection point in 3D
        function [P_intersect,distances] = LineIntersection3D(~,PA,PB)
            % Find intersection point of lines in 3D space, in the least squares sense.
            % PA :          Nx3-matrix containing starting point of N lines
            % PB :          Nx3-matrix containing end point of N lines
            % P_Intersect : Best intersection point of the N lines, in least squares sense.
            % distances   : Distances from intersection point to the input lines
            % Anders Eikenes, 2012
            Si = PB - PA; %N lines described as vectors
            ni = Si ./ (sqrt(sum(Si.^2,2))*ones(1,3)); %Normalize vectors
            nx = ni(:,1); ny = ni(:,2); nz = ni(:,3);
            SXX = sum(nx.^2-1);
            SYY = sum(ny.^2-1);
            SZZ = sum(nz.^2-1);
            SXY = sum(nx.*ny);
            SXZ = sum(nx.*nz);
            SYZ = sum(ny.*nz);
            S = [SXX SXY SXZ;SXY SYY SYZ;SXZ SYZ SZZ];
            CX  = sum(PA(:,1).*(nx.^2-1) + PA(:,2).*(nx.*ny)  + PA(:,3).*(nx.*nz));
            CY  = sum(PA(:,1).*(nx.*ny)  + PA(:,2).*(ny.^2-1) + PA(:,3).*(ny.*nz));
            CZ  = sum(PA(:,1).*(nx.*nz)  + PA(:,2).*(ny.*nz)  + PA(:,3).*(nz.^2-1));
            C   = [CX;CY;CZ];
            P_intersect = (S\C)';
            if nargout>1
                N = size(PA,1);
                distances=zeros(N,1);
                for segment_index=1:N %This is faster:
                    ui=(P_intersect-PA(segment_index,:))*Si(segment_index,:)'/(Si(segment_index,:)*Si(segment_index,:)');
                    distances(segment_index)=norm(P_intersect-PA(segment_index,:)-ui*Si(segment_index,:));
                end
                %for segment_index=1:N %http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html:
                %    distances(segment_index) = norm(cross(P_intersect-PA(segment_index,:),P_intersect-PB(segment_index,:))) / norm(Si(segment_index,:));
                %end
            end
        end
    end
    methods (Static)
        function P_intersected = FxyzGuv(OB,OA,surfaceCoeffs,surfaceDegree)
            %% FxyzGuv
            P_intersected = [];
            if surfaceDegree == 4
                H0 = F_uv_H0_coeffs(OB,OA,surfaceCoeffs);
                H1 = F_uv_H1_coeffs(OB,OA,surfaceCoeffs);
                H2 = F_uv_H2_coeffs(OB,OA,surfaceCoeffs);
                H3 = F_uv_H3_coeffs(OB,OA,surfaceCoeffs);
                H4 = F_uv_H4_coeffs(OB,OA,surfaceCoeffs);
                discrminant = Discriminant_deg_4(H4,H3,H2,H1,H0);
            elseif surfaceDegree == 3
                H0 = F_uv_H0_coeffs(OB,OA,surfaceCoeffs);
                H1 = F_uv_H1_coeffs(OB,OA,surfaceCoeffs);
                H2 = F_uv_H2_coeffs(OB,OA,surfaceCoeffs);
                H3 = F_uv_H3_coeffs(OB,OA,surfaceCoeffs);
                discrminant = Discriminant_deg_3(H3,H2,H1,H0);
            elseif surfaceDegree == 2
                H0 = F_uv_H0_coeffs(OB,OA,surfaceCoeffs);
                H1 = F_uv_H1_coeffs(OB,OA,surfaceCoeffs);
                H2 = F_uv_H2_coeffs(OB,OA,surfaceCoeffs);
                discrminant = Discriminant_deg_2(H2,H1,H0);
            else
                return
            end
            
            u = round(roots(discrminant),9);
            u = unique(u(imag(u) == 0));
            u(u>1) = [];
            u(u<0) = [];
            if ~isempty(u)
                for i = 1:size(u,1)
                    v_coeffs = F_uv_v_coeffs(OB,OA,surfaceCoeffs,u(i));
                    v = roots(v_coeffs);
                    tmp_v = round(v,3);
                    [tmp_v,index] = unique(tmp_v(imag(tmp_v) == 0));
                    v = v(index(find(tmp_v<=1 & tmp_v>=0)));
                    v = real(v);
                    %                     v(v>1) = [];
                    %                     v(v<0) = [];
                    
                    %                     v = round(roots(v_coeffs),5);
                    %                     v = unique(v(imag(v) == 0));
                    
                    if ~isempty(v)
                        for j = 1:size(v,1)
                            P_intersected = [P_intersected,...
                                (((OA(:,1) - OA(:,2))*u(i) +  OA(:,2)) - OB) * v(j) + OB];
                        end
                        %                     LOA =@(u) (OA(:,1) - OA(:,2)).*u + OA(:,2);
                        %                     Guv_fun = @(u,v) OB + (LOA(u) - OB).*v;
                        %                     P_j = Guv_fun(u(1),v_roots(1))
                        %                     scatter3(P_j(1),P_j(2),P_j(3),'filled')
                        
                    end
                end
            end
        end
        
        function P_intersected = FxyzEt(OB,OA,surfaceCoeffs)
            P_intersected = [];
            E{1} = [OB,OA(:,1)];
            E{2} = [OB,OA(:,2)];
            E{3} = OA;
            %             line(E{3}(1,:),E{3}(2,:),E{3}(3,:));
            %             line(E{2}(1,:),E{2}(2,:),E{2}(3,:));
            %             line(E{1}(1,:),E{1}(2,:),E{1}(3,:));
            %
            for i = 1:3
                F_G_t =  F_G_t_coeffs(E{i},surfaceCoeffs);
                t = round(roots(F_G_t),9);
                t = unique(t(imag(t) == 0));
                t(t>1) = [];
                t(t<0) = [];
                if ~isempty(t)
                    P_intersected = [P_intersected,(E{i}(:,1) - E{i}(:,2)) .* t' + E{i}(:,2)];
                end
            end
            
            
            
        end
        
        %         function P_intersected = GxyzTrigonometric(Gxyz,boundaryEqu)
        %             syms t
        %             P_intersected = [];
        %             X = [boundaryEqu(t);1];
        %             xsol = round(double(solve(Gxyz*X == 0,t)),9);
        %             for i = 1:numel(xsol)
        %             if isreal(xsol(i))
        %                 P_intersected = [P_intersected,boundaryEqu(xsol(i))];
        %             end
        %             end
        %         end
        
        function P_intersected = GxyzFt(Gxyz,boundaryEquCoeff,boundaryEqu)
            P_intersected = [];
            G_F_t = G_F_t_coeffs(Gxyz,boundaryEquCoeff);
            t = round(roots(G_F_t),9);
            t = unique(t(imag(t) == 0));
            t(t>1) = [];
            t(t<0) = [];
            if ~isempty(t)
                for i = 1:max(size(t))
                    P_intersected = [P_intersected,boundaryEqu(t(i))];
                end
            end
        end
        
        
        function [OA_u, P_out, P_surf_index] = sortIntersectedPoints(OB,OA,P_intersected,P_index,surf_equ, surf_dir,obstacle_hull)
            [P_intersected,ind] = unique(P_intersected','rows','stable');
            P_index = P_index(ind,:); P_intersected = P_intersected';
            OA_u_tmp = []; P_out = []; u_compare = []; OA_u =[]; P_surf_index = []; u = [];v = [];
            for i = 1:size(P_intersected,2)
                [tmp_u,tmp_v] =  G_uv(OA,OB,P_intersected(:,i));
                P_side = [];
                
                if ~isempty(tmp_u)
                    if inhull(P_intersected(:,i)',obstacle_hull.Vertices)
                        %                         u_compare = [u_compare,tmp_u];
                        P_surf_index = [P_surf_index;P_index(i,:)];
                        OA_u_tmp = [OA_u_tmp,(OA(:,1) - OA(:,2)) * tmp_u + OA(:,2)];
                        P_out = [P_out, P_intersected(:,i)];
                        u = [u,tmp_u];v = [v,tmp_v];
                    end
                    %%%%%%% need to deal with this, point inside where
                    %                     for j = 1:size(surf_equ,2)
                    %                         P_side(j) = sign(round(surf_equ{j}(P_intersected(1,i),P_intersected(2,i),P_intersected(3,i)),3));
                    %                         if P_side(j) == 0
                    %                             P_side(j) = surf_dir(j);
                    %                         end
                    %                     end
                    %
                    %                     if all(P_side == surf_dir)
                    %                         u_compare = [u_compare,tmp_u];
                    %                         P_surf_index = [P_surf_index;P_index(i,:)];
                    %                         OA_u_tmp = [OA_u_tmp,(OA(:,1) - OA(:,2)) * tmp_u + OA(:,2)];
                    %                         P_out = [P_out, P_intersected(:,i)];
                    %                         u = [u,tmp_u];v = [v,tmp_v];
                    %                     end
                    
                end
            end
            
            [u,index_order] = unique(u','rows','sorted');
            OA_u = OA_u_tmp(:,index_order);
            
            P_surf_index = P_surf_index(index_order,:);
        end
        
        %% function to fit the cable swept surface into implicit form for orientation
        function [ v,center ] = Cone_fit(surface_data )
            % Yury (2022). Ellipsoid fit (https://www.mathworks.com/matlabcentral/fileexchange/24693-ellipsoid-fit), MATLAB Central File Exchange. Retrieved June 27, 2022.
            % * center    -  ellispoid or other conic center coordinates [xc; yc; zc]
            % * radii     -  ellipsoid or other conic radii [a; b; c]
            % * evecs     -  the radii directions as columns of the 3x3 matrix
            % * v         -  the 10 parameters describing the ellipsoid / conic algebraically:
            %             %                Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz + J = 0
            %         now       Ax^2 + By^2 + Cz^2 + Dxy + Exz + Fyz + Gx + Hy + Iz + J = 0
            % Dv = d2 - > D'Dv = D'd2 -> v = (D'D)\(D'd2)
            x = surface_data( :, 1 );
            y = surface_data( :, 2 );
            z = surface_data( :, 3 );
            D = [ x .* x + y .* y - 2 * z .* z, ...
                x .* x + z .* z - 2 * y .* y, ...
                2 * x .* y, ...
                2 * x .* z, ...
                2 * y .* z, ...
                2 * x, ...
                2 * y, ...
                2 * z, ...
                1 + 0 * x ];
            % solve the normal system of equations
            d2 = x .* x + y .* y + z .* z; % the RHS of the llsq problem (y's)
            u = ( D' * D ) \ ( D' * d2 );  % solution to the normal equations
            v(1) = u(1) +     u(2) - 1;
            v(2) = u(1) - 2 * u(2) - 1;
            v(3) = u(2) - 2 * u(1) - 1;
            v( 4 : 10 ) = u( 3 : 9 );
            v = v';
            
            % form the algebraic form of the ellipsoid
            A = [ v(1) v(4) v(5) v(7); ...
                v(4) v(2) v(6) v(8); ...
                v(5) v(6) v(3) v(9); ...
                v(7) v(8) v(9) v(10) ];
            % find the center of the ellipsoid
            center = -A( 1:3, 1:3 ) \ v( 7:9 );
            if abs( v(end) ) > 1e-6
                v = -v / v(end); % normalize to the more conventional form with constant term = -1
            else
                v = -sign( v(end) ) * v;
            end
            v(4:9) = 2*v(4:9);
        end
        
        %% function to fit the cable swept surface into implicit form for translation
        function v = Plane_fit(surface_data)
            % ref: Adrien Leygue (2022). Plane fit (https://www.mathworks.com/matlabcentral/fileexchange/43305-plane-fit), MATLAB Central File Exchange. Retrieved June 21, 2022
            
            p = mean(surface_data,1);
            R = bsxfun(@minus,surface_data,p);
            [V,D] = eig(R'*R);
            
            v = V(:,1);
            v(4) = -v'*p';
            
        end
        
        %% function to find the interval intersection
        function out = range_intersection(first,second)
            % Purpose: Range/interval intersection
            %
            % A and B two ranges of closed intervals written
            % as vectors [lowerbound1 upperbound1 lowerbound2 upperbound2]
            % or as matrix [lowerbound1, lowerbound2, lowerboundn;
            %               upperbound1, upperbound2, upperboundn]
            % A and B have to be sorted in ascending order
            %
            % out is the mathematical intersection A n B
            %
            %
            % EXAMPLE USAGE:
            %   >> out=range_intersection([1 3 5 9],[2 9])
            %   	out =  [2 3 5 9]
            %   >> out=range_intersection([40 44 55 58], [42 49 50 52])
            %   	out =  [42 44]
            %
            % Author: Xavier Beudaert <xavier.beudaert@gmail.com>
            % Original: 10-June-2011
            % Major modification and bug fixing 30-May-2012
            
            % Allocate, as we don't know yet the size, we assume the largest case
            out1(1:(numel(second)+(numel(first)-2)))=0;
            
            k=1;
            while isempty(first)==0 && isempty(second)==0
                % make sure that first is ahead second
                if first(1)>second(1)
                    temp=second;
                    second=first;
                    first=temp;
                end
                
                if first(2)<second(1)
                    first=first(3:end);
                    continue;
                elseif first(2)==second(1)
                    out1(k)=second(1);
                    out1(k+1)=second(1);
                    k=k+2;
                    
                    first=first(3:end);
                    continue;
                else
                    if first(2)==second(2)
                        out1(k)=second(1);
                        out1(k+1)=second(2);
                        k=k+2;
                        
                        first=first(3:end);
                        second=second(3:end);
                        
                    elseif first(2)<second(2)
                        out1(k)=second(1);
                        out1(k+1)=first(2);
                        k=k+2;
                        
                        first=first(3:end);
                    else
                        out1(k)=second(1);
                        out1(k+1)=second(2);
                        k=k+2;
                        
                        second=second(3:end);
                    end
                end
            end
            
            % Remove the tails
            out=out1(1:k-1);
        end
        
        function out = FindFeasibleRange(first,second)
            if isempty(second)
                out = first;
                return
            end
            a = first(1); b = first(end);
            c = second(1); d = second(end);
            
            if a >= d || c >= b
                out = [a,b];
            elseif a >= c && b > d
                out = [d,b];
            elseif c > a && b > d
                out = [a,c;d,b];
            elseif c > a && d >= b
                out = [a,c];
            elseif a == c && b == d
                out = [];
            end
            
            
        end
        
        function real_ans = polyRootsTol(polyfun,root_tol)
            if any(isnan(polyfun))
                polyfun = 0;
            end
            roots_ans = roots(polyfun);
            real_ans = [];
            for j = 1:numel(roots_ans)
                u_i = roots_ans(j);
                if abs(imag(u_i)) < root_tol
                    u_i = real(u_i);
                else
                    u_i = [];
                end
                real_ans = [real_ans;u_i];
            end
        end
    end
end