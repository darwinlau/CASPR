% Class to compute whether a ray inside the obstacles from quadtratic and
% quatic degree s
%
% Author        : Paul Cheng
% Created       : 2020
% Description   :
% Cheng, H. H., & Lau, D. (2022). Ray-based cable and obstacle interference-free workspace for cable-driven parallel robots. Mechanism and Machine Theory, 172, 104782.

classdef InterferenceFreeRayConditionCableObstacle < WorkspaceRayConditionBase
    properties (Constant)
        ROUNDING_DIGIT = 4;
        OBSTACLE_DIST_TOL = 0.002; % the tol should be handle carefully for detection of inside the obstacle
        root_tol = 1e-4;
        % Type of workspace condition (WorkspaceConditionType enum)
        type = WorkspaceRayConditionType.INTERFERENCE_CABLE_SURF;
        
    end
    
    properties (SetAccess = protected)
        % Set constants
        TranslationDoFIndex;        % Array for the q of the joint (true if translation and false if rotation)
        isTranslationDoF            % indicate the current q if translation or no
        numDofs;                    % The number of dofs
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
        OB_u;
        OB_at_u;
        OB_0;
        OB_1;
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
        
        
    end
    
    methods
        % Constructor for interference free worksapce
        function w = InterferenceFreeRayConditionCableObstacle(model, min_ray_lengths, Obstacles)
            w@WorkspaceRayConditionBase(min_ray_lengths);
            w.TranslationDoFIndex = (model.bodyModel.q_dofType == DoFType.TRANSLATION);
            w.numDofs = model.numDofs;
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
            w.G_uv = @(u,v,OB,OA)(OB*[u.^2;u;1] ./ (1+u.^2) - OA)*v +  OA;
        end
        
        % Evaluate the interference free intervals
        function intervals =  evaluateFunction(obj, model, ws_ray)
            % Variable initialisation
            
            intervals = [];
            obj.Case1time = 0;obj.Case2time = 0;obj.Case3time = 0;obj.Verifytime = 0;obj.Case4time = 0;
            free_variable_index = ws_ray.freeVariableIndex;
            obj.isTranslationDoF = obj.TranslationDoFIndex(free_variable_index);
            
            q_begin = [ws_ray.fixedVariables(1:free_variable_index-1);ws_ray.freeVariableRange(1);ws_ray.fixedVariables(free_variable_index:end)];
            q_end = [ws_ray.fixedVariables(1:free_variable_index-1);ws_ray.freeVariableRange(2);ws_ray.fixedVariables(free_variable_index:end)];
            
            if obj.isTranslationDoF
                obj.u_range = [0 1];
            else
                obj.u_range = tan([q_begin(free_variable_index),q_end(free_variable_index)]/2);
            end
            
            if obj.needImplicitSweptSurface
                [obj.G_coeffs,obj.G_xyz] = obj.GetGxyz(model,q_begin,q_end,free_variable_index,obj.isTranslationDoF);
            end
            
            [obj.OB_u,~]  = obj.GetOB_u(model,q_begin,q_end,free_variable_index,obj.isTranslationDoF);
            
            
            [OA_i_begin,OB_i_begin] = obj.GetSegmentData(model,q_begin);
            [OA_i_end,OB_i_end] = obj.GetSegmentData(model,q_end);
            
            obj.OB_0 = OB_i_begin;
            obj.OB_1 = OB_i_end;
            
            for i = 1:obj.numSegment
                [obj.OA_i_hat(i,:),~] = obj.LineIntersection3D([OB_i_begin(i,:);OB_i_end(i,:)],[OA_i_begin(i,:);OA_i_end(i,:)]);
            end
            
            FeasibleRange = obj.u_range;
            for CSI = 1:obj.numSegment
                
                obj.CableSegmentInterRange(CSI).u = [];
                obj.CableSegmentInterRange(CSI).v = [];
                obj.CableSegmentInterRange(CSI).ObstacleNum = [];
                obj.CableSegmentInterRange(CSI).SurfaceNum = [];
                obj.NumSol = 0;
               
                for OI = 1:obj.NumObstacle
                    
                    for SI = 1:obj.Obstacles(OI).NumSurfaces
                        tic
                        obj.TangentToSurface(CSI,OI,SI);
                        obj.Case3time = obj.Case3time + toc;
                        tic
                        obj.CurveCutSurface(CSI,OI,SI);
                        obj.Case4time = obj.Case4time + toc;
                        tic
                        obj.DetectStartEndIntersection(CSI,OI,SI);
                        obj.Case2time = obj.Case2time + toc;
                    end
                    
                    tic
                    for BI = 1:obj.Obstacles(OI).NumBoundary
                        obj.SegmentIntersection_Gxyz(CSI,OI,BI);
                    end
                    obj.Case1time = obj.Case1time + toc;
                end
                
                tic
                u = obj.CableSegmentInterRange(CSI).u;
                v = obj.CableSegmentInterRange(CSI).v;
                if ~isempty(u)
                    
                    if obj.verifyInterior
                        if obj.isTranslationDoF
                            IntersectPoint = (obj.OB_u{CSI}*[u;ones(1,numel(u))] - obj.OA_i_hat(CSI,:)').*v + obj.OA_i_hat(CSI,:)'; % [x;y;z,x;y;z]
                        else
                            IntersectPoint = (obj.OB_u{CSI}*[u.^2;u;ones(1,numel(u))]./[1+u.^2] - obj.OA_i_hat(CSI,:)').*v + obj.OA_i_hat(CSI,:)';
                        end
                        
                        inHullu = [];
                        for OI = 1:obj.NumObstacle
                            isInHull = obj.DectectPointInObstacle(OI,IntersectPoint);
                            inHullu = [inHullu,u(isInHull)];
                            
                        end
                        
                        if ~isempty(inHullu)
                            FeasibleRangeSI = [];
                            last_is_feasible = 0;
                            u_unique = unique([inHullu,obj.u_range]);
                            for UI = 1:numel(u_unique) - 1
                                u_middle = 0.5*(u_unique(UI) + u_unique(UI+1));
                                OB_u_s = obj.OB_at_u{CSI}(u_middle);
                                OA_u_s = obj.OA_at_u{CSI}(u_middle);
                                
                                
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
                            FeasibleRangeSI = obj.u_range;
                        end
                        
                    else
                        
                        FeasibleRangeSI = obj.u_range;
                        if obj.isTranslationDoF
                            IntersectPoint = (obj.OB_u{CSI}*[u;ones(1,numel(u))] - obj.OA_i_hat(CSI,:)').*v + obj.OA_i_hat(CSI,:)'; % [x;y;z,x;y;z]
                        else
                            IntersectPoint = (obj.OB_u{CSI}*[u.^2;u;ones(1,numel(u))]./[1+u.^2] - obj.OA_i_hat(CSI,:)').*v + obj.OA_i_hat(CSI,:)';
                        end
                        
                        
                        for OI = 1:obj.NumObstacle
                            isInHull = obj.DectectPointInObstacle(OI,IntersectPoint);
                            inHullu = u(isInHull);
                            FeasibleRangeOI = obj.u_range;
                            if ~isempty(inHullu)
                                tmp_u = unique(inHullu);
                                InfeasibleRange = [tmp_u(1),tmp_u(end)];
                                tmp_CurrentFeasibleRange = obj.FindFeasibleRange(obj.u_range,InfeasibleRange)';
                                tmp_CurrentFeasibleRange = tmp_CurrentFeasibleRange(:);
                                CurrentFeasibleRange = tmp_CurrentFeasibleRange';
                                FeasibleRangeOI = obj.range_intersection(CurrentFeasibleRange,FeasibleRangeOI);
                                
                            end
                            FeasibleRangeSI = obj.range_intersection(FeasibleRangeSI,FeasibleRangeOI);
                        end
                    end
                    obj.Verifytime = obj.Verifytime + toc;
                    FeasibleRange = obj.range_intersection(FeasibleRange,FeasibleRangeSI);
                    
                end
            end
            if ~isempty(FeasibleRange)
                u = FeasibleRange;
                q_i = repmat(q_begin,1,numel(u));
                
                if obj.isTranslationDoF
                    q_i = (q_end - q_begin)*u + q_begin;
                else
                    q_i(free_variable_index,:) = 2*atan(u);
                end
                
                tmp_int = q_i(free_variable_index,:);
                odd_num = 1:2:numel(tmp_int);
                even_num = 2:2:numel(tmp_int);
                intervals(:,1) = tmp_int(odd_num);
                intervals(:,2) = tmp_int(even_num);
            else
                intervals = [];
            end
            obj.total_time =  [obj.Case1time,obj.Case2time, obj.Case3time,obj.Case4time,obj.Verifytime];
        end
        
        %%
        function isInHull = DectectPointInObstacle(obj,OI,Points)
            % POINTS - > N X 3
            % inhull approach
            %             tic
            %             isInHull = inhull(Points',obj.Obstacles(OI).ObstacleHull.SimplifiedVertices,[],1e-3);
            %             toc
            % analytical approach
            %             tic
            for i = 1:size(Points,2)
                IntersectPoint = Points(:,i);
                %                 if obj.Obstacles(OI).NumSurfaces == 1
                %                     % A lazy way for one surface obstacles =P
                %                     XYZ_Range = obj.Obstacles(OI).SurfacesXYZRange{1};
                %                    isInHull(i,:) = [IntersectPoint(1) >= XYZ_Range(1) &  IntersectPoint(1) <= XYZ_Range(2) & ...
                %                      IntersectPoint(2) >= XYZ_Range(3) &  IntersectPoint(2) <= XYZ_Range(4) & ...
                %                       IntersectPoint(3) >= XYZ_Range(5) &  IntersectPoint(3) <= XYZ_Range(6)];
                %                 else
                for K = 1:obj.Obstacles(OI).NumSurfaces
                    PointToSurfaceDist = obj.Obstacles(OI).ImplicitEqu{K}(IntersectPoint(1),IntersectPoint(2),IntersectPoint(3));
                    if obj.Obstacles(OI).ObstacleTol >= abs(PointToSurfaceDist)
                        PointToSurfaceDist = 0;
                    end
                    Sign = sign(PointToSurfaceDist);
                    if Sign == 0
                        Sign = obj.Obstacles(OI).SurfacesDirection(K);
                    end
                    Surfacesign(K) = Sign;
                end
                
                
                if all(Surfacesign == obj.Obstacles(OI).SurfacesDirection)
                    isInHull(i,:) = true;
                else
                    isInHull(i,:) = false;
                end
            end
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
        
        function DetectStartEndIntersection(obj,CSI,OI,SI)
            % check start point have intersection
            %             FoGu_coeff = FoGuT(obj.OB_0(CSI,:)',obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
            FoGu_coeff = FoGuT(obj.OB_0(CSI,:)',obj.OA_at_u{CSI}(obj.u_range(1)),obj.Obstacles(OI).SurfaceCoeffs{SI});
            t_roots = obj.polyRootsTol(FoGu_coeff,obj.root_tol);
            t_roots(t_roots > 1) = [];t_roots(t_roots < 0) = [];
            if ~isempty(t_roots)
                IntersectPoint = (obj.OB_0(CSI,:)' - obj.OA_i_hat(CSI,:)').*t_roots' + obj.OA_i_hat(CSI,:)';
                %             isInHull = inhull(IntersectPoint',obj.Obstacles(OI).ObstacleHull.SimplifiedVertices,[],1e-3);
                isInHull = obj.DectectPointInObstacle(OI,IntersectPoint);
                t_roots = (t_roots(isInHull));
                if ~isempty(t_roots)
                    for TI = 1:numel(t_roots)
                        obj.NumSol = obj.NumSol + 1;
                        obj.CableSegmentInterRange(CSI).u(obj.NumSol) = obj.u_range(1);
                        obj.CableSegmentInterRange(CSI).v(obj.NumSol) = t_roots(TI);
                        obj.CableSegmentInterRange(CSI).ObstacleNum(obj.NumSol) = OI;
                        obj.CableSegmentInterRange(CSI).SurfaceNum(obj.NumSol) = SI;
                    end
                end
            end
            % check end point have intersection
            %             FoGu_coeff = FoGuT(obj.OB_1(CSI,:)',obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
            FoGu_coeff = FoGuT(obj.OB_1(CSI,:)',obj.OA_at_u{CSI}(obj.u_range(2)),obj.Obstacles(OI).SurfaceCoeffs{SI});
            
            t_roots = obj.polyRootsTol(FoGu_coeff,obj.root_tol);
            t_roots(t_roots > 1) = [];t_roots(t_roots < 0) = [];
            if ~isempty(t_roots)
                IntersectPoint = (obj.OB_1(CSI,:)' - obj.OA_i_hat(CSI,:)').*t_roots' + obj.OA_i_hat(CSI,:)';
                isInHull = obj.DectectPointInObstacle(OI,IntersectPoint);
                %             isInHull = inhull(IntersectPoint',obj.Obstacles(OI).ObstacleHull.SimplifiedVertices,[],1e-3);
                t_roots = (t_roots(isInHull));
                if ~isempty(t_roots)
                    for TI = 1:numel(t_roots)
                        obj.NumSol = obj.NumSol + 1;
                        obj.CableSegmentInterRange(CSI).u(obj.NumSol) = obj.u_range(end);
                        obj.CableSegmentInterRange(CSI).v(obj.NumSol) = t_roots(TI);
                        obj.CableSegmentInterRange(CSI).ObstacleNum(obj.NumSol) = OI;
                        obj.CableSegmentInterRange(CSI).SurfaceNum(obj.NumSol) = SI;
                    end
                end
            end
        end
        
        %% function to find intersection between OB_u and Obstacles surface
        function CurveCutSurface(obj,CSI,OI,SI)
            %             seg_surf_ind = [];
            
            if obj.isTranslationDoF
                FoGu_coeff = FoGuT(obj.OB_1(CSI,:)',obj.OB_0(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
            else
                FoGu_coeff = FoGuO(obj.OB_u{CSI},obj.Obstacles(OI).SurfaceCoeffs{SI});
            end
            
            u_roots = obj.polyRootsTol(FoGu_coeff,obj.root_tol);
            u_roots(u_roots>obj.u_range(2)) = [];
            u_roots(u_roots<obj.u_range(1)) = [];
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
        function SegmentIntersection_Gxyz(obj,CSI,OI,BI)
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
                [u_roots,v_roots] = obj.Finduv(obj.OB_u{CSI},obj.OA_i_hat(CSI,:),IntersectPoint(:,i));
%                 u_roots = round(u_roots,obj.ROUNDING_DIGIT);
%                 v_roots = round(v_roots,obj.ROUNDING_DIGIT);

                if ~isempty(u_roots) && u_roots <= obj.u_range(2) &&  u_roots >= obj.u_range(1) &&...
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
        
        %% function to calculate the u,v where points on the G(u,v)
        function [u,v] = Finduv(obj,OB_u,OA,P)
            
            if obj.isTranslationDoF
                OA = OA';
                B0 = OB_u*[0;1];
                B1 = OB_u*[1;1];
                
                StartPt = [OA,B0]';
                EndPt   = [P ,B1]';
                PAB = obj.LineIntersection3D(StartPt,EndPt)';
                u = (PAB - B0)'*(B1-B0)/norm(B1 - B0);
                v = norm(P - OA)/norm(PAB - OA);
            else
                
                a1 = OB_u(1,1); b1 = OB_u(1,2); c1 = OB_u(1,3);  d1 = OA(1);
                a2 = OB_u(2,1); b2 = OB_u(2,2); c2 = OB_u(2,3);  d2 = OA(2);
                %             a3 = OB_u(3,1); b3 = OB_u(3,2); c3 = OB_u(3,3);  d3 = OA(3);
                x = P(1); y = P(2); z = P(3);
                u = [-(b1*d2 - b2*d1 + b2*x - b1*y + (b1^2*d2^2 + b2^2*d1^2 + b2^2*x^2 + b1^2*y^2 - 4*d2^2*x^2 - 4*d1^2*y^2 - 4*a1*c1*d2^2 - 4*a2*c2*d1^2 - 4*a2*c2*x^2 - 4*a1*c1*y^2 + 4*a1*d2^2*x + 4*a2*d2*x^2 + 4*a1*d1*y^2 + 4*a2*d1^2*y - 2*b2^2*d1*x + 4*c1*d2^2*x - 2*b1^2*d2*y + 4*c2*d2*x^2 + 4*c1*d1*y^2 + 4*c2*d1^2*y + 8*a1*c1*d2*y - 4*a1*c2*d1*y - 4*a2*c1*d1*y + 2*b1*b2*d1*y - 4*a2*d1*d2*x - 4*a1*d1*d2*y - 4*c2*d1*d2*x - 4*c1*d1*d2*y + 4*a1*c2*x*y + 4*a2*c1*x*y - 2*b1*b2*x*y - 4*a1*d2*x*y - 4*a2*d1*x*y - 4*c1*d2*x*y - 4*c2*d1*x*y + 8*d1*d2*x*y + 4*a1*c2*d1*d2 + 4*a2*c1*d1*d2 - 2*b1*b2*d1*d2 - 4*a1*c2*d2*x - 4*a2*c1*d2*x + 8*a2*c2*d1*x + 2*b1*b2*d2*x)^(1/2))/(2*(a1*d2 - a2*d1 + a2*x - a1*y - d2*x + d1*y));
                    (b2*d1 - b1*d2 - b2*x + b1*y + (b1^2*d2^2 + b2^2*d1^2 + b2^2*x^2 + b1^2*y^2 - 4*d2^2*x^2 - 4*d1^2*y^2 - 4*a1*c1*d2^2 - 4*a2*c2*d1^2 - 4*a2*c2*x^2 - 4*a1*c1*y^2 + 4*a1*d2^2*x + 4*a2*d2*x^2 + 4*a1*d1*y^2 + 4*a2*d1^2*y - 2*b2^2*d1*x + 4*c1*d2^2*x - 2*b1^2*d2*y + 4*c2*d2*x^2 + 4*c1*d1*y^2 + 4*c2*d1^2*y + 8*a1*c1*d2*y - 4*a1*c2*d1*y - 4*a2*c1*d1*y + 2*b1*b2*d1*y - 4*a2*d1*d2*x - 4*a1*d1*d2*y - 4*c2*d1*d2*x - 4*c1*d1*d2*y + 4*a1*c2*x*y + 4*a2*c1*x*y - 2*b1*b2*x*y - 4*a1*d2*x*y - 4*a2*d1*x*y - 4*c1*d2*x*y - 4*c2*d1*x*y + 8*d1*d2*x*y + 4*a1*c2*d1*d2 + 4*a2*c1*d1*d2 - 2*b1*b2*d1*d2 - 4*a1*c2*d2*x - 4*a2*c1*d2*x + 8*a2*c2*d1*x + 2*b1*b2*d2*x)^(1/2))/(2*(a1*d2 - a2*d1 + a2*x - a1*y - d2*x + d1*y))];
                %             u2 = [-(b1*d3 - b3*d1 + b3*x - b1*z + (b1^2*d3^2 + b3^2*d1^2 + b3^2*x^2 + b1^2*z^2 - 4*d3^2*x^2 - 4*d1^2*z^2 - 4*a1*c1*d3^2 - 4*a3*c3*d1^2 - 4*a3*c3*x^2 + 4*a1*d3^2*x + 4*a3*d3*x^2 - 4*a1*c1*z^2 - 2*b3^2*d1*x + 4*a1*d1*z^2 + 4*a3*d1^2*z + 4*c1*d3^2*x + 4*c3*d3*x^2 - 2*b1^2*d3*z + 4*c1*d1*z^2 + 4*c3*d1^2*z - 4*a3*d1*d3*x + 8*a1*c1*d3*z - 4*a1*c3*d1*z - 4*a3*c1*d1*z + 2*b1*b3*d1*z - 4*a1*d1*d3*z - 4*c3*d1*d3*x - 4*c1*d1*d3*z + 4*a1*c3*x*z + 4*a3*c1*x*z - 2*b1*b3*x*z - 4*a1*d3*x*z - 4*a3*d1*x*z - 4*c1*d3*x*z - 4*c3*d1*x*z + 8*d1*d3*x*z + 4*a1*c3*d1*d3 + 4*a3*c1*d1*d3 - 2*b1*b3*d1*d3 - 4*a1*c3*d3*x - 4*a3*c1*d3*x + 8*a3*c3*d1*x + 2*b1*b3*d3*x)^(1/2))/(2*(a1*d3 - a3*d1 + a3*x - a1*z - d3*x + d1*z));
                %                 (b3*d1 - b1*d3 - b3*x + b1*z + (b1^2*d3^2 + b3^2*d1^2 + b3^2*x^2 + b1^2*z^2 - 4*d3^2*x^2 - 4*d1^2*z^2 - 4*a1*c1*d3^2 - 4*a3*c3*d1^2 - 4*a3*c3*x^2 + 4*a1*d3^2*x + 4*a3*d3*x^2 - 4*a1*c1*z^2 - 2*b3^2*d1*x + 4*a1*d1*z^2 + 4*a3*d1^2*z + 4*c1*d3^2*x + 4*c3*d3*x^2 - 2*b1^2*d3*z + 4*c1*d1*z^2 + 4*c3*d1^2*z - 4*a3*d1*d3*x + 8*a1*c1*d3*z - 4*a1*c3*d1*z - 4*a3*c1*d1*z + 2*b1*b3*d1*z - 4*a1*d1*d3*z - 4*c3*d1*d3*x - 4*c1*d1*d3*z + 4*a1*c3*x*z + 4*a3*c1*x*z - 2*b1*b3*x*z - 4*a1*d3*x*z - 4*a3*d1*x*z - 4*c1*d3*x*z - 4*c3*d1*x*z + 8*d1*d3*x*z + 4*a1*c3*d1*d3 + 4*a3*c1*d1*d3 - 2*b1*b3*d1*d3 - 4*a1*c3*d3*x - 4*a3*c1*d3*x + 8*a3*c3*d1*x + 2*b1*b3*d3*x)^(1/2))/(2*(a1*d3 - a3*d1 + a3*x - a1*z - d3*x + d1*z))];
                v = -((u.^2 + 1)*(d1 - x))./(c1 - d1 + b1.*u + a1*u.^2 - d1*u.^2);
                OB = [polyval(OB_u(1,:),u),...
                    polyval(OB_u(2,:),u),...
                    polyval(OB_u(3,:),u)]./(1+u.^2);
                m1 = [OB - OA]'./vecnorm([OB - OA]');
                m2 = (P-OA')/ norm(P-OA');
                cor_ind = all(round(m1 - m2,obj.ROUNDING_DIGIT)==0);
                
                %
                u = u(cor_ind);
                v = v(cor_ind);
            end
        end
        
        %% %function to find intersection between Obstacles and cable swept surface
        function TangentToSurface(obj,CSI,OI,SI)
            
            if obj.Obstacles(OI).SurfaceDegree(SI) == 1
                return;
            end
            Delta_n_coeff = [];
            if ~obj.isTranslationDoF
                if obj.Obstacles(OI).SurfaceDegree(SI) == 4
                    a = H4_O(obj.OB_u{CSI},obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
                    b = H3_O(obj.OB_u{CSI},obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
                    c = H2_O(obj.OB_u{CSI},obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
                    d = H1_O(obj.OB_u{CSI},obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
                    e = H0_O(obj.OB_u{CSI},obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
                    Delta_n_coeff = Delta4u(a,b,c,d,e);
                elseif obj.Obstacles(OI).SurfaceDegree(SI) == 3
                    a = H3_O(obj.OB_u{CSI},obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
                    b = H2_O(obj.OB_u{CSI},obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
                    c = H1_O(obj.OB_u{CSI},obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
                    d = H0_O(obj.OB_u{CSI},obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
                    Delta_n_coeff = Delta3u(a,b,c,d);
                elseif obj.Obstacles(OI).SurfaceDegree(SI) == 2
                    a = H2_O(obj.OB_u{CSI},obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
                    b = H1_O(obj.OB_u{CSI},obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
                    c = H0_O(obj.OB_u{CSI},obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
                    Delta_n_coeff = Delta2u(a,b,c);
                end
            else
                if obj.Obstacles(OI).SurfaceDegree(SI) == 4
                    a = zeros(1,9); b = zeros(1,7); c = zeros(1,5); d = zeros(1,3); e = zeros(1,1);
                    a(5:end) = H4_T(obj.OB_u{CSI},obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
                    b(4:end) = H3_T(obj.OB_u{CSI},obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
                    c(3:end) = H2_T(obj.OB_u{CSI},obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
                    d(2:end) = H1_T(obj.OB_u{CSI},obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
                    e(1:end) = H0_T(obj.OB_u{CSI},obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
                    Delta_n_coeff = Delta4u(a,b,c,d,e);
                elseif obj.Obstacles(OI).SurfaceDegree(SI) == 3
                    a = zeros(1,7); b = zeros(1,5); c = zeros(1,3); d = zeros(1,1);
                    a(4:end) = H3_T(obj.OB_u{CSI},obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
                    b(3:end) = H2_T(obj.OB_u{CSI},obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
                    c(2:end) = H1_T(obj.OB_u{CSI},obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
                    d(1:end) = H0_T(obj.OB_u{CSI},obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
                    Delta_n_coeff = Delta3u(a,b,c,d);
                elseif obj.Obstacles(OI).SurfaceDegree(SI) == 2
                    a = zeros(1,5); b = zeros(1,3); c = zeros(1,1);
                    a(3:end) = H2_T(obj.OB_u{CSI},obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
                    b(2:end) = H1_T(obj.OB_u{CSI},obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
                    c = H0_T(obj.OB_u{CSI},obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI});
                    Delta_n_coeff = Delta2u(a,b,c);
                end
            end
            
            if any(isnan(Delta_n_coeff))
                Delta_n_coeff = zeros(size(Delta_n_coeff));
            end
            
            u_roots = obj.polyRootsTol(Delta_n_coeff,obj.root_tol);
            
            u_roots(u_roots>obj.u_range(2)) = [];
            u_roots(u_roots<obj.u_range(1)) = [];
            if ~isempty(u_roots)
                for UI = 1:size(u_roots,1)
                    
                    if obj.isTranslationDoF
                        FoGv = FoGvT(obj.OB_u{CSI},obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI},u_roots(UI));
                    else
                        FoGv = FoGvO(obj.OB_u{CSI},obj.OA_i_hat(CSI,:)',obj.Obstacles(OI).SurfaceCoeffs{SI},u_roots(UI));
                    end
                    
                    v_roots = unique(obj.polyRootsTol(FoGv,4e-2)); % this tolerance need to be handle carefully, high degree need bigger tol
                    v_lower = norm((obj.OA_at_u{CSI}(u_roots(UI))-obj.OA_i_hat(CSI,:)'))/norm((obj.OB_at_u{CSI}(u_roots(UI))-obj.OA_i_hat(CSI,:)'));
                    v_roots(v_roots>1) = [];
                    v_roots(v_roots<v_lower) = [];
                    for VI = 1:numel(v_roots)
                        obj.NumSol = obj.NumSol + 1;
                        obj.CableSegmentInterRange(CSI).u(obj.NumSol) =  u_roots(UI);
                        obj.CableSegmentInterRange(CSI).v(obj.NumSol) = v_roots(VI);
                        obj.CableSegmentInterRange(CSI).ObstacleNum(obj.NumSol) = OI;
                        obj.CableSegmentInterRange(CSI).SurfaceNum(obj.NumSol) = SI;
                    end
                end
            end
        end
        
        %%
        function [cable_implicit_coeff,cable_implicit_fun] = GetGxyz(obj,model,q_begin,q_end,free_variable_index,isTranslationDoF)
            cable_implicit_fun = []; cable_implicit_coeff = [];
            if isTranslationDoF
                sample_size = 2;
            else
                sample_size = 7;
            end
            
            q_sample = linspace(q_begin(free_variable_index),q_end(free_variable_index),sample_size);
            q_update = q_begin;
            surface_data = {};
            for segment_index = 1:size(q_sample,2)
                q_update(free_variable_index) = q_sample(segment_index);
                [OA_i_u,OB_i_u]  = obj.GetSegmentData(model,q_update);
                t = [0.1;0.25;0.5;0.75;0.9];
                for j = 1:size(t,1)
                    tmp_data = (OB_i_u - OA_i_u).*t(j) + OA_i_u;
                    for k = 1:obj.numSegment
                        if segment_index == 1 && j == 1
                            surface_data{k}(j,:) = tmp_data(k,:);
                        else
                            surface_data{k}(end+1,:) = tmp_data(k,:);
                        end
                    end
                end
            end
            v = zeros(obj.numSegment,10);
            for segment_index = 1:obj.numSegment
                if isTranslationDoF
                    v(segment_index,7:end) = obj.Plane_fit(surface_data{segment_index});
                    cable_implicit_fun{segment_index} =@(x,y,z) v(segment_index,7).*x +  v(segment_index,8).*y +  v(segment_index,9).*z +  v(segment_index,10);
                else
                    [v(segment_index,:),center(segment_index,:)] = obj.Cone_fit(surface_data{segment_index});
                    cable_implicit_fun{segment_index} =@(x,y,z) v(segment_index,1).*x.^2 + v(segment_index,2).*y.^2 + v(segment_index,3).*z.^2 + ...
                        v(segment_index,4).*x.*y + v(segment_index,5).*x.*z + v(segment_index,6).*y.*z + v(segment_index,7).*x + v(segment_index,8).*y +...
                        v(segment_index,9).*z + v(segment_index,10);
                end
            end
            
            cable_implicit_coeff = v;
        end
        %% function to fit the cable swept surface into implicit form for orientation
        function [ v,center ] = Cone_fit(~, surface_data )
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
        function v = Plane_fit(~,surface_data)
            % ref: Adrien Leygue (2022). Plane fit (https://www.mathworks.com/matlabcentral/fileexchange/43305-plane-fit), MATLAB Central File Exchange. Retrieved June 21, 2022
            
            p = mean(surface_data,1);
            R = bsxfun(@minus,surface_data,p);
            [V,D] = eig(R'*R);
            
            v = V(:,1);
            v(4) = -v'*p';
            
        end
        
        %% functin to get the cable segment data
        function [OA_i,OB_i] = GetSegmentData(obj,model,q)
            q_zero = zeros(model.numDofs,1);
            if obj.is_compiled_mode
                rOAi = model.cableModel.compiled_r_OAs_fn(q,q_zero,q_zero,q_zero);
                OA_i =  rOAi(1:3,:)';
                OB_i = rOAi(4:6,:)';
            else
                model.update(q,q_zero,q_zero,q_zero);
                rOAi = model.cableModel.r_OAs;
                OA_i =  rOAi(1:3,:)';
                OB_i = rOAi(4:6,:)';
            end
        end
        %%
        function [OB_u,OA_u]  = GetOB_u(obj,model,q_begin,q_end,free_variable_index,isTranslationDoF)
            OB_u_denominator = [];
            if isTranslationDoF
                sample_size = 2;
                q_sample = linspace(q_begin(free_variable_index),q_end(free_variable_index),sample_size);
                u_sample = linspace(0,1,sample_size);
                H_i_u_denominator = ones(size(u_sample));
                %                 OB_u_denominator = 1;
                OA_u_deg = 1;
                OB_u_deg = 1;
            else
                sample_size = 3;
                q_sample = linspace(1.05*q_begin(free_variable_index),1.05*q_end(free_variable_index),sample_size);
                u_sample = tan(q_sample/2);
                H_i_u_denominator = (1+u_sample.^2);
                %                 OB_u_denominator = @(u) 1+u.^2;
                OA_u_deg = 2;
                OB_u_deg = 2;
            end
            q_update = q_begin;
            for segment_index = 1:size(q_sample,2)
                q_update(free_variable_index) = q_sample(segment_index);
                [OA_i_u,OB_i_u]  = obj.GetSegmentData(model,q_update);
                for j = 1:obj.numSegment
                    OB_i_u_sample{j}(segment_index,:) = OB_i_u(j,:).*H_i_u_denominator(segment_index);
                    OA_i_u_sample{j}(segment_index,:) = OA_i_u(j,:).*H_i_u_denominator(segment_index);
                end
            end
            for segment_index = 1:obj.numSegment
                for j = 1:3
                    OB_u{segment_index}(j,:) = GeneralMathOperations.PolynomialFit(u_sample', OB_i_u_sample{segment_index}(:,j), OB_u_deg)';
                    OA_u{segment_index}(j,:) = GeneralMathOperations.PolynomialFit(u_sample', OA_i_u_sample{segment_index}(:,j), OA_u_deg)';
                end
                if isTranslationDoF
                    obj.OB_at_u{segment_index} =@(u) OB_u{segment_index}*[u;1];
                    obj.OA_at_u{segment_index} =@(u) OA_u{segment_index}*[u;1];
                else
                    obj.OB_at_u{segment_index} =@(u) OB_u{segment_index}*[u^2;u;1]/(1+u^2);
                    obj.OA_at_u{segment_index} =@(u) OA_u{segment_index}*[u^2;u;1]/(1+u^2);
                end
            end
            %OB_u -> [x(u);y(u);z(u)]
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
    
    methods(Static)
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
        
        function GenerateCloseFormSolution
            error('This function must be used carefully, please refer to the paper first')
            % this is a function to record the IFW method
            % first define your implicit surface
            
            c = sym('c%d',[31,1]);
            % C = sym('c%d',[31,1]);
            Fi = @(x,y,z) c(1).*x.^4 + c(2).*y.^4 + c(3).*z.^4 +...
                c(4).*x.^3.*y + c(5).*x.^3.*z + c(6).*y.^3.*x + c(7).*y.^3.*z + c(8).*z.^3.*x + c(9).*z.^3.*y + ...
                c(10)*x.^2.*y^2 + c(11)*x.^2.*z^2 + c(12)*y.^2.*z^2 + ...
                c(13).*x.^3 + c(14).*y.^3 + c(15).*z.^3 +...
                c(16).*x.^2.*y + c(17).*x.^2.*z + c(18).*y.^2.*x + c(19).*y.^2.*z +...
                c(20).*z.^2.*x + c(21).*z.^2.*y + ...
                c(22).*x.^2 + c(23).*y.^2 + c(24).*z.^2 +...
                c(25).*x.*y + c(26).*x.*z + c(27).*y.*z + ...
                c(28).*x + c(29).*y + c(30).*z + c(31);
            
            % second define cable segement for translation and orientation
            % 1 translation:
            syms b11 b12 b13 b21 b22 b23 b31 b32 b33
            B = sym('b%d',[3,2]);
            OA = sym('A%d',[3,1]);
            OBu = @(u) [b11.*u + b12;
                b21.*u + b22;
                b31.*u + b32];
            % 2 orientation
            CDu =@(u) 1/(u^2 + 1);
            OBu =@(u) CDu*[(b11.*u^2 + b12.*u + b13);
                (b21.*u^2 + b22.*u + b23);
                (b31.*u^2 + b32.*u + b33)];
            % general form of cable segment surface in u v
            Guv_fun = @(u,v) (OBu(u) - OA).*v + OA;
            syms u v
            Guv = Guv_fun(u,v);
            %%
            FoG = Fi(Guv(1),Guv(2),Guv(3));
            % the coefficient in terms of v
            [FoGv_Coeff,v_p] = coeffs(FoG,v);
            %matlabFunction(FoGv_Coeff,'file','FoGvT.m','Vars', {B,OA,C,u}); %translation
            
            % for orientation, you need to remove the denominator
            [FoG_N,FoG_D] = numden(FoG);
            [FoGv_Coeff,v_p] = coeffs(FoG_N,v);
            %matlabFunction(FoGv_Coeff,'file','FoGvO.m','Vars', {B,OA,C,u}); %orientation
            
            % then get every term w.r.t u for FoGv_Coeff
            
            for i = 1:size(FoGv_Coeff,2)
                [FoGu_Coeff{i},u_power{i}] = coeffs(FoGv_Coeff(i),u);
            end
            
            % translation
            H4 = matlabFunction(u_v{1},'file','H4_T.m','Vars', {B,OA,C});
            H3 = matlabFunction(u_v{2},'file','H3_T.m','Vars', {B,OA,C});
            H2 = matlabFunction(u_v{3},'file','H2_T.m','Vars', {B,OA,C});
            H1 = matlabFunction(u_v{4},'file','H1_T.m','Vars', {B,OA,C});
            H0 = matlabFunction(u_v{5},'file','H0_T.m','Vars', {B,OA,C});
            % orientation
            H4 = matlabFunction(u_v{1},'file','H4_O.m','Vars', {B,OA,C});
            H3 = matlabFunction(u_v{2},'file','H3_O.m','Vars', {B,OA,C});
            H2 = matlabFunction(u_v{3},'file','H2_O.m','Vars', {B,OA,C});
            H1 = matlabFunction(u_v{4},'file','H1_O.m','Vars', {B,OA,C});
            H0 = matlabFunction(u_v{5},'file','H0_O.m','Vars', {B,OA,C});
            
            % then generate the close form solution w.r.t Hn for different
            % degree
            U4 = sym('u4%d',[1 9]);
            U3 = sym('u3%d',[1 7]);
            U2 = sym('u2%d',[1 5]);
            U1 = sym('u1%d',[1 3]);
            syms u0;
            
            a_u(1) = U4*[u^8 u^7 u^6 u^5 u^4 u^3 u^2 u^1 u^0].';
            b_u(2) = U3*[u^6 u^5 u^4 u^3 u^2 u^1 u^0].';
            c_u(3) = U2*[u^4 u^3 u^2 u^1 u^0].';
            d_u(4) = U1*[u^2 u^1 u^0].';
            e_u(5) = u0;
            % 4th degree
            u_fun = simplify(256*a_u^3*e_u^3 - 192*a_u^2*b_u*d_u*e_u^2 - 128*a_u^2*c_u^2*e_u^2 + 144*a_u^2*c_u*d_u^2*e_u + ...
                -27*a_u^2*d_u^4 + 144*a_u*b_u^2*c_u*e_u^2 - 6*a_u*b_u^2*d_u^2*e_u - 80*a_u*b_u*c_u^2*d_u*e_u + ...
                18*a_u*b_u*c_u*d_u^3 + 16*a_u*c_u^4*e_u - 4*a_u*c_u^3*d_u^2 - 27*b_u^4*e_u^2 + 18*b_u^3*c_u*d_u*e_u + ...
                -4*b_u^3*d_u^3 - 4*b_u^2*c_u^3*e_u + b_u^2*c_u^2*d_u^2);
            
            [u_c,u_p] = coeffs(u_fun,u);
            %matlabFunction(u_c,'file','Delta4u.m','Vars', {U3,U2,U1,u0});
            % 3rd degree
            a_u = U3*[u^6 u^5 u^4 u^3 u^2 u^1 u^0].'; %a
            b_u = U2*[u^4 u^3 u^2 u^1 u^0].'; %b
            c_u = U1*[u^2 u^1 u^0].'; %c
            d_u= u0; %d
            
            u_fun = simplify(b_u^2*c_u^2 - 4*a_u*c_u^3 - 4*b_u^3*d_u - 27*a_u^2*d_u^2 + 18*a_u*b_u*c_u*d_u);
            [u_c,u_p] = coeffs(u_fun,u);
            %matlabFunction(u_c,'file','Delta3u.m','Vars', {U3,U2,U1,u0});
            % 2nd degree
            a_u = U2*[u^4 u^3 u^2 u^1 u^0].'; %a
            b_u = U1*[u^2 u^1 u^0].'; %b
            c_u= u0; %c
            
            u_fun = simplify(b_u^2 - 4*a_u*c_u);
            [u_c,u_p] = coeffs(u_fun,u);
            % matlabFunction(u_c,'file','Delta2u.m','Vars', {U2,U1,u0});
            
            %% %%%%%% the the fjk intersect the Gxyz
            % First define the general Gxyz
            %% curve/line v.s implicit surface G(x,y,z)
            G = sym('g%d',[10,1])
            Gi = @(x,y,z) G(1)*x.^2 + G(2)*y.^2 + G(3)*z.^2 + G(4)*x.*y + ...
                G(5)*x.*z + G(6)*y.*z + G(7)*x + G(8)*y +  G(9)*z + G(10);
            % define the bezier intersected curve N(u)/D(u)
            % example of 4 degree curve
            D = sym('d%d%d',[3,5]);
            N = sym('n%d%d',[3,5]);
            DS = D*[t^4,t^3,t^2,t,1].';
            NS = N*[t^4,t^3,t^2,t,1].';
            FS = NS./DS;
            G1 = Gi(FS(1),FS(2),FS(3));
            [N_T,D_T] = numden(G1);
            [g_c,g_p] = coeffs(N_T,t);
            N_T = simplify(N_T);
            %matlabFunction(g_c,'file','GoftD4.m','Vars', {D,N,G});
            
            syms t
            OB = sym('b%d',[3,1]);
            OA = sym('a%d',[3,1]);
            GT = (OB - OA)*t + OA;
            FGT = Fi(GT(1),GT(2),GT(3));
            
            [t_c,t_p] = coeffs(FGT,t);
            matlabFunction(t_c,'file','FoSt.m','Vars', {OB,OA,c});
            
        end
    end
end
