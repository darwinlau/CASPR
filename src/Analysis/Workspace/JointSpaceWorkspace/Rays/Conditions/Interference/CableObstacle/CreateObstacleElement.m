% A container class to hold the obstacle used in interference-free
% workspace analysis in ray-based
%
% Author        : Paul CHENG
% Created       : 2020
% Description    : This class contains the the obstacles formed by its
% implicit surface equations and its parametric form boundary curves

% Important note: only support up to 4 degree surface

classdef CreateObstacleElement < handle
    properties(SetAccess = protected)
        ObstacleHull              % the convex hull of the obstacle vertices
        MeshGridDensity = 50;     % to detect the intersection point inside the obstacle, large number may be slower
        
        ImplicitEqu               % the surface implicit equations
        SurfaceDegree             % the degree of surface implicit equations
        SurfaceCoeffs             % coefficient of surface equation
        SurfacesDirection
        SurfacesXYZRange          % XYZ boundary of surface
        NumSurfaces = 0;          % number of surface
        
        
        ParametricEqu             % the boundary parametric equations
        ParametricDeg             % the degree of boundary parametric equations
        ParametricRange           % range of the parametric boundary equations
        ParametricCoeffs          % coefficient of the boundary parametric equations
        NumBoundary = 0;          % number of boundary
        
        P2SIndex                  % n x 2 parametric boundary to related surfaces index
        S2PIndex                  % n x m surfaces surface to related parametric boundary index
        
        ObstacleTol = 0.0025;       % numerical tolerance allowed in computation,
                                  % the computation distance(in meter) error between
                                  % the intersection point and the obstacle
                                  % surface, higher degree surface required larger error
                                  
        NeglectInterior = 1       % deafult: 1 -> only consider the exterior of the obstacle; 
                                  % option : 0 -> slower but more comprehensive. For
                                  %               obstacle has hollow interior such
                                  %               as donut;
                                   
    end
    properties (Access = private)
        
    end
    methods
        function obs = CreateObstacleElement(ImplicitSurfaces,SurfacesXYZRange,SurfacesDirection,ParametricBoundary,ParametricRange,BoundarySurfacesConnectivity,varargin)
            
            disp('Creating the obstacle(s), please plot the obstacle for ensurance of the surfaces and boundary equations')
            
            syms x y z t;
            obs.SurfacesDirection = SurfacesDirection;
            obs.NumSurfaces = size(ImplicitSurfaces,2);
            obs.NumBoundary = size(ParametricBoundary,2);
            if ~isempty(varargin)
               for i = 1:2:size(varargin,2)
                    if strcmpi(varargin{i},'NeglectInterior')
                        obs.NeglectInterior = varargin{i+1};
                    elseif strcmpi(varargin{i},'ObstacleTol')
                        obs.ObstacleTol = varargin{i+1};
                        
                    end
               end
            end
            if  obs.NeglectInterior == 0
                disp('Considering the interior of the obstacles. Slower computation might be occured...')
            else
                disp('Only considering the exterior of the obstacles.')
            end
                
            
            % generate the surface coefficient and degrees
            VerticeHul = [];
            for SI = 1:obs.NumSurfaces
                
                Q = obs.getSurfaceCoeffs(ImplicitSurfaces{SI}(x,y,z),x,y,z);
                
                obs.SurfaceCoeffs{SI} = Q;
                obs.ImplicitEqu{SI}  = ImplicitSurfaces{SI};
                
                obs.SurfacesXYZRange{SI} = SurfacesXYZRange{SI};
                deg = [size(coeffs(ImplicitSurfaces{SI},x),2);
                    size(coeffs(ImplicitSurfaces{SI},y),2);
                    size(coeffs(ImplicitSurfaces{SI},z),2)];
                max_deg = max(deg);
                if  max_deg == 1
                    obs.SurfaceDegree(SI) = max_deg;
                else
                    obs.SurfaceDegree(SI) = max_deg-1;
                end
                SurfaceHull{SI} = obs.getSurfaceHull(ImplicitSurfaces{SI},SurfacesXYZRange{SI},obs.MeshGridDensity);
                VerticeHul = [VerticeHul;SurfaceHull{SI}];
            end
            
            
            for SI = 1:obs.NumSurfaces
                vertic_sign = sign(ImplicitSurfaces{SI}(VerticeHul(:,1),VerticeHul(:,2),VerticeHul(:,3)));
                vertic_sign(vertic_sign == 0) = SurfacesDirection(SI);
                OutOfSurface(:,SI) = ( vertic_sign ~= SurfacesDirection(SI));
            end
            VerticeHul(any(OutOfSurface,2),:) = [];
            
            [k,v] = convhull(VerticeHul,'Simplify',true);
            obs.ObstacleHull.Indices = k;
            obs.ObstacleHull.Vertices = VerticeHul;
            Vert_ind = unique(k);
            obs.ObstacleHull.SimplifiedVertices = VerticeHul(Vert_ind,:);
            % trisurf(k,VerticeHul(:,1),VerticeHul(:,2),VerticeHul(:,3),'FaceColor','cyan','EdgeColor',[0.1,0.1,0.1]);hold on
            
            % generate the boundaries coefficient, degrees and index
            obs.ParametricRange = ParametricRange;
            if ~isempty(BoundarySurfacesConnectivity)
                obs.P2SIndex = BoundarySurfacesConnectivity;
                for SI = 1:obs.NumSurfaces
                    [col,~] = find(ismember(obs.P2SIndex(:,2:3),SI));
                    obs.S2PIndex{SI} = [SI,obs.P2SIndex(col,1)'];
                end
            end
            
            obs.S2PIndex = obs.S2PIndex';
            
            for i = 1:obs.NumBoundary
                obs.ParametricDeg(i) = max(obs.SurfaceDegree(obs.P2SIndex(i,[2,3])));
            end
            
            for PI = 1:obs.NumBoundary
                obs.ParametricEqu{PI}  = ParametricBoundary{PI};
                [N,D] = obs.getParametricCoeffs(ParametricBoundary{PI}(t),t);
                
                obs.ParametricCoeffs{PI}.N = N;
                obs.ParametricCoeffs{PI}.D = D;
            end
            
        end
        
        function obs_plot = plotObstacle(obj,varargin)
            plot_num = 1:obj.NumSurfaces;
            mesh_density = 20;
            face_transparent = 1;
            boundary_color = 'k';
            face_color = 'interp';
            plot_method = 1; % 1->mesh plot; 2->implicit
            PlotEdge = 0;
            if ~isempty(varargin)
                for i = 1:2:size(varargin,2)
                    if strcmpi(varargin{i},'EdgeColor')
                        boundary_color = varargin{i+1};
                    elseif strcmpi(varargin{i},'FaceAlpha')
                        face_transparent = varargin{i+1};
                    elseif strcmpi(varargin{i},'PlotEdge')
                        PlotEdge = varargin{i+1};
                    elseif strcmpi(varargin{i},'Color')
                        face_color = varargin{i+1};
                    elseif strcmpi(varargin{i},'meshdensity')
                        mesh_density = varargin{i+1};
                    elseif strcmpi(varargin{i},'PlotMethod')
                        plot_method = varargin{i+1};
                    end
                end
            end
            
            if plot_method == 2
                for i = 1:numel(plot_num)
                    obs_plot(i)= fimplicit3(obj.ImplicitEqu{plot_num(i)},obj.SurfacesXYZRange{plot_num(i)},'FaceColor',face_color,'FaceAlpha',face_transparent,'MeshDensity',mesh_density,'EdgeColor',boundary_color);
                    hold on
                end
                if PlotEdge
                    for i = 1:obj.NumBoundary
                        t = linspace(obj.ParametricRange{i}(1),obj.ParametricRange{i}(2),50);
                        for j = 1:numel(t)
                            data(:,j) = obj.ParametricEqu{i}(t(j));
                        end
                        obs_plot(end+1) = plot3(data(1,:),data(2,:),data(3,:),'k','linewidth',2);                        
                    end
                end                
            else
                obs_plot = trisurf(obj.ObstacleHull.Indices,obj.ObstacleHull.Vertices(:,1),obj.ObstacleHull.Vertices(:,2),obj.ObstacleHull.Vertices(:,3),'FaceColor',face_color,'FaceAlpha',face_transparent,'EdgeColor',boundary_color);
                hold on
                if PlotEdge
                    for i = 1:obj.NumBoundary
                        t = linspace(obj.ParametricRange{i}(1),obj.ParametricRange{i}(2),50);
                        for j = 1:numel(t)
                            data(:,j) = obj.ParametricEqu{i}(t(j));
                        end
                        obs_plot(end+1) = plot3(data(1,:),data(2,:),data(3,:),'k','linewidth',2);                        
                    end
                end
            end
        end
        
    end
    
    methods(Static)
        function Q = getSurfaceCoeffs(Surfaces,x,y,z)
            % currently up to 4th degree only
            [coeff_f,var_f] = coeffs(Surfaces);
            Fi = @(x,y,z) c(1).*x.^4 + c(2).*y.^4 + c(3).*z.^4 +...
                c(4).*x.^3.*y + c(5).*x.^3.*z + c(6).*y.^3.*x + c(7).*y.^3.*z + c(8).*z.^3.*x + c(9).*z.^3.*y + ...
                c(10)*x.^2.*y^2 + c(11)*x.^2.*z^2 + c(12)*y.^2.*z^2 + ...
                c(13).*x.^3 + c(14).*y.^3 + c(15).*z.^3 +...
                c(16).*x.^2.*y + c(17).*x.^2.*z + c(18).*y.^2.*x + c(19).*y.^2.*z +...
                c(20).*z.^2.*x + c(21).*z.^2.*y + ...
                c(22).*x.^2 + c(23).*y.^2 + c(24).*z.^2 +...
                c(25).*x.*y + c(26).*x.*z + c(27).*y.*z + ...
                c(28).*x + c(29).*y + c(30).*z + c(31);
            Q = zeros(31,1);
            for i = 1:size(coeff_f,2)
                if isequal(var_f(i),x^4)
                    Q(1) = coeff_f(i);
                elseif isequal(var_f(i),y^4)
                    Q(2) = coeff_f(i);
                elseif isequal(var_f(i),z^4)
                    Q(3) = coeff_f(i);
                elseif isequal(var_f(i),x^3*y)
                    Q(4) = coeff_f(i);
                elseif isequal(var_f(i),x^3*z)
                    Q(5) = coeff_f(i);
                elseif isequal(var_f(i),y^3*x)
                    Q(6) = coeff_f(i);
                elseif isequal(var_f(i),y^3*z)
                    Q(7) = coeff_f(i);
                    elseifisequal(var_f(i),z^3*x)
                    Q(8) = coeff_f(i);
                elseif isequal(var_f(i),z^3*y)
                    Q(9) = coeff_f(i);
                elseif isequal(var_f(i),x^2*y^2)
                    Q(10) = coeff_f(i);
                elseif isequal(var_f(i),x^2*z^2)
                    Q(11) = coeff_f(i);
                elseif isequal(var_f(i),y^2*z^2)
                    Q(12) = coeff_f(i);
                elseif isequal(var_f(i),x^3)
                    Q(13) = coeff_f(i);
                elseif isequal(var_f(i),y^3)
                    Q(14) = coeff_f(i);
                elseif isequal(var_f(i),z^3)
                    Q(15) = coeff_f(i);
                elseif isequal(var_f(i),x^2*y)
                    Q(16) = coeff_f(i);
                elseif isequal(var_f(i),x^2*z)
                    Q(17) = coeff_f(i);
                elseif isequal(var_f(i),y^2*x)
                    Q(18) = coeff_f(i);
                elseif isequal(var_f(i),y^2*z)
                    Q(19) = coeff_f(i);
                elseif isequal(var_f(i),z^2*x)
                    Q(20) = coeff_f(i);
                elseif isequal(var_f(i),z^2*y)
                    Q(21) = coeff_f(i);
                elseif isequal(var_f(i),x^2)
                    Q(22) = coeff_f(i);
                elseif isequal(var_f(i),y^2)
                    Q(23) = coeff_f(i);
                elseif isequal(var_f(i),z^2)
                    Q(24) = coeff_f(i);
                elseif isequal(var_f(i),x*y)
                    Q(25) = coeff_f(i);
                elseif isequal(var_f(i),x*z)
                    Q(26) = coeff_f(i);
                elseif isequal(var_f(i),y*z)
                    Q(27) = coeff_f(i);
                elseif isequal(var_f(i),x)
                    Q(28) = coeff_f(i);
                elseif isequal(var_f(i),y)
                    Q(29) = coeff_f(i);
                elseif isequal(var_f(i),z)
                    Q(30) = coeff_f(i);
                else
                    Q(31) = coeff_f(i);
                end
                
            end
            
        end
        
        function [Nume,Deno] = getParametricCoeffs(ParametricEqu,t)
            [N,D] = numden(ParametricEqu);
            for i = 1:numel(N)
                [coeff_t,var_t] = coeffs(N(i),t);
                P = zeros(5,1);
                for k = 1:size(coeff_t,2)
                    if isequal(var_t(k),t^4)
                        P(1) = coeff_t(k);
                    elseif isequal(var_t(k),t^3)
                        P(2) = coeff_t(k);
                    elseif isequal(var_t(k),t^2)
                        P(3) = coeff_t(k);
                    elseif isequal(var_t(k),t)
                        P(4) = coeff_t(k);
                    else
                        P(5) = coeff_t(k);
                    end
                end
                Nume(i,:) = P;
            end
            
            for i = 1:numel(D)
                [coeff_t,var_t] = coeffs(D(i),t);
                P = zeros(5,1);
                for k = 1:size(coeff_t,2)
                    if isequal(var_t(k),t^4)
                        P(1) = coeff_t(k);
                    elseif isequal(var_t(k),t^3)
                        P(2) = coeff_t(k);
                    elseif isequal(var_t(k),t^2)
                        P(3) = coeff_t(k);
                    elseif isequal(var_t(k),t)
                        P(4) = coeff_t(k);
                    else
                        P(5) = coeff_t(k);
                    end
                end
                Deno(i,:) = P;
            end
            
        end
        
        function H = getSurfaceHull(Surfaces,Boundary,Density)
            % method 1
            xyz_range = Boundary;
            x = linspace(xyz_range(1),xyz_range(2),Density);
            y = linspace(xyz_range(3),xyz_range(4),Density);
            z = linspace(xyz_range(5),xyz_range(6),Density);
            [X,Y,Z ]= meshgrid(x,y,z);
            V = Surfaces(X,Y,Z);
            I = isosurface(x,y,z,V,0);
            H = I.vertices;
            % method 2 this required external toolbox: Philip Bean (2022). ImplicitData3 (https://www.mathworks.com/matlabcentral/fileexchange/72733-implicitdata3), MATLAB Central File Exchange. Retrieved June 21, 2022.
            %                 [x_data,y_data,z_data]=ImplicitData3(ImplicitSurfaces{SI}, SurfacesXYZRange{SI},obs.obstacle_surface_density);
            %                 H = [x_data',y_data',z_data'];
        end
        
        % John D'Errico (2022). Inhull (https://www.mathworks.com/matlabcentral/fileexchange/10226-inhull), MATLAB Central File Exchange. Retrieved June 17, 2022.
        function in = inhull(testpts,xyz,tess,tol)
            % inhull: tests if a set of points are inside a convex hull
            % usage: in = inhull(testpts,xyz)
            % usage: in = inhull(testpts,xyz,tess)
            % usage: in = inhull(testpts,xyz,tess,tol)
            %
            % arguments: (input)
            %  testpts - nxp array to test, n data points, in p dimensions
            %       If you have many points to test, it is most efficient to
            %       call this function once with the entire set.
            %
            %  xyz - mxp array of vertices of the convex hull, as used by
            %       convhulln.
            %
            %  tess - tessellation (or triangulation) generated by convhulln
            %       If tess is left empty or not supplied, then it will be
            %       generated.
            %
            %  tol - (OPTIONAL) tolerance on the tests for inclusion in the
            %       convex hull. You can think of tol as the distance a point
            %       may possibly lie outside the hull, and still be perceived
            %       as on the surface of the hull. Because of numerical slop
            %       nothing can ever be done exactly here. I might guess a
            %       semi-intelligent value of tol to be
            %
            %         tol = 1.e-13*mean(abs(xyz(:)))
            %
            %       In higher dimensions, the numerical issues of floating
            %       point arithmetic will probably suggest a larger value
            %       of tol.
            %
            %       DEFAULT: tol = 0
            %
            % arguments: (output)
            %  in  - nx1 logical vector
            %        in(i) == 1 --> the i'th point was inside the convex hull.
            %
            % Example usage: The first point should be inside, the second out
            %
            %  xy = randn(20,2);
            %  tess = convhulln(xy);
            %  testpoints = [ 0 0; 10 10];
            %  in = inhull(testpoints,xy,tess)
            %
            % in =
            %      1
            %      0
            %
            % A non-zero count of the number of degenerate simplexes in the hull
            % will generate a warning (in 4 or more dimensions.) This warning
            % may be disabled off with the command:
            %
            %   warning('off','inhull:degeneracy')
            %
            % See also: convhull, convhulln, delaunay, delaunayn, tsearch, tsearchn
            %
            % Author: John D'Errico
            % e-mail: woodchips@rochester.rr.com
            % Release: 3.0
            % Release date: 10/26/06
            
            % get array sizes
            % m points, p dimensions
            p = size(xyz,2);
            [n,c] = size(testpts);
            if p ~= c
                error 'testpts and xyz must have the same number of columns'
            end
            if p < 2
                error 'Points must lie in at least a 2-d space.'
            end
            
            % was the convex hull supplied?
            if (nargin<3) || isempty(tess)
                tess = convhulln(xyz);
            end
            [nt,c] = size(tess);
            if c ~= p
                error 'tess array is incompatible with a dimension p space'
            end
            
            % was tol supplied?
            if (nargin<4) || isempty(tol)
                tol = 0;
            end
            
            % build normal vectors
            switch p
                case 2
                    % really simple for 2-d
                    nrmls = (xyz(tess(:,1),:) - xyz(tess(:,2),:)) * [0 1;-1 0];
                    
                    % Any degenerate edges?
                    del = sqrt(sum(nrmls.^2,2));
                    degenflag = (del<(max(del)*10*eps));
                    if sum(degenflag)>0
                        warning('inhull:degeneracy',[num2str(sum(degenflag)), ...
                            ' degenerate edges identified in the convex hull'])
                        
                        % we need to delete those degenerate normal vectors
                        nrmls(degenflag,:) = [];
                        nt = size(nrmls,1);
                    end
                case 3
                    % use vectorized cross product for 3-d
                    ab = xyz(tess(:,1),:) - xyz(tess(:,2),:);
                    ac = xyz(tess(:,1),:) - xyz(tess(:,3),:);
                    nrmls = cross(ab,ac,2);
                    degenflag = false(nt,1);
                otherwise
                    % slightly more work in higher dimensions,
                    nrmls = zeros(nt,p);
                    degenflag = false(nt,1);
                    for i = 1:nt
                        % just in case of a degeneracy
                        % Note that bsxfun COULD be used in this line, but I have chosen to
                        % not do so to maintain compatibility. This code is still used by
                        % users of older releases.
                        %  nullsp = null(bsxfun(@minus,xyz(tess(i,2:end),:),xyz(tess(i,1),:)))';
                        nullsp = null(xyz(tess(i,2:end),:) - repmat(xyz(tess(i,1),:),p-1,1))';
                        if size(nullsp,1)>1
                            degenflag(i) = true;
                            nrmls(i,:) = NaN;
                        else
                            nrmls(i,:) = nullsp;
                        end
                    end
                    if sum(degenflag)>0
                        warning('inhull:degeneracy',[num2str(sum(degenflag)), ...
                            ' degenerate simplexes identified in the convex hull'])
                        
                        % we need to delete those degenerate normal vectors
                        nrmls(degenflag,:) = [];
                        nt = size(nrmls,1);
                    end
            end
            
            % scale normal vectors to unit length
            nrmllen = sqrt(sum(nrmls.^2,2));
            % again, bsxfun COULD be employed here...
            %  nrmls = bsxfun(@times,nrmls,1./nrmllen);
            nrmls = nrmls.*repmat(1./nrmllen,1,p);
            
            % center point in the hull
            center = mean(xyz,1);
            
            % any point in the plane of each simplex in the convex hull
            a = xyz(tess(~degenflag,1),:);
            
            % ensure the normals are pointing inwards
            % this line too could employ bsxfun...
            %  dp = sum(bsxfun(@minus,center,a).*nrmls,2);
            dp = sum((repmat(center,nt,1) - a).*nrmls,2);
            k = dp<0;
            nrmls(k,:) = -nrmls(k,:);
            
            % We want to test if:  dot((x - a),N) >= 0
            % If so for all faces of the hull, then x is inside
            % the hull. Change this to dot(x,N) >= dot(a,N)
            aN = sum(nrmls.*a,2);
            
            % test, be careful in case there are many points
            in = false(n,1);
            
            % if n is too large, we need to worry about the
            % dot product grabbing huge chunks of memory.
            memblock = 1e6;
            blocks = max(1,floor(n/(memblock/nt)));
            aNr = repmat(aN,1,length(1:blocks:n));
            for i = 1:blocks
                j = i:blocks:n;
                if size(aNr,2) ~= length(j),
                    aNr = repmat(aN,1,length(j));
                end
                in(j) = all((nrmls*testpts(j,:)' - aNr) >= -tol,1)';
            end
            
            
        end
    end
end