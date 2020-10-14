% A container class to hold the obstacle used in interference-free
% workspace analysis in ray-based
%
% Author        : Paul CHENG
% Created       : 2020
% Description    : This class contains the the obstacles formed by its
% implicit surface equations and its parametric form boundary curves
classdef CreateObstacleElement < handle
    properties(SetAccess = protected)
        surfaceEqu                 % the surface implicit equations
        surfaceDeg                 % the degree of surface implicit equations        
        surfaceCoeffs              % coefficient of surface equation
        surfaceBoundXYZ            % XYZ boundary of surface
        
        boundaryEqu                % the boundary parametric equations
        boundaryEquCoeff           % coefficient of the boundary parametric equations
        boundaryDeg                % the degree of boundary parametric equations
        
        surfaceDirection
        
        surfaceNum
        boundaryNum
    end
    methods
        function obs = CreateObstacleElement(surface,surfaceBoundXYZ,Direction,boundary)
            syms x y z t;
            obs.surfaceDirection = Direction;
            obs.surfaceNum = size(surface,2);
            obs.boundaryNum = size(boundary,2);
            
            for surf_ind = 1:size(surface,2)
                [coeff_f,var_f] = coeffs(surface{surf_ind}(x,y,z));   
                Fi = @(x,y,z) c(1).*x.^4 + c(2).*y.^4 + c(3).*z.^4 +...
                    c(4).*x.^3.*y + c(5).*x.^3.*z + c(6).*y.^3.*x + c(7).*y.^3.*z + c(8).*z.^3.*x + c(9).*z.^3.*y + ...
                    c(10)*x.^2.*y^2 + c(11)*x.^2.*z^2 + c(12)*y.^2.*z^2 + ...
                    c(13).*x.^3 + c(14).*y.^3 + c(15).*z.^3 +...
                    c(16).*x.^2.*y + c(17).*x.^2.*z + c(18).*y.^2.*x + c(19).*y.^2.*z +...
                    c(20).*z.^2.*x + c(21).*z.^2.*y + ...
                    c(22).*x.^2 + c(23).*y.^2 + c(24).*z.^2 +...
                    c(25).*x.*y + c(26).*x.*z + c(27).*y.*z + ...
                    c(28).*x + c(29).*y + c(30).*z + c(31);
               % Fi = @(x,y,z) c1.*x.^4 + c2.*y.^4 + c3.*z.^4 +...
                % c4.*x.^3.*y + c5.*x.^3.*z + c6.*y.^3.*x + c7.*y.^3.*z + c8.*z.^3.*x + c9.*z.^3.*y + ...
                % c10.*x.^3 + c11.*y.^3 + c12.*z.^3 +...
                % c13.*x.^2.*y + c14.*x.^2.*z + c15.*y.^2.*x + c16.*y.^2.*z + c17.*z.^2.*x + c18.*z.^2.*y + ...
                % c19.*x.^2 + c20.*y.^2 + c21.*z.^2 +...
                % c22.*x.*y + c23.*x.*z + c4.*y.*z + ...
                % c25.*x + c26.*y + c27.*z + c28;
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
                obs.surfaceCoeffs{surf_ind} = Q;
                obs.surfaceEqu{surf_ind}  = surface{surf_ind};
                
                obs.surfaceBoundXYZ{surf_ind} = surfaceBoundXYZ{surf_ind};
                deg = [size(coeffs(surface{surf_ind},x),2);
                    size(coeffs(surface{surf_ind},y),2);
                    size(coeffs(surface{surf_ind},z),2)];
                max_deg = max(deg);
                if  max_deg == 1
                    obs.surfaceDeg(surf_ind) = max_deg;
                else
                    obs.surfaceDeg(surf_ind) = max_deg-1;
                end
            end
            
            for surf_ind = 1:size(boundary,2)
                obs.boundaryEquCoeff{surf_ind} = zeros(3,5);
                obs.boundaryEqu{surf_ind}  = boundary{surf_ind};
                boundaryEqu = boundary{surf_ind}(t);
                for j = 1:3
                   [t_c,t_p]  = coeffs(boundaryEqu(j),t);
                   deg(j) = size(t_p,2);
                   for k = 1:size(t_p,2)
                       if isequal(t_p(k),t^4)
                           obs.boundaryEquCoeff{surf_ind}(j,1) =  t_c(k);
                       elseif isequal(t_p(k),t^3)
                           obs.boundaryEquCoeff{surf_ind}(j,2) =  t_c(k);
                       elseif isequal(t_p(k),t^2)
                           obs.boundaryEquCoeff{surf_ind}(j,3) =  t_c(k);
                       elseif isequal(t_p(k),t)
                           obs.boundaryEquCoeff{surf_ind}(j,4) =  t_c(k);
                       else
                           obs.boundaryEquCoeff{surf_ind}(j,5) =  t_c(k);
                       end
                   end
                    
                end
                max_deg = max(deg);
                if  max_deg == 1
                    obs.boundaryDeg(surf_ind) = max_deg;
                else
                    obs.boundaryDeg(surf_ind) = max_deg-1;
                end
            end
        end
        
        function obs_plot = plotObstacle(obj)
            
        end
        
    end
end