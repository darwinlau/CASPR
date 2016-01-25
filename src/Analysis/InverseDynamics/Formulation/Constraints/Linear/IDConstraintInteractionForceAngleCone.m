classdef IDConstraintInteractionForceAngleCone < IDConstraintLinear
    
    properties (SetAccess = private)
        jointNum
        F_centre
        c_angle
        numSides
    end
    
    properties (Access = private)
        vertices
        plane_coeff
        constraintPoints
        % A_F * interaction_F <= 0
        A_F
    end
    
    methods
        function con =  IDConstraintInteractionForceAngleCone(jointNum, F_centre, c_angle, numSides)
            con.jointNum = jointNum;
            con.F_centre = F_centre;
            con.c_angle = c_angle;
            con.numSides = numSides;
            
            con.computeConstraintPoints();
        end
                       
        function updateConstraint(obj, dynamics)
            % F = a + R' * f
            a = dynamics.P'*(dynamics.bodyDynamics.M_b*dynamics.q_ddot + dynamics.bodyDynamics.C_b - dynamics.bodyDynamics.G_b);
            R_T = dynamics.P'*dynamics.V';
            % A_F * F <= 0
            % A_F ( a + R' * f ) <= 0
            obj.A = obj.A_F * R_T(6*obj.jointNum-5:6*obj.jointNum-3, :);
            obj.b = -obj.A_F * a(6*obj.jointNum-5:6*obj.jointNum-3);
        end
        
        function computeConstraintPoints(obj)
            assert(norm(obj.F_centre) ~= 0, 'The F_centre vector cannot equal to zero');
            obj.vertices = cell(1, obj.numSides+1);
            obj.plane_coeff = cell(1, obj.numSides);
            angle_rad = 2*pi/(obj.numSides);
            R = norm(obj.F_centre)*tan(obj.c_angle);
            
            % Fx x + Fy y + Fz z = 0
            if (obj.F_centre(1) ~= 0)
                u = [-obj.F_centre(2)/obj.F_centre(1); 1; 0];
            elseif (obj.F_centre(2) ~= 0)
                u = [0; -obj.F_centre(3)/obj.F_centre(2); 1];
            else
                u = [1; 0; 0];
            end
            v = cross(obj.F_centre, u);
            u = u/norm(u);
            v = v/norm(v);            
            
            for i=1:obj.numSides+1
                obj.vertices{i} = obj.F_centre + R*cos((i-1)*angle_rad)*u + R*sin((i-1)*angle_rad)*v;
            end
            obj.A_F = zeros(obj.numSides, 3);
            
            % The cone is formed by going clock-wise about the centre, and
            % hence the coeff' * F >= 0 means it is inside the cone, so A_F
            % should be negative since standard form is A_F * F <= 0
            for i = 1:obj.numSides
                obj.plane_coeff{i} = cross(obj.vertices{i}, obj.vertices{i+1});
                obj.A_F(i, :) = obj.plane_coeff{i}';
            end
        end
    end
    
end

