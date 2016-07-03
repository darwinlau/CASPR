% Constraint definition for the interaction force of a particular joint
% where the constraint is a cone constraint that the interaction force must
% lie within.
%
% Please cite the following paper when using this:
% D. Lau, D. Oetomo, and S. K. Halgamuge, "Inverse Dynamics of Multilink
% Cable-Driven Manipulators With the Consideration of Joint Interaction
% Forces and Moments," IEEE Trans. Robot., vol. 31, no. 2, pp. 479�488, 2015.
%
% Author        : Darwin LAU
% Created       : 2016
% Description	: The constraint is constructed by the joint number, the
% centre vector for the interaction force and the critical angle of the
% cone. This constraint should be a quadratic constraint, but it is
% possible to represent it as a set of linear constraints. Hence the number
% of sides for the approximated linear cone must also be specified.
classdef IDConstraintInteractionForceAngleCone < IDConstraintLinear
    properties (SetAccess = private)
        jointNum % The number of joints that the constraints act on
        F_centre % The central force
        c_angle  % The constrained angle
        numSides % The number of sides for the linear cone
    end

    properties (Access = private)
        vertices            % The number of vertices
        plane_coeff
        constraintPoints    % The number of points that are constrained
        % A_F * interaction_F <= 0
        A_F
    end

    methods
        % The constraint constructor. Take in the number of joints, central
        % forces, constraint angle and number of sides.
        function con =  IDConstraintInteractionForceAngleCone(jointNum, F_centre, c_angle, numSides)
            con.jointNum = jointNum;
            con.F_centre = F_centre;
            con.c_angle = c_angle;
            con.numSides = numSides;

            con.computeConstraintPoints();
        end

        % Updates the constraints to match the dynamics.
        function updateConstraint(obj, dynamics)
            % F = a + R' * f
            a = dynamics.bodyModel.P'*(dynamics.bodyModel.M_b*dynamics.q_ddot + dynamics.bodyModel.C_b - dynamics.bodyModel.G_b);
            R_T = dynamics.bodyModel.P'*dynamics.cableModel.V';
            % A_F * F <= 0
            % A_F ( a + R' * f ) <= 0
            % A_F ( a + R_passive' * f_passive + R_active' * f_active ) <= 0
            obj.A = obj.A_F * R_T(6*obj.jointNum-5:6*obj.jointNum-3, dynamics.cableModel.cableIndicesActive);
            obj.b = -obj.A_F * (a(6*obj.jointNum-5:6*obj.jointNum-3) + R_T(6*obj.jointNum-5:6*obj.jointNum-3, dynamics.cableModel.cableIndicesPassive) * dynamics.cableForcesPassive);
        end

        % Compute the constaint points.
        function computeConstraintPoints(obj)
            CASPR_log.Assert(norm(obj.F_centre) ~= 0, 'The F_centre vector cannot equal to zero');
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
