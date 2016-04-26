% System kinematics and dynamics of the entire cable robot system
%
% Please cite the following paper when using this for multilink cable
% robots:
% D. Lau, D. Oetomo, and S. K. Halgamuge, "Generalized Modeling of
% Multilink Cable-Driven Manipulators with Arbitrary Routing Using the
% Cable-Routing Matrix," IEEE Trans. Robot., vol. 29, no. 5, pp. 1102-1113,
% Oct. 2013.
%
% Author        : Darwin LAU
% Created       : 2011
% Description    :
%    Data structure that represents the both the kinematics and dynamics of
% the cable robot system. This object inherits from SystemKinematics, the
% kinematics and dynamics are stored within SystemKinematicsBodies,
% SystemKinematicsCables, SystemDynamicsBodies and SystemDynamicsCables.
%   This class also provides direct access to a range of data and matrices.
% In addition to that from SystemKinematics, the matrices and vectors for
% the equations of motion (M, C, G etc.), the interaction wrenches and its
% magnitudes are available.
classdef SystemModel < handle
    properties (SetAccess = protected)
        bodyModel               % SystemModelBodies object
        cableModel              % SystemModelCables object
    end

    properties (Constant)
        GRAVITY_CONSTANT = 9.81;
    end

    properties (Dependent)
        % Number of variables
        numLinks                % Number of links
        numDofs                 % Number of degrees of freedom
        numDofVars              % Number of variables for the DoFs
        numOPDofs               % Number of operational space degrees of freedom
        numCables               % Number of cables
        
        % Cable Lengths
        cableLengths            % Vector of cable lengths
        cableLengthsDot         % Vector of cable length derivatives

        % Jacobians
        L                       % cable to joint Jacobian matrix L = VW
        J                       % joint to operational space Jacobian matrix
        J_dot                   % derivative of J
        K                       % The stiffness matrix
        
        % Hessians
        L_grad                  % The gradient of the jacobian L

        % Generalised coordinates
        q                       % Generalised coordinates state vector
        q_deriv                 % Generalised coordinates time derivative (for special cases q_dot does not equal q_deriv)
        q_dot                   % Generalised coordinates derivative
        q_ddot                  % Generalised coordinates double derivative
        
        % Operational space coordinates
        y                       % Operational space coordinate vector
        y_dot                   % Operational space coordinate derivative
        y_ddot                  % Operational space coordinate double derivative
                
        % The equations of motion with the interaction wrench w_p
        % P^T ( M_b * q_ddot + C_b ) = P^T ( G_b - V^T f ) + w_p
        interactionWrench               % Joint interaction wrenches (w_p)
        interactionForceMagnitudes      % Magnitudes of the interaction force at each joint
        interactionMomentMagnitudes     % Magnitudes of the interaction moments at each joint

        % Since S^T w_p = 0 and S^T P^T = W^T
        % W^T ( M_b * q_ddot + C_b ) = W^T ( G_b - V^T f )
        % The equations of motion can then be expressed in the form
        % M * q_ddot + C + G + W_e = - L^T f
        M
        C
        G
        W_e

        % Instead of q_ddot being passed as input to the system, it could
        % also be determined from:
        % M * q_ddot + C + G + W_e = - L^T f
        q_ddot_dynamics
        
        % The cable force information
        cableForces                 % cable forces
        forcesMin                   % vector of min forces from cables
        forcesMax                   % vector of max forces from cables
        forcesInvalid               % vector of invalid forces
    end

    methods (Static)
        function b = LoadXmlObj(body_xmlobj, cable_xmlobj)
            b               =   SystemModel();
            b.bodyModel     =   SystemModelBodies.LoadXmlObj(body_xmlobj);
            b.cableModel    =   SystemModelCables.LoadXmlObj(cable_xmlobj, b.bodyModel);
            b.update(b.bodyModel.q_default, b.bodyModel.q_dot_default, b.bodyModel.q_ddot_default, zeros(b.numDofs,1));
        end
    end

    methods
        function b = SystemModel()
        end

        % Function updates the kinematics and dynamics of the bodies and
        % cables of the system with the joint state (q, q_dot and q_ddot)
        function update(obj, q, q_dot, q_ddot, w_ext)
            obj.bodyModel.update(q, q_dot, q_ddot, w_ext);
            obj.cableModel.update(obj.bodyModel);
        end
        
        % The following functions get the dependent variable values
        function value = get.numLinks(obj)
            value = obj.bodyModel.numLinks;
        end

        function value = get.numDofs(obj)
            value = obj.bodyModel.numDofs;
        end
        
        function value = get.numOPDofs(obj)
            value = obj.bodyModel.numOPDofs;
        end

        function value = get.numDofVars(obj)
            value = obj.bodyModel.numDofVars;
        end

        function value = get.numCables(obj)
            value = obj.cableModel.numCables;
        end

        function value = get.cableLengths(obj)
            value = zeros(obj.numCables,1);
            for i = 1:obj.numCables
                value(i) = obj.cableModel.cables{i}.length;
            end
        end

        function value = get.cableLengthsDot(obj)
            value = obj.L*obj.q_dot;
        end

        function value = get.q(obj)
            value = obj.bodyModel.q;
        end
        
        function value = get.q_deriv(obj)
            value = obj.bodyModel.q_deriv;
        end

        function value = get.q_dot(obj)
            value = obj.bodyModel.q_dot;
        end

        function value = get.q_ddot(obj)
            value = obj.bodyModel.q_ddot;
        end

        function value = get.L(obj)
            value = obj.cableModel.V*obj.bodyModel.W;
        end
        
        function value = get.L_grad(obj)
            if(isempty(obj.bodyModel.W_grad))
                if(~obj.bodyModel.occupied.hessian)
                    obj.bodyModel.occupied.hessian = true;
                    obj.bodyModel.update_hessian();
                    obj.cableModel.update_hessian(obj.bodyModel);
                end
            end
            is_symbolic = isa(obj.q,'sym');
            value = TensorOperations.LeftMatrixProduct(obj.cableModel.V,obj.bodyModel.W_grad,is_symbolic) + TensorOperations.RightMatrixProduct(obj.cableModel.V_grad,obj.bodyModel.W,is_symbolic);
        end
        
        function value = get.K(obj)
            is_symbolic = isa(obj.q,'sym');
            value = obj.L.'*obj.cableModel.K*obj.L + TensorOperations.VectorProduct(obj.L_grad,obj.cableForces,1,is_symbolic);
        end
        
        function value = get.J(obj)
            value = obj.bodyModel.J;
        end
        
        function value = get.J_dot(obj)
            value = obj.bodyModel.J_dot;
        end
        
        function value = get.y(obj)
            value = obj.bodyModel.y;
        end
        
        function value = get.y_dot(obj)
            value = obj.bodyModel.y_dot;
        end
        
        function value = get.y_ddot(obj)
            value = obj.bodyModel.y_ddot;
        end
        
        % Function computes the interaction wrench between the joint of the
        % links.
        % M_b*q_ddot + C_b = G_b +
        function value = get.interactionWrench(obj)
            value = obj.bodyModel.P.'*(obj.cableModel.V.'*obj.cableModel.forces + obj.bodyModel.M_b*obj.q_ddot + obj.bodyModel.C_b - obj.bodyModel.G_b);
        end


        function value = get.interactionForceMagnitudes(obj)
            vector = obj.interactionWrench;
            mag = zeros(obj.numLinks,1);
            for k = 1:obj.numLinks
                mag(k) = norm(vector(6*k-5:6*k-3),2);
            end
            value = mag;
        end

        function value = get.interactionMomentMagnitudes(obj)
            vector = obj.interactionWrench;
            mag = zeros(obj.numLinks,1);
            for k = 1:obj.numLinks
                mag(k) = norm(vector(6*k-2:6*k),2);
            end
            value = mag;
        end

%         function value = get.jointForceAngles(obj)
%             vector = obj.jointWrenches;
%             angle = zeros(obj.numLinks,1);
%             for k = 1:obj.numLinks
%                 angle(k) = atan(norm(vector(6*k-5:6*k-4),2)/abs(vector(6*k-3)))*180/pi;
%             end
%             value = angle;
%         end

        function value = get.q_ddot_dynamics(obj)
            obj.bodyModel.q_ddot = obj.M\(-obj.L.'*obj.cableModel.forces - obj.C - obj.G - obj.W_e);
            value = obj.q_ddot;
        end

        function value = get.M(obj)
            value = obj.bodyModel.M;
        end

        function value = get.C(obj)
            value = obj.bodyModel.C;
        end

        function value = get.G(obj)
            value = obj.bodyModel.G;
        end
        
        function value = get.W_e(obj)
            value = obj.bodyModel.W_e;
        end

        function set.cableForces(obj, f)
            obj.cableModel.forces = f;
        end

        function value = get.cableForces(obj)
            value = obj.cableModel.forces;
        end
        
        function value = get.forcesMin(obj)
           value = obj.cableModel.forcesMin; 
        end
        
        function value = get.forcesMax(obj)
            value = obj.cableModel.forcesMax; 
        end
        
        function value = get.forcesInvalid(obj)
            value = obj.cableModel.forcesInvalid; 
        end
        
        function loadOpXmlObj(obj,op_space_xmlobj)
            obj.bodyModel.loadOpXmlObj(op_space_xmlobj);
        end
    end
end
