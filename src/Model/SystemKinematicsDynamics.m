% System kinematics and dynamics of the entire cable robot system
%
% Please cite the following paper when using this for multilink cable
% robots:
% D. Lau, D. Oetomo, and S. K. Halgamuge, "Generalized Modeling of
% Multilink Cable-Driven Manipulators with Arbitrary Routing Using the
% Cable-Routing Matrix," IEEE Trans. Robot., vol. 29, no. 5, pp. 1102–1113,
% Oct. 2013.
%
% Author        : Darwin LAU
% Created       : 2011
% Description	:
%	Data structure that represents the both the kinematics and dynamics of 
% the cable robot system. This object inherits from SystemKinematics, the
% kinematics and dynamics are stored within SystemKinematicsBodies, 
% SystemKinematicsCables, SystemDynamicsBodies and SystemDynamicsCables.
%   This class also provides direct access to a range of data and matrices. 
% In addition to that from SystemKinematics, the matrices and vectors for
% the equations of motion (M, C, G etc.), the interaction wrenches and its 
% magnitudes are available.
classdef SystemKinematicsDynamics < SystemKinematics
    
    properties (SetAccess = protected)            
        bodyDynamics              % SystemDynamicsBodies object
        cableDynamics             % SystemDynamicsCables object
    end
    
    properties (Constant)
        GRAVITY_CONSTANT = 9.81;
    end
    
    properties (Dependent)
        % The equations of motion with the interaction wrench w_p
        % P^T ( M_b * q_ddot + C_b ) = P^T ( G_b - V^T f ) + w_p
        interactionWrench               % Joint interaction wrenches (w_p)
        interactionForceMagnitudes      % Magnitudes of the interaction force at each joint
        interactionMomentMagnitudes     % Magnitudes of the interaction moments at each joint
        
        % Since S^T w_p = 0 and S^T P^T = W^T
        % W^T ( M_b * q_ddot + C_b ) = W^T ( G_b - V^T f )
        % The equations of motion can then be expressed in the form
        % M * q_ddot + C + G = - L^T f
        M
        C
        G
        
%         B                         % Linearised B matrix
        
        % Instead of q_ddot being passed as input to the system, it could
        % also be determined from:
        % M * q_ddot + C + G = - L^T f
        q_ddot_dynamics
        
        % The set of cable forces f = [f_1; f_2; ... ; f_m]
        cableForces               % cable forces 
    end
    
    methods (Static)
        function b = LoadXmlObj(body_xmlobj, cable_xmlobj)
            b = SystemKinematicsDynamics;
            b.bodyKinematics = SystemKinematicsBodies.LoadXmlObj(body_xmlobj);
            b.bodyDynamics = SystemDynamicsBodies.LoadXmlObj(body_xmlobj);
            b.cableKinematics = SystemKinematicsCables.LoadXmlObj(cable_xmlobj, b.numLinks);
            b.cableDynamics = SystemDynamicsCables.LoadXmlObj(cable_xmlobj);
            b.update(b.bodyKinematics.q_default, b.bodyKinematics.q_dot_default, b.bodyKinematics.q_ddot_default);
        end
    end
    
    methods
        function b = SystemKinematicsDynamics()
        end
        
        % Function updates the kinematics and dynamics of the bodies and 
        % cables of the system with the joint state (q, q_dot and q_ddot)
        function update(obj, q, q_dot, q_ddot)         
            update@SystemKinematics(obj, q, q_dot, q_ddot);
            obj.bodyDynamics.update(obj.bodyKinematics);
            obj.cableDynamics.update(obj.cableKinematics, obj.bodyKinematics);
        end
        
        % Function computes the interaction wrench between the joint of the
        % links. 
        % M_b*q_ddot + C_b = G_b + 
        function value = get.interactionWrench(obj)
            value = obj.P.'*(obj.V.'*obj.cableDynamics.forces + obj.bodyDynamics.M_b*obj.q_ddot + obj.bodyDynamics.C_b - obj.bodyDynamics.G_b);
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
            value =  obj.M\(-obj.L.'*obj.cableDynamics.forces - obj.C - obj.G);
        end
        
        function value = get.M(obj)
            value = obj.bodyDynamics.M;
        end
        
        function value = get.C(obj)
            value = obj.bodyDynamics.C;
        end
        
        function value = get.G(obj)
            value = obj.bodyDynamics.G;
        end
        
        function set.cableForces(obj, f)
            obj.cableDynamics.forces = f;
        end
        
        function value = get.cableForces(obj)
            value = obj.cableDynamics.forces;
        end
    end
    
end

