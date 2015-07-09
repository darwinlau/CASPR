classdef SystemDynamics < SystemKinematics
    %SystemKinematics Contains the information for the kinematic state for
    %the entire cable-driven system
    
    properties (SetAccess = private)            
        bodyDynamics              % SystemDynamicsBodies object
        cableDynamics             % SystemDynamicsCables object
        
    end
    
    properties (Constant)
        GRAVITY_CONSTANT = 9.81;
    end
    
    properties (Dependent)
        % M*q_ddot + C + G = external forces in joint space (external forces, cable forces (-L^T*f))
        M
        C
        G
        
        jointWrenches             % Vector of joint wrenches
        jointForceMagnitudes
        jointForceAngles
        jointMomentMagnitudes
        
        
        
%         B                         % Linearised B matrix
        
        q_ddot_dynamics           % q_ddot from the system dynamics
        
        cableForces               % cable forces 
%         
%         JointInteractionVector      % Vector of interaction forces/moments
%         JointInteractionForceMagnitudes
%         JointInteractionForceAngles
%         JointInteractionMomentMagnitudes
%         
%         JointInteractionForceMagnitudes2
%         JointInteractionMomentMagnitudes2
    end
    
    methods (Static)
    end
    
    methods
        function b = SystemDynamics(bdConstructor, cdConstructor, bkConstructor, ckConstructor)
            % Constructor from the bodies and cables properties
            b@SystemKinematics(bkConstructor, ckConstructor);
            b.bodyDynamics = bdConstructor();
            b.cableDynamics = cdConstructor();
        end
        
        
        function update(obj, q, q_dot, q_ddot)
            % Assign the system states q, q_dot, q_ddot
            % Calls set state for BodyKinematics and CableKinematics, and
            % sets the system Jacobian matrix                
            update@SystemKinematics(obj, q, q_dot, q_ddot);
            obj.bodyDynamics.update(obj.bodyKinematics);
            obj.cableDynamics.update(obj.cableKinematics, obj.bodyKinematics, obj.bodyDynamics);
        end
        
        function value = get.jointWrenches(obj)
            value = obj.P'*(obj.V'*obj.cableDynamics.forces + obj.bodyDynamics.M_b*obj.q_ddot + obj.bodyDynamics.C_b - obj.bodyDynamics.G_b);
        end
        
        
        function value = get.jointForceMagnitudes(obj)
            vector = obj.jointWrenches;
            mag = zeros(obj.numLinks,1);
            for k = 1:obj.numLinks
                mag(k) = norm(vector(6*k-5:6*k-3),2);
            end
            value = mag;
        end
        
        function value = get.jointForceAngles(obj)
            vector = obj.jointWrenches;
            angle = zeros(obj.numLinks,1);
            for k = 1:obj.numLinks
                angle(k) = atan(norm(vector(6*k-5:6*k-4),2)/abs(vector(6*k-3)))*180/pi;
            end
            value = angle;
        end
        
        function value = get.jointMomentMagnitudes(obj)
            
            vector = obj.jointWrenches;
            mag = zeros(obj.numLinks,1);
            for k = 1:obj.numLinks
                mag(k) = norm(vector(6*k-2:6*k),2);
            end
            value = mag;
        end
        
        function value = get.q_ddot_dynamics(obj)
            value =  obj.M\(-obj.L'*obj.cableDynamics.forces - obj.C - obj.G);
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

