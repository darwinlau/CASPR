classdef (Abstract) Joint < handle
    %JOINT Summary of this class goes here
    %   Detailed explanation goes here
   
    properties (SetAccess = private)
        type                % Type of joint from JointType enum
        q
        q_dot
        q_ddot
        
        % Dependent but stored values (hence private set)
        R_pe
        r_rel        
        S
        S_dot
    end
    
    properties (Dependent)
        % This is useful particularly if the derivative of q is not the
        % same as q_dot, but in most cases they are the same
        q_deriv
    end
    
    properties (Abstract, Constant)
        numDofs
        numVars
        
        q_default
        q_dot_default
        q_ddot_default
        q_lb
        q_ub
    end
    
    methods
        function update(obj, q, q_dot, q_ddot)
            obj.q = q;
            obj.q_dot = q_dot;
            obj.q_ddot = q_ddot;
            obj.R_pe = obj.RelRotationMatrix(q);
            obj.r_rel = obj.RelTranslationVector(q);
            obj.S = obj.RelVelocityMatrix(q);
            obj.S_dot = obj.RelVelocityMatrixDeriv(q, q_dot);
        end
        
        function value = get.q_deriv(obj)
            value = obj.QDeriv(obj.q, obj.q_dot);
        end
	end
        
    methods (Static)
        function j = CreateJoint(jointType)
            switch jointType
                case JointType.R_X
                    j = RevoluteX;
                case JointType.R_Y
                    j = RevoluteY;
                case JointType.R_Z
                    j = RevoluteZ;
                case JointType.U_XY
                    j = UniversalXY;
                case JointType.PLANAR_XY
                    j = PlanarXY;
                case JointType.S_EULER_XYZ
                    j = SphericalEulerXYZ;
                case JointType.S_FIXED_XYZ
                    j = SphericalFixedXYZ;
                case JointType.SPHERICAL
                    j = Spherical;
                case JointType.T_XYZ
                    j = TranslationalXYZ;
                case JointType.SPATIAL
                    j = Spatial;
                case JointType.SPATIAL_EULER_XYZ
                    j = SpatialEulerXYZ;
                otherwise
                    error('Joint type is not defined');
            end
            j.type = jointType;
            j.update(j.q_default, j.q_dot_default, j.q_ddot_default);
        end
        
        % Perform a simple first order integral
        function q = QIntegrate(q0, q_dot, dt)
            q = q0 + q_dot * dt;
        end
        
        % This is useful particularly if the derivative of q is not the
        % same as q_dot, but in most cases they are the same
        function q_deriv = QDeriv(~, q_dot)
            q_deriv = q_dot;
        end
    end
    
    methods (Abstract, Static)
        % Relative rotation matrix ^p_eR : 
        % Where "p" is previous frame frame and "e" is end-effector frame
        % Hence, vector_in_p = ^p_eR * vector_in_e
        R_pe = RelRotationMatrix(q)
        % Relative translation of the joint
        r_rel = RelTranslationVector(q)
        % Relationship between x_{rel}'
        S = RelVelocityMatrix(q)
        S_dot = RelVelocityMatrixDeriv(q, q_dot)
        
        % Generates trajectory
        [q, q_dot, q_ddot] = GenerateTrajectory(q_s, q_s_d, q_s_dd, q_e, q_e_d, q_e_dd, total_time, time_step)
    end
    
end

