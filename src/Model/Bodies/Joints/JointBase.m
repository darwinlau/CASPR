% Base class for the type of joint for a link
%
% Author        : Darwin LAU
% Created       : 2014
% Description   :
%   All user-defined types of joints should implement this base class and
%   define the following:
%       - The rotation matrix from previous frame to this joint's frame
%       - The translation vector from previous frame to this joint's frame
%       - The relative velocity relationship (S matrix)
%       - The derivative of the velocity relationship (S_dot)
%       - The method to generate a trajectory for the joint
%   Any new types of joints need to be added to the JointType enum and also
%   added to the CreateJoint method.
classdef (Abstract) JointBase < handle
    properties (SetAccess = private)
        type                % Type of joint from JointType enum
        q                   % Joint variable q (generalised coordinates)
        q_dot               % Derivative of q
        q_ddot              % Double derivative of q
        q_initial           % The initial value of q for plotting
        
        % Dependent but stored values (hence private set)
        R_pe                % The relative rotation
        r_rel               % The relative translation      
        S_grad              % The gradient of the S matrix
        S_dot_grad          % The gradient of the \dot{S} matrix
    end
    
    properties (Dependent)
        % This is useful particularly if the derivative of q is not the
        % same as q_dot, but in most cases they are the same
        q_deriv
        S_dot
        S
    end
    
    properties (Abstract, Constant)
        numDofs             % The number of degrees of freedom
        numVars             % The number of variables to describe the degrees of freedom
        
        q_default           % The default q
        q_dot_default       % The default q_dot
        q_ddot_default      % The default q_ddot
        q_lb                % The lower bound on joints
        q_ub                % The upper bound on joints
    end
    
    methods
        % Updates the joint given the new system kinematics
        function update(obj, q, q_dot, q_ddot)
            obj.q = q;
            obj.q_dot = q_dot;
            obj.q_ddot = q_ddot;
            obj.R_pe = obj.RelRotationMatrix(q);
            obj.r_rel = obj.RelTranslationVector(q);
%             obj.S = obj.RelVelocityMatrix(q);
            obj.S_grad  = obj.RelVelocityMatrixGradient(q);
            obj.S_dot_grad = obj.RelVelocityMatrixDerivGradient(q,q_dot);
%             obj.S_dot = obj.RelVelocityMatrixDeriv(q, q_dot);
        end
        
        % -------
        % Getters
        % -------
        function value = get.q_deriv(obj)
            value = obj.QDeriv(obj.q, obj.q_dot);
        end
        
        function value = get.S_dot(obj)
            % Do we want this here or elsewhere
            value = TensorOperations.VectorProduct(obj.S_grad,obj.q_dot,3,isa(obj.q,'sym'));
        end
        
        function value = get.S(obj)
            value = obj.RelVelocityMatrix(obj.q);
        end
	end
        
    methods (Static)
        % Create a new joint
        function j = CreateJoint(jointType,q_initial)
            switch jointType
                case JointType.R_X
                    j = RevoluteX;
                case JointType.R_Y
                    j = RevoluteY;
                case JointType.R_Z
                    j = RevoluteZ;
                case JointType.U_XY
                    j = UniversalXY;
                case JointType.U_YZ  %%
                    j = UniversalYZ;
                case JointType.P_XY
                    j = PrismaticXY;
                case JointType.PLANAR_XY
                    j = PlanarXY;
                case JointType.PLANAR_YZ
                    j = PlanarYZ;
                case JointType.PLANAR_XZ %%
                    j = PlanarXZ;
                case JointType.S_EULER_XYZ
                    j = SphericalEulerXYZ;
                case JointType.S_FIXED_XYZ
                    j = SphericalFixedXYZ;
                case JointType.S_QUATERNION
                    j = SphericalQuaternion;
                case JointType.T_XYZ
                    j = TranslationalXYZ;
                case JointType.SPATIAL_QUATERNION
                    j = SpatialQuaternion;
                case JointType.SPATIAL_EULER_XYZ
                    j = SpatialEulerXYZ;
                otherwise
                    CASPR_log.Print('Joint type is not defined',CASPRLogLevel.ERROR);
            end
            j.type = jointType;
            if(nargin == 2)
                j.q_initial = q_initial;
                j.update(j.q_initial, j.q_dot_default, j.q_ddot_default);
            else
                j.q_initial = j.q_default;
                j.update(j.q_initial, j.q_dot_default, j.q_ddot_default);
            end
        end
        
        % Load new xml objects
        function j = LoadXmlObj(xmlObj)
            jointType = JointType.(char(xmlObj.getAttribute('type')));
            q_initial = XmlOperations.StringToVector(char(xmlObj.getAttribute('q_initial')));
            assert(sum(isnan(q_initial))==0,'q_initial must be defined in the xml file');
            j = JointBase.CreateJoint(jointType, q_initial);
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
        S_grad = RelVelocityMatrixGradient(q)
        S_dot_grad = RelVelocityMatrixDerivGradient(q,q_dot)
        
        % Generates trajectory
        [q, q_dot, q_ddot] = GenerateTrajectory(q_s, q_s_d, q_s_dd, q_e, q_e_d, q_e_dd, total_time, time_step)
    end
end

