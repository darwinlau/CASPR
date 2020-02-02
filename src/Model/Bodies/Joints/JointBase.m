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
    properties (Constant)
        INVALID_TAU = -Inf
    end
    
    properties
        tau                 % Actuator effort for joint (if actuated)
        tau_min             % Minimum actuation (if actuated)
        tau_max             % Maximum actuation (if actuated)
        axis                % Rotation axis for revolute joints / Translation axis for prismatic joints
    end
   
    properties (SetAccess = private)
        type                % Type of joint from JointType enum
        q                   % Joint variable q (generalised coordinates)
        q_dot               % Derivative of q
        q_ddot              % Double derivative of q
        
        q_min               % Minimum joint values of q (set by user)
        q_max               % Maximum joint values of q (set by user)
        q_initial           % The initial value of q for plotting
        
        isActuated = 0      % Is this joint actuated, by default not actuated
        
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
        
        q_dofType           % The type of DoF (translation or rotation) for each q value
        q_default           % The default q
        q_dot_default       % The default q_dot
        q_ddot_default      % The default q_ddot
        q_lb                % The lower bound on joints that are physically meaningful (by definition)
        q_ub                % The upper bound on joints that are physically meaningful (by definition)
    end
    
    methods
        % Updates the joint given the new system kinematics
        function update(obj, q, q_dot, q_ddot)
            obj.q = q;
            obj.q_dot = q_dot;
            obj.q_ddot = q_ddot;
            obj.R_pe = obj.RelRotationMatrix(q);
            obj.r_rel = obj.RelTranslationVector(q);
            obj.S_grad  = obj.RelVelocityMatrixGradient(q);
            obj.S_dot_grad = obj.RelVelocityMatrixDerivGradient(q,q_dot);
        end
        
        % -------
        % Getters and setters
        % -------
        function value = get.q_deriv(obj)
            value = obj.QDeriv(obj.q, obj.q_dot);
        end
        
        function value = get.S_dot(obj)
            % Do we want this here or elsewhere
            value = TensorOperations.VectorProduct(obj.S_grad, obj.q_dot, 3, isa(obj.q,'sym'));
        end
        
        function value = get.S(obj)
            value = obj.RelVelocityMatrix(obj.q);
        end
        
        function set_tau(obj, value)
            if (obj.isActuated)
                obj.tau = value;
            end
        end
        
        function set_tau_min(obj, value)
            if (obj.isActuated)
                obj.tau_min = value;
            end
        end
        
        function set_tau_max(obj, value)
            if (obj.isActuated)
                obj.tau_max = value;
            end
        end
        
        function set.axis(obj, value)
            % Normalize the axis
            obj.axis = value/norm(value);            
        end
        
        % These functions generate trajectory spline for a joint
        % By default it assumes that the interpolation is done on all
        % joints independently. If not, override it in your own joint
        % implementation
        function [q, q_dot, q_ddot] = generateTrajectoryLinearSpline(obj, q_s, q_e, time_vector)
            n_dof = obj.numDofs;
            CASPR_log.Assert(n_dof == length(q_s) && n_dof == length(q_e), 'Length of input states are different to the number of DoFs');
            q = zeros(n_dof, length(time_vector)); 
            q_dot = zeros(n_dof, length(time_vector));
            q_ddot = zeros(n_dof, length(time_vector));
            for i = 1:n_dof
                [q(i,:), q_dot(i,:), q_ddot(i,:)] = Spline.LinearInterpolation(q_s(i), q_e(i), time_vector);
            end
        end
        
        function [q, q_dot, q_ddot] = generateTrajectoryCubicSpline(obj, q_s, q_s_d, q_e, q_e_d, time_vector)
            n_dof = obj.numDofs;
            CASPR_log.Assert(n_dof == length(q_s) && n_dof == length(q_e) && n_dof == length(q_s_d) && n_dof == length(q_e_d), 'Length of input states are different to the number of DoFs');
            q = zeros(n_dof, length(time_vector)); 
            q_dot = zeros(n_dof, length(time_vector));
            q_ddot = zeros(n_dof, length(time_vector));
            for i = 1:n_dof
                [q(i,:), q_dot(i,:), q_ddot(i,:)] = Spline.CubicInterpolation(q_s(i), q_s_d(i), q_e(i), q_e_d(i), time_vector);
            end
        end
        
        function [q, q_dot, q_ddot] = generateTrajectoryQuinticSpline(obj, q_s, q_s_d, q_s_dd, q_e, q_e_d, q_e_dd, time_vector)
            n_dof = obj.numDofs;
            CASPR_log.Assert(n_dof == length(q_s) && n_dof == length(q_e) && n_dof == length(q_s_d) && n_dof == length(q_e_d) && n_dof == length(q_s_dd) && n_dof == length(q_e_dd), 'Length of input states are different to the number of DoFs');
            q = zeros(n_dof, length(time_vector)); 
            q_dot = zeros(n_dof, length(time_vector));
            q_ddot = zeros(n_dof, length(time_vector));
            for i = 1:n_dof
                [q(i,:), q_dot(i,:), q_ddot(i,:)] = Spline.QuinticInterpolation(q_s(i), q_s_d(i), q_s_dd(i), q_e(i), q_e_d(i), q_e_dd(i), time_vector);
            end
        end
        
        function [q, q_dot, q_ddot] = generateTrajectoryParabolicBlend(obj, q_s, q_e, time_vector, time_blend)
            n_dof = obj.numDofs;
            CASPR_log.Assert(n_dof == length(q_s) && n_dof == length(q_e), 'Length of input states are different to the number of DoFs');
            q = zeros(n_dof, length(time_vector)); 
            q_dot = zeros(n_dof, length(time_vector));
            q_ddot = zeros(n_dof, length(time_vector));
            for i = 1:n_dof
                [q(i,:), q_dot(i,:), q_ddot(i,:)] = Spline.ParabolicBlend(q_s(i), q_e(i), time_vector, time_blend);
            end
        end
	end
        
    methods (Static)
        % Create a new joint
        function j = CreateJoint(jointType, q_initial, q_min, q_max, isActuated, axis)
            CASPR_log.Assert(nargin >= 5, 'Not enough input arguments');
            switch jointType
                case JointType.R_X
                    j = RevoluteX;
                case JointType.R_Y
                    j = RevoluteY;
                case JointType.R_Z
                    j = RevoluteZ;
                case JointType.R_AXIS
                    j = RevoluteAxis;
                    CASPR_log.Assert(nargin >= 6, 'Rotation Axis must be defined.');
                    j.axis = axis;
                case JointType.U_XY
                    j = UniversalXY;
                case JointType.U_YZ
                    j = UniversalYZ;
                case JointType.U_XZ
                    j = UniversalXZ;
                case JointType.P_X
                    j = PrismaticX;
                case JointType.P_AXIS
                    j = PrismaticAxis;
                    CASPR_log.Assert(nargin >= 6, 'Translation Axis must be defined.');
                    j.axis = axis;
                case JointType.P_XY
                    j = PrismaticXY;
                case JointType.P_XZ
                    j = PrismaticXZ;
                case JointType.PLANAR_XY
                    j = PlanarXY;
                case JointType.PLANAR_YZ
                    j = PlanarYZ;
                case JointType.PLANAR_XZ
                    j = PlanarXZ;
                case JointType.S_EULER_XYZ
                    j = SphericalEulerXYZ;
                case JointType.S_FIXED_XYZ
                    j = SphericalFixedXYZ;
%                 case JointType.S_QUATERNION
%                     j = SphericalQuaternion;
                case JointType.T_XYZ
                    j = TranslationalXYZ;
%                 case JointType.SPATIAL_QUATERNION
%                     j = SpatialQuaternion;
                case JointType.SPATIAL_EULER_XYZ
                    j = SpatialEulerXYZ;                
                otherwise
                    CASPR_log.Print('Joint type is not defined', CASPRLogLevel.ERROR);
            end
            j.type = jointType;
            
            % Initialise the values
            if (~isempty(q_initial))
                j.q_initial = q_initial;
            else
                j.q_initial = j.q_default;
            end            
            if (~isempty(q_min))
                j.q_min = q_min;
            else
                j.q_min = j.q_lb;
            end            
            if (~isempty(q_max))
                j.q_max = q_max;
            else
                j.q_max = j.q_ub;
            end
            j.isActuated = isActuated;
            
            j.update(j.q_initial, j.q_dot_default, j.q_ddot_default);
            
            if j.isActuated
                j.tau = zeros(j.numDofs, 1);
            else       
                j.tau = [];
            end
            j.tau_min = -Inf*ones(j.numDofs, 1);
            j.tau_max = Inf*ones(j.numDofs, 1);
        end
        
        % Load new xml objects
        function j = LoadXmlObj(xmlObj)
            jointType = JointType.(char(xmlObj.getAttribute('type')));
            q_initial = XmlOperations.StringToVector(char(xmlObj.getAttribute('q_initial')));
            CASPR_log.Assert(sum(isnan(q_initial))==0,'q_initial must be defined in the xml file');
            q_min = XmlOperations.StringToVector(char(xmlObj.getAttribute('q_min')));
            CASPR_log.Assert(sum(isnan(q_min))==0,'q_min must be defined in the xml file');
            q_max = XmlOperations.StringToVector(char(xmlObj.getAttribute('q_max')));
            CASPR_log.Assert(sum(isnan(q_max))==0,'q_max must be defined in the xml file'); 
            
            % Actuated joints
            if (~isempty(xmlObj.getAttribute('actuated') == '') && strcmp(xmlObj.getAttribute('actuated'), 'true'))
                isActuated = 1;        
            else
                isActuated = 0;
            end
            
            % Rotation axis 
            if (~isempty(xmlObj.getAttribute('axis') == ''))
                CASPR_log.Assert(jointType==JointType.R_AXIS || jointType==JointType.P_AXIS, ...
                    'Axis definition is only allowed for R_AXIS/P_AXIS joints.'); 
                axis = XmlOperations.StringToVector(char(xmlObj.getAttribute('axis')));
                CASPR_log.Assert(length(axis)==3,'Dimension of rotation axis must be 3.'); 
                CASPR_log.Assert(norm(axis)~=0,'Axis must not be a zero vector.');
                j = JointBase.CreateJoint(jointType, q_initial, q_min, q_max, isActuated, axis);
            else
                j = JointBase.CreateJoint(jointType, q_initial, q_min, q_max, isActuated);
            end
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
    end
end

