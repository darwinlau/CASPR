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
    
    properties (Abstract, Constant)
        numDofs
    end
    
    methods
        
        function update(obj, q, q_dot, q_ddot)
            obj.q = q;
            obj.q_dot = q_dot;
            obj.q_ddot = q_ddot;
            obj.R_pe = obj.RelRotationMatrix(q);
            obj.r_rel = obj.RelTranslation(q);
            obj.S = obj.RelJointMatrix(q);
            obj.S_dot = obj.RelJointMatrixD(q, q_dot);
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
                case JointType.PLANAR_XY
                    j = PlanarXY;
                case JointType.S_EULER_XYZ
                    j = SphericalEulerXYZ;
                case JointType.S_FIXED_XYZ
                    j = SphericalFixedXYZ;
                otherwise
                    error('Joint type is not defined');
            end
            j.type = jointType;
            j.update(zeros(j.numDofs, 1), zeros(j.numDofs, 1), zeros(j.numDofs, 1))
        end
    end
    
    methods (Abstract, Static)
        % Relative rotation matrix ^p_eR : 
        % Where "p" is previous frame frame and "e" is end-effector frame
        % Hence, vector_in_p = ^p_eR * vector_in_e
        R_pe = RelRotationMatrix(q)
        % Relative translation of the joint
        r_rel = RelTranslation(q)
        % Relationship between x_{rel}'
        S = RelJointMatrix(q)
        S_dot = RelJointMatrixD(q, q_dot)
    end
    
end

