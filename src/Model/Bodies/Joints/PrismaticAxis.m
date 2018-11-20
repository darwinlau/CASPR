% translation joint in the defined axis
%
% Author        : Dominic Chan
% Created       : 2018
% Description   :
classdef PrismaticAxis < JointBase        
    properties (Constant = true)
        numDofs = 1;
        numVars = 1;
        q_default = [0];
        q_dot_default = [0];
        q_ddot_default = [0];
        q_lb = [-Inf];
        q_ub = [Inf];
        q_dofType = [DoFType.TRANSLATION];
    end
    
    properties (Dependent)
        d;
        d_dot;
    end
    
    methods 
        % -------
        % Getters
        % -------
        function value = get.d(obj)
            value = obj.GetD(obj.q);
        end
        
        function value = get.d_dot(obj)
            value = obj.GetD(obj.q_dot);
        end
        
        % Get the relative translation vector
        function r_rel = RelTranslationVector(obj, q)
            value = PrismaticAxis.GetD(q);
            axis = obj.axis;
            r_rel = value*axis;
        end  
        
        % Generate the S matrix
        function [S] = RelVelocityMatrix(obj, ~)
            S = [obj.axis; 0; 0; 0];
        end
    end
    
    methods (Static)
        % Get the relative rotation matrix
        function R_pe = RelRotationMatrix(~)
            R_pe = eye(3);
        end
                
        % Generate the S gradient tensor
        function [S_grad] = RelVelocityMatrixGradient(~)
            S_grad = zeros(6,1,1);
        end
        
        % Generate the \dot{S} gradient tensor
        function [S_dot_grad] = RelVelocityMatrixDerivGradient(~,~)
            S_dot_grad = zeros(6,1,1);
        end
                
        % Generate the N matrix for the joint
        function [N_j,A] = QuadMatrix(~)
            N_j = 0;
            A = zeros(6,1);
        end
        
        % Get variables from the gen coordinates
        function d = GetD(q)
            d = q(1);
        end
    end
end

