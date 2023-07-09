classdef ReconfigureWrenchClosureConstraintGeneration < handle
    %UNTITLED11 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Polynomials = [];
    end
    properties (Hidden)
        Digit = 1e0;
    end
    methods
        function obj = ReconfigureWrenchClosureConstraintGeneration(Kinematic_Model,Cable_Index,sgn,fixedlength)
            
            
            J_bar = Kinematic_Model.L_Removed_Norm;
            slack_cable_length = Kinematic_Model.Cable_Length;
            obj.Polynomials.MoM = [];
            slack_cable_length(Kinematic_Model.ValidReconfigCableID) = Kinematic_Model.ValidAuxLength;
            
            J = J_bar/diag(slack_cable_length);
            J_s = J(:,Cable_Index{1,1});
            if ~isempty(Cable_Index{1,2})
                J_c = sum(J(:,Cable_Index{1,2}),2);
            else
                J_c = [];
            end
            J_C_sub = ([J_s,J_c]);
            [J_hat,Unit_Length] = numden(J_C_sub);
            [n,m] = size(J_hat);
            T_hat_0 = det(J_hat(1:n,1:n));
            adj_J = adjoint(J_hat(1:n,1:n));
            T_hat_i = adj_J*J_hat(:,end);            
            
            [coe,~]=coeffs(T_hat_0);            
            T_hat_0 = T_hat_0/abs(coe(end))*1e-0;
            T_hat_i = T_hat_i/abs(coe(end))*1e-0;
%             end
            
            
            if sgn == -1
            Necessary_Constraints = [T_hat_i;-T_hat_0];
            else
            Necessary_Constraints = [-T_hat_i;T_hat_0];    
            end
            
            for i = 1:numel(Necessary_Constraints)
                for j = 1:numel(Kinematic_Model.ValidAuxLength)
                Necessary_Constraints(i) = subs(Necessary_Constraints(i),Kinematic_Model.ValidAuxLength(j),Kinematic_Model.Cable_Length_Poly{j});
                end
            end
            
            count = 1;
            for i = 1:numel(Necessary_Constraints)
                if ~isempty(symvar(Necessary_Constraints(i)))
                    obj.Polynomials.Inequalities(count) =  -Necessary_Constraints(i) * obj.Digit <= 0;
                    count = count + 1;
                end                
            end
            
            for i = 1:numel(Kinematic_Model.Cable_Length_Poly)
                obj.Polynomials.Inequalities(count) =  -Kinematic_Model.Cable_Length_Poly{i} <= 0;
                count = count + 1;
            end
% % for UAV only
%             count = 1;
%             for i = 1:numel(Kinematic_Model.Cable_Length_Poly)
%                 obj.Polynomials.Equalities(count) =  Kinematic_Model.Cable_Length_Poly{i} - 0.8878 == 0;
% %                 obj.Polynomials.Inequalities(count) =  Kinematic_Model.Cable_Length_Poly{i} >= 0;
%                 count = count + 1;
%             end
            obj.Polynomials.Equalities = [];
        end
        
    end
end