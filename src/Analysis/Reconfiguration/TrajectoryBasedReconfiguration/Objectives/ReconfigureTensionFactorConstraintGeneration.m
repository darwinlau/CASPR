classdef ReconfigureTensionFactorConstraintGeneration < handle
    %UNTITLED11 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Polynomials;
    end
    properties (Hidden)
        Digit = 1e0;
    end
    methods
        function obj = ReconfigureTensionFactorConstraintGeneration(Kinematic_Model,Cable_Index,sgn)
            %             obj.Polynomials.Inequalities = [];
%             obj.Polynomials.Fun = [];
            
            obj.Polynomials.Equalities = [];
            obj.Polynomials.aux_var = [];
            
            J_bar = Kinematic_Model.L_Removed_Norm;
            slack_cable_length = Kinematic_Model.Cable_Length;           
            
            slack_cable_length(Kinematic_Model.ValidReconfigCableID) = Kinematic_Model.ValidAuxLength;
            
            J = J_bar/diag(slack_cable_length);
            %             J = vpa(J_bar,8);
%             J = vpa(J,5);
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
            
            syms z;
            
            obj.Polynomials.Fun = -z;
            
            if sgn == -1
                T_i = [T_hat_i;-T_hat_0].*Unit_Length(1,:).';
            else
                T_i = [-T_hat_i;T_hat_0].*Unit_Length(1,:).';
            end
            
            for i = 1:numel(T_i)
                for j = 1:numel(Kinematic_Model.ValidAuxLength)
                    T_i(i) = expand(subs(T_i(i),Kinematic_Model.ValidAuxLength(j),Kinematic_Model.Cable_Length_Poly{j}));
                end
            end
            
            T_i_square = T_i;
            
            count = 1;            
            for i = 1:numel(T_i)
                for j = 1:numel(T_i)
                    if i ~= j
                        %                         obj.Polynomials.Inequalities(count) = T_i_square(i)*z*obj.Digit <= T_i_square(j)*obj.Digit;
                        obj.Polynomials.Inequalities(count) = T_i_square(i)*z*obj.Digit - T_i_square(j)*obj.Digit <= 0;
                        count = count +1;
                    end
                end
            end
            
            count = 1;
%             obj.Polynomials.AuxVarRange(count) = -z<=0;
            obj.Polynomials.AuxVarRangeNum(:,count) = [0;1];
            obj.Polynomials.AuxVarRange(count) = z >= 0;
            count = count +1;
            obj.Polynomials.AuxVarRange(count) = z <=1;
            
            
            
            obj.Polynomials.AuxVar = z;
            
            
        end
    end
end