classdef ReconfigureMinimumCableForceConstraintGeneration < handle
    %UNTITLED11 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Polynomials;
    end
    properties (Hidden)
        Digit = 1e0;
    end
    methods
        function obj= ReconfigureMinimumCableForceConstraintGeneration(Kinematic_Model,Cable_Index,wrench,sgn,old_x)
            
            %tmp hack
            %             wrench(7:end,:) = [];
            
            J_bar = Kinematic_Model.L_Removed_Norm;
            slack_cable_length = Kinematic_Model.Cable_Length;
   
            slack_cable_length(Kinematic_Model.ValidReconfigCableID) = Kinematic_Model.ValidAuxLength;
            
            J = J_bar/diag(slack_cable_length);
            
            J_s = J(:,Cable_Index{1,1});
            if ~isempty(Cable_Index{1,2})
                J_c = sum(J(:,Cable_Index{1,2}),2);
            else
                J_c = [];
            end
            
            J_C_sub = [J_s,J_c,-wrench];
            
            if ~isempty(wrench)
                [J_hat,Unit_Length] = numden(J_C_sub(:,1:end-1));
                J_hat(:,end+1) = -wrench;
                Unit_Length(:,end+1) = 1;
            else
                [J_hat,Unit_Length] = numden(J_C_sub);
            end
            
            [n,m] = size(J_hat);
            T_hat_0 = det(J_hat(1:n,1:n));
            adj_J = adjoint(J_hat(1:n,1:n));
            T_hat_i = adj_J*J_hat(:,end);
            
%             T_i = T_hat_i;
            T_hat_i = T_hat_i.*Unit_Length(1,1:n).';
            
            
            [coe,~]=coeffs(T_hat_0);
%             T_hat_0 = T_hat_0/abs(coe(end));
%             T_hat_i = T_hat_i/abs(coe(end));
            
            
            T_i = [T_hat_i;T_hat_0];
            
%             min_coe = Inf;
            for i = 1:numel(T_i)
                for j = 1:numel(Kinematic_Model.ValidAuxLength)
                    T_i(i) = expand(subs(T_i(i),Kinematic_Model.ValidAuxLength(j),Kinematic_Model.Cable_Length_Poly{j}));
                end
%                 [coe,~]=coeffs(T_i(i));
%                 T_i(i) = T_i(i)/abs(coe(end))*obj.Digit;
            end
            
            %             if sgn == -1
            %                 obj.Polynomials.Fun = (sum((T_i(1:end-1))) - (-T_i(end) ))* obj.Digit ;
            %             else
            %                 obj.Polynomials.Fun = ((sum((-T_i(1:end-1)))) - (T_i(end) )) * obj.Digit ;
            %             end
            %
            %             obj.Polynomials.Equalities = [];
            %             obj.Polynomials.Inequalities = [];
            %             obj.Polynomials.AuxVar = [];
            %             obj.Polynomials.AuxVarRange = [];
            %             obj.Polynomials.AuxVarRangeNum = [];
            
            %%
%             syms z;
%             obj.Polynomials.AuxVar = z;
%             obj.Polynomials.AuxVar = [];
            obj.Polynomials.Inequalities = [];
%             obj.Polynomials.AuxVarRange = [];
%             obj.Polynomials.AuxVarRangeNum = [];
            obj.Polynomials.Equalities = [];
            syms z;
            obj.Polynomials.AuxVar = z;
            obj.Polynomials.Fun = -z;
            if sgn == -1
                obj.Polynomials.Inequalities = ( sum( T_i(1:end-1) )  - (-T_i(end)*(old_x - z)))  <= 0;
%                 obj.Polynomials.Fun = (sum((T_i(1:end-1))) - (-T_i(end)))*obj.Digit;
%                 obj.Polynomials.Inequalities = ( sum( (T_i(1:end-1)) ) - T_i(end)*(old_x - 10) )*obj.Digit <= 0;
%                 obj.Polynomials.Fun = (sum((T_i(1:end-1))) - (-T_i(end))*old_x)* obj.Digit;
%                 obj.Polynomials.Fun = z;
%                 obj.Polynomials.Inequalities = (sum((T_i(1:end-1))) - (-T_i(end))*old_x - 10)*obj.Digit <= 0;
%                 obj.Polynomials.Inequalities = (sum((T_i(1:end-1))) - z*(-T_i(end)))<= 0;
%                 obj.Polynomials.AuxVarRange(1) = z >= 0;
%                 obj.Polynomials.AuxVarRangeNum = [0;1000000];
            else
                obj.Polynomials.Inequalities = ( sum( -T_i(1:end-1) )  - (T_i(end)*(old_x - z)))  <= 0;
%                  obj.Polynomials.Fun = (sum((-T_i(1:end-1))) - (T_i(end))*old_x) * obj.Digit;
%                  obj.Polynomials.Inequalities = (sum((-T_i(1:end-1))) - (T_i(end))*old_x - 10)*obj.Digit <= 0;
%              
%                 obj.Polynomials.Fun = z;
%                 obj.Polynomials.Inequalities = (sum((-T_i(1:end-1))) - z) <= 0;
%                 obj.Polynomials.Inequalities = (sum((-T_i(1:end-1))) - z*T_i(end)) <= 0;
%                 obj.Polynomials.AuxVarRange(1) = z >= 0;
%                 obj.Polynomials.AuxVarRangeNum = [0;1000000];
            end
            
            obj.Polynomials.AuxVarRangeNum(:,1) = [0;5];
            count = 1;
            obj.Polynomials.AuxVarRange(count) = z >= 5;
            count = count +1;
            obj.Polynomials.AuxVarRange(count) = z <=5;
            
%              obj.Polynomials.Equalities = -T_i(end) - 1 == 0;
            %             if sgn == -1
            %                 obj.Polynomials.Fun = (sum((T_i(1:end-1)))*z);
            %                 obj.Polynomials.AuxVarRangeNum = [0;10000];
            %             else
            %                  obj.Polynomials.Fun = (sum((-T_i(1:end-1)))*z);
            %                 obj.Polynomials.AuxVarRangeNum = [0;10000];
            %             end
            
            %             obj.Polynomials.AuxVarRange(1) = z >= 0;
            %             obj.Polynomials.AuxVarRange(2) = z <= 1;
           
            %             obj.Polynomials.Inequalities = [];
            
            %
            % if sgn == -1
            %     obj.Polynomials.Fun = z;
            %     obj.Polynomials.Inequalities = z*sum((T_i(1:end-1)))* obj.Digit;
            %     obj.Polynomials.AuxVarRange(1) = z >= 0;
            %     obj.Polynomials.AuxVarRange(2) = z <= 100000;
            %     obj.Polynomials.AuxVarRangeNum = [0;100000];
            % else
            % %     obj.Polynomials.Fun = z;
            %     obj.Polynomials.Fun = z*sum((T_i(1:end-1)))* obj.Digit;
            %     obj.Polynomials.Inequalities(1) = z*T_i(end) - 1<= 0;
            %
            %     obj.Polynomials.AuxVarRange(1) = z >= 0;
            %     obj.Polynomials.AuxVarRange(2) = z <= 100000;
            %     obj.Polynomials.AuxVarRangeNum = [0;10000];
            % end
            
            
            %             obj.Polynomials.Fun = Necessary_Constraints * obj.Digit;
            %             count = 1;
            %             for i = 1:numel(Necessary_Constraints)
            %                 if ~isempty(symvar(Necessary_Constraints(i)))
            %                     obj.Polynomials.Inequalities(count) =  Necessary_Constraints(i) * obj.Digit <= 0;
            %                     count = count + 1;
            %                 end
            %             end
            %             count = 1;
            %             obj.Polynomials.AuxVarRangeNum(:,count) = [0];
            %             obj.Polynomials.AuxVarRange(count) = z >= 0.05;
            %             count = count +1;
            %             obj.Polynomials.AuxVarRange(count) = z <=1;
            
            
            
            
            
        end
        
    end
end