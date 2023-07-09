classdef ReconfigureWrenchFeasibleConstraintGeneration < handle
    %UNTITLED11 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Polynomials;
    end
    properties (Hidden)
        Digit = 1e0;
    end
    methods
        function obj = ReconfigureWrenchFeasibleConstraintGeneration(Kinematic_Model,Cable_Index,w,f_max,sgn,fixedLength)
            
            %tmp hack
            
            J_bar = Kinematic_Model.L_Removed_Norm;
            slack_cable_length = Kinematic_Model.Cable_Length;
            
            %             for i = 1:numel(Kinematic_Model.ValidReconfigCableID)
            %                 obj.Polynomials.MoM(i) =  slack_cable_length(Kinematic_Model.ValidReconfigCableID(i))^2 == Kinematic_Model.ValidAuxLength(i)^2;
            %             end
            
            slack_cable_length(Kinematic_Model.ValidReconfigCableID) = Kinematic_Model.ValidAuxLength;
            
            
            J = J_bar/diag(slack_cable_length);
            
            
            J_s = J(:,Cable_Index{1,1});
            if ~isempty(Cable_Index{1,2})
                J_c = sum(J(:,Cable_Index{1,2}),2);
            else
                J_c = [];
            end
            J_C_sub = [J_s,J_c,-w];
            if ~isempty(w)
                [J_hat,Unit_Length] = numden(J_C_sub(:,1:end-1));
                J_hat(:,end+1) = -w;
                Unit_Length(:,end+1) = 1;
            else
                [J_hat,Unit_Length] = numden(J_C_sub);
            end
            
            [n,m] = size(J_hat);
            T_hat_0 = det(J_hat(1:n,1:n));            
            adj_J = adjoint(J_hat(1:n,1:n));
            T_hat_i = adj_J*J_hat(:,end);
            
            %% normalize 
            [coe,~]=coeffs(T_hat_0);
            T_hat_0 = T_hat_0/abs(coe(end))* obj.Digit;
            T_hat_i = T_hat_i/abs(coe(end))* obj.Digit;

%             for i = 1:numel(T_hat_i)
%                 [coe,~]=coeffs(T_hat_i(i));
%                 T_hat_i(i) = T_hat_i(i)/abs(coe(end));
%             end
            
%             T_hat_0 = T_hat_0/abs(coe(end))* obj.Digit;
%             T_hat_i = T_hat_i*10000;
            %%    
            if sgn == -1
                Necessary_Constraints =  [T_hat_i;(-T_hat_0)];
                %                 T_hat_0 = T_hat_0*-1;
                %                 Force_Constraints =  -(  T_hat_i.*Unit_Length(1,1:end-1).' - (f_max * T_hat_0)  ) ;
            else
                Necessary_Constraints =  -[T_hat_i;-(T_hat_0)];
                %                 Force_Constraints =  -(  T_hat_i.*Unit_Length(1,1:end-1).' - (f_max * T_hat_0)  ) ;
            end
            
            for i = 1:numel(Necessary_Constraints)
                for j = 1:numel(Kinematic_Model.ValidAuxLength)
                    Necessary_Constraints(i) = subs(Necessary_Constraints(i),Kinematic_Model.ValidAuxLength(j),Kinematic_Model.Cable_Length_Poly{j});
                end
            end
            
            count = 1;
            for i = 1:numel(Necessary_Constraints)
                if ~isempty(symvar(Necessary_Constraints(i)))
                    obj.Polynomials.Inequalities(count) =  -Necessary_Constraints(i) <= 0;
                    count = count + 1;
                end
                
            end
            if isempty(fixedLength)
                for i = 1:numel(Kinematic_Model.Cable_Length_Poly)                    
                    obj.Polynomials.Inequalities(count) =  -Kinematic_Model.Cable_Length_Poly{i} <= 0;
                    count = count + 1;                    
                end                
                obj.Polynomials.Equalities = [];
            else
                count = 1;
                for i = 1:numel(Kinematic_Model.Cable_Length_Poly)                    
                    obj.Polynomials.Equalities(count) =  Kinematic_Model.Cable_Length_Poly{i} - fixedLength(i) == 0;
                    count = count + 1;                    
                end                  
                
            end
            %             % for UAV only
            %             count = 1;
            %             for i = 1:numel(Kinematic_Model.Cable_Length_Poly)
            %                 obj.Polynomials.Equalities(count) =  Kinematic_Model.Cable_Length_Poly{i} - [1.48677503341965] == 0;
            % %                 obj.Polynomials.Inequalities(count) =  Kinematic_Model.Cable_Length_Poly{i} >= 0;
            %                 count = count + 1;
            %             end
            
            
            obj.Polynomials.MoM = [];
            
        end
        
    end
end