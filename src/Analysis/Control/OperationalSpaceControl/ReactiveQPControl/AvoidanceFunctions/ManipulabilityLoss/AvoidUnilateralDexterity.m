% Class for avoiding loss of manipulability (unilateral dexterity)
%
% Author        : Dominic Chan
% Created       : 2019
% Description   : Avoiding loss of manipulability (unilateral dexterity)
classdef AvoidUnilateralDexterity < AvoidanceBase
    properties        
    end

    methods
        % Constructor
        function aud = AvoidUnilateralDexterity(cdpr, dt, k_q)         
            aud@AvoidanceBase(cdpr, dt, k_q, -1);
        end     
        
        % Evaluate function
        function value = evaluate(obj, q, q_d)            
            % For convenience
            n_dofs = obj.cdpr.numDofs;
            n_a = obj.cdpr.numCablesActive;  
            try
                L = compile_L(q, q_d, zeros(n_dofs,1), zeros(n_dofs,1));
                assert(size(L,1)==n_a && size(L,2)==n_dofs, 'Wrong compile_L file');
            catch
                obj.cdpr.update(q, q_d, zeros(n_dofs,1), zeros(n_dofs,1));
                L = obj.cdpr.L;
            end
            
            % Unilateral Dexterity (heuristic)
            n = (eye(n_a) - L*pinv(L))*ones(n_a,1);
            n_min = min(n./norm(n));
            value = 1/cond(L)*sqrt(n_a+1)*n_min/sqrt(n_min^2 + 1);
        end
        
        % Triggers
        function flag = isAvoidanceObjective(~, ~, ~)
            % Always turn on objective function
            flag = true;
        end
        function flag = isAvoidanceConstraints(~, ~, ~)
            % Always turn off hard constraints
            flag = false;
        end
        
        % Not needed for this function
        function initVariables(~)
        end
    end   
end
