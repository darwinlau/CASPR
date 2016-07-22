% Objective function for minimising the interaction forces and moments at
% the joints of the manipulator
%
% Please cite the following paper when using this:
% D. Lau, D. Oetomo, and S. K. Halgamuge, "Inverse Dynamics of Multilink
% Cable-Driven Manipulators With the Consideration of Joint Interaction 
% Forces and Moments," IEEE Trans. Robot., vol. 31, no. 2, pp. 479-488, 2015.
% 
% Author        : Darwin LAU
% Created       : 2016
% Description	: The weights are for each joint force/moment component
classdef IDObjectiveMinInteractions < IDObjectiveQuadratic
    properties (SetAccess = protected)
        weights
    end
    
    methods
        % The constructor function for minimising the interactions
        function o = IDObjectiveMinInteractions(weights)
            o.updateWeights(weights);
        end
        
        % The objective update implementation
        function updateObjective(obj, dynamics)
            CASPR_log.Assert(length(obj.weights) == 6 * dynamics.numLinks, 'Dimensions of weight is not correct, it should be a vector of length 6*numLinks');
            
            num_actuators = dynamics.numActuatorsActive;
            num_active_cables = dynamics.numCablesActive;
            obj.A = zeros(num_actuators, num_actuators);
            obj.b = zeros(num_actuators,1);
            obj.c = 0;
            
            a = dynamics.bodyModel.P'*(dynamics.bodyModel.M_b*dynamics.q_ddot + dynamics.bodyModel.C_b - dynamics.bodyModel.G_b);
            w_T = dynamics.bodyModel.P'*dynamics.cableModel.V';
            
            % Only consider the active cables
            active_indices = dynamics.cableModel.cableIndicesActive;
            passive_indices = dynamics.cableModel.cableIndicesPassive;
            
            for k = 1:dynamics.numLinks
                for dof = 1:6
                    H_vector = w_T(6*(k-1)+dof, :).';
                    H_vector_active = H_vector(active_indices);
                    H_vector_passive = H_vector(passive_indices);
                    a_x = a(6*(k-1)+dof) + H_vector_passive.' * dynamics.cableForcesPassive;
                    % Only cables affect the joint interaction forces and
                    % moments
                    obj.A(1:num_active_cables,1:num_active_cables) = obj.A(1:num_active_cables,1:num_active_cables) + obj.weights(6*(k-1)+dof)*(H_vector_active*H_vector_active.');
                    obj.b(1:num_active_cables) = obj.b(1:num_active_cables) + obj.weights(6*(k-1)+dof)*2*a_x*H_vector_active;
                    obj.c = obj.c + obj.weights(6*(k-1)+dof)*a_x^2;
                end
            end
            % This is because the general form is
            % (1/2) x^T A x + b^T x + c
            obj.A = 2*obj.A;
        end
        
        % An update of the weights
        function updateWeights(obj, weights)
            obj.weights = weights;
        end
    end    
end

