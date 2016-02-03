% Objective function for minimising the interaction forces and moments at
% the joints of the manipulator
%
% Please cite the following paper when using this:
% D. Lau, D. Oetomo, and S. K. Halgamuge, "Inverse Dynamics of Multilink
% Cable-Driven Manipulators With the Consideration of Joint Interaction 
% Forces and Moments," IEEE Trans. Robot., vol. 31, no. 2, pp. 479–488, 2015.
% 
% Author        : Darwin LAU
% Created       : 2016
% Description	: The weights are for each joint force/moment component
classdef IDObjectiveMinInteractions < IDObjectiveQuadratic
    properties (SetAccess = protected)
        weights
    end
    
    methods
        function o = IDObjectiveMinInteractions(weights)
            o.weights = weights;
        end
        
        function updateObjective(obj, dynamics)
            obj.A = zeros(dynamics.numCables, dynamics.numCables);
            obj.b = zeros(dynamics.numCables,1);
            obj.c = 0;
            
            a = dynamics.P'*(dynamics.bodyDynamics.M_b*dynamics.q_ddot + dynamics.bodyDynamics.C_b - dynamics.bodyDynamics.G_b);
            w_T = dynamics.P'*dynamics.V';
            
            for k = 1:dynamics.numLinks
                for dof = 1:6
                    a_x = a(6*(k-1)+dof);
                    H_vector = w_T(6*(k-1)+dof, :).';
                    obj.A = obj.A + obj.weights(6*(k-1)+dof)*(H_vector*H_vector.');
                    obj.b = obj.b + obj.weights(6*(k-1)+dof)*2*a_x*H_vector;
                    obj.c = obj.c + obj.weights(6*(k-1)+dof)*a_x^2;
                end
            end
            % This is because the general form is
            % (1/2) x^T A x + b^T x + c
            obj.A = 2*obj.A;
        end
        
        function updateWeights(obj, weights)
            obj.weights = weights;
        end
    end    
end

