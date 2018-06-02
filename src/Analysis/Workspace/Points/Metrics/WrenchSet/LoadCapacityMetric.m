% Measures the maximum mass that the desired wrench direction can support
%
% PLEASE NOTE THAT THIS IS CURRENTLY ONLY WRITTEN FOR SCDMS
% Author        : Jonathan EDEN
% Created       : 2016
% Description   : Load Capacity
classdef LoadCapacityMetric < WorkspaceMetricBase
    properties (SetAccess = protected, GetAccess = protected)
        desired_wrench_direction
        mass_normalisation
    end
    
    methods
        % Constructor
        function m = LoadCapacityMetric(desired_wrench_direction,mass_normalisation)
            m.desired_wrench_direction = desired_wrench_direction;
            m.mass_normalisation = mass_normalisation;
        end
        
        % Evaluate Function implementation
        function v = evaluateFunction(obj,dynamics,~)
            % Construct the 
            L   =   dynamics.L_active;
            f_u =   dynamics.cableForcesActiveMax;
            f_l =   dynamics.cableForcesActiveMin;
            w   =   WrenchSet(L,f_u,f_l);
            q = size(w.b,1);
            % Test if 0 is in the wrench set
            if(sum(zeros(q,1) <=w.b) == q)
                % Normalise the desired_wrench_direction
                if(obj.mass_normalisation)
                    d = dynamics.G/dynamics.bodyModel.bodies{1}.m;
                else
                    d = obj.desired_wrench_direction/norm(obj.desired_wrench_direction);
                end
                % Go through each plane and find the closest intersection
                % (it must be positive).
                min_v = 1e50;
                for i = 1:q
                    if((w.b(i) == 0) && (w.A(i,:)*d > 0))
                        % The ray is outside of the wrench set
                        v = 0;
                        return;
                    elseif(w.A(i,:)*d <= 0)
                        % Their is no ray scaling that will intersect this
                        % boundary
                        continue;
                    else
                        v_temp = w.b(i)/(w.A(i,:)*d);
                        if(v_temp < min_v)
                            min_v = v_temp;
                        end
                    end
                end
                v = min_v;
            else
                v = 0;
            end
        end
    end
end