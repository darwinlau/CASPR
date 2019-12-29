% Measures the robustness of the CDPR in producing a given workspace 
% condition.
%
% Please cite the following:
% 
% Measures the robustness of the CDPR in producing a given workspace 
% condition.
%
% Please cite the following paper when using this algorithm:
% A. L. Cruz-Ruizy, S. Caro, P. Cardou and F.Guay "ARACHNIS: Analysis
% of robots actuated by cables with handy and neat interface software", 
% in Proceedings of the Cable-Driven Paralle Robots, 2015.
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   : Capacity margin metric
classdef CapacityMarginMetric < WorkspaceMetricBase
    % Constants that needs to be defined from parent
    properties (Constant)
        type = WorkspaceMetricType.CAPACITY_MARGIN;
        metricMin = -Inf;
        metricMax = Inf;
    end
    
    properties (SetAccess = protected)
        desired_wrench_set
    end
    
    methods
        % Constructor
        function m = CapacityMarginMetric(desired_wrench_set)
            m.desired_wrench_set = desired_wrench_set;
        end
        
        % Evaluate Function implementation
        function v = evaluateFunction(obj,dynamics)
            w = dynamics.availWrenchSet;
            
            if isempty(w.b)
                % in case the generation of the feasible wrench set convex
                % hull failed, consider a negative capacity margin value
                v = -1;
            else
                q   =   length(w.b);
                p   =   size(obj.desired_wrench_set,2);
                s   =   zeros(p,q);
                for i = 1:p
                    for j=1:q
                        s(i,j) = (w.b(j) - w.A(j,:)*obj.desired_wrench_set(:,i))/norm(w.A(j,:),2);
                    end
                end
                v = min(min(s));
            end
        end
    end
end