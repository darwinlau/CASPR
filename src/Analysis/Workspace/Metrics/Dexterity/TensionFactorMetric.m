% A measure of the relative tension distribution for the cables which can
% be used to evaluate the quality of wrench closure at a specific
% configuration.
%
% Please cite the following paper when using this algorithm:
% C.B Pham, S.H Yeo, G. Yang and I. Chen, "Workspace analysis of fully
% restrained cable-driven manipulators", Robotics and Autonomous Systems, 
% vol. 57, no. 9, pp. 901-912, 2009.
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   : Implementation of the tension factor metric.
classdef TensionFactorMetric < WorkspaceMetricBase
    properties (SetAccess = protected, GetAccess = protected)
    end
    
    methods
        % Constructor
        function m = TensionFactorMetric()
            m.metricMin = 0;
            m.metricMax = 1;
        end
        
        % Evaluate function implementation
        function v = evaluateFunction(~,dynamics,options)
            % Determine the Jacobian Matrix
            L = dynamics.L_active;
            [u,~,exit_flag] = linprog(ones(1,dynamics.numCablesActive),[],[],-L',zeros(dynamics.numDofs,1),1e-6*ones(dynamics.numCablesActive,1),1e6*ones(dynamics.numCablesActive,1),[],options);
            if((exit_flag == 1) && (rank(L) == dynamics.numDofs))
                h = u/norm(u);
                h_min = min(h); h_max = max(h);
                v = (h_min/h_max);
            else
                v = 0;
            end
        end
    end
end