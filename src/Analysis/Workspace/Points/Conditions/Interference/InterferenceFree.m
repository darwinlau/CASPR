% Class to compute whether a pose (dynamics) is within the interference free workspace
% (IFW)
%
% Author        : Zeqing ZHANG
% Created       : 2017
% Description   : The class for evaluation of IFW  
classdef InterferenceFree < WorkspaceConditionBase    
    properties (SetAccess = protected, GetAccess = protected)
        options
    end
    
    methods
        % Constructor for Interference Free WS
        function w = InterferenceFree(method)
            if (isempty(method))
                w.method = InterferenceFreeMethods.M_MINDISTANCE_CABLE_CABLE;
            else
                w.method = method;
            end
        end
        
        % Evaluate the Interference Free condition return true if
        % satisified
        function inWorkspace = evaluateFunction(obj, dynamics, ~)
            switch(obj.method)
                case InterferenceFreeMethods.M_MINDISTANCE_CABLE_CABLE
                    inWorkspace = interference_free_mindistance_cable_cable(dynamics, obj.options);
                otherwise
                    CASPR_log.Print('Interference free method is not defined',CASPRLogLevel.ERROR);
            end
        end
    end 
end

