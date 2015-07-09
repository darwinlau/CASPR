classdef WorkspaceStub < WorkspaceCondition
    %IDFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected, GetAccess = protected)
    end
    
    methods
        function id = WorkspaceStub()
        end
        
        function [inWorkspace,max_vel] = evaluate(obj,dynamics)
            % This is just a dummy workspace all points will be returned.
            inWorkspace = 1;
            max_vel = 1;
        end
        
        function [isConnected] = connected(obj,workspace,i,j,grid)
            % This function always returns true.
            isConnected = 1;
        end
    end
end

