classdef WorkspaceStub < WorkspaceCondition
    %IDFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function id = WorkspaceStub()
            id.type = WorkspaceType.NULL;
        end
        
        function [inWorkspace] = evaluate(obj,dynamics)
            % This is just a dummy workspace all points will be returned.
            inWorkspace = 1;
        end
        
        function [isConnected] = connected(obj,workspace,i,j,grid)
            % This function always returns true.
            isConnected = 1;
        end
    end
end

