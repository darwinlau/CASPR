classdef WorkspaceType
    %WorkspaceType is the enumeration for the type of workspace
    enumeration 
        IF                      % Inteference workspace
        CW                      % Wrench Closure
        SW                      % Static
        FW                      % Feasible workspace 
        NULL                    % Stub Workspace                      
    end
    
    methods (Static)
        function L = workspace_type_list()
            L = {'Interference Free','Wrench Closure','Static','Feasible Workspace'};
        end
    end
end