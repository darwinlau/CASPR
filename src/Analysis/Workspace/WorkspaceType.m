classdef WorkspaceType
    %WorkspaceType is the enumeration for the type of workspace
    enumeration 
        WCW                     % Wrench Closure
        SW                      % Static
        SCW                     % Static Closure
        OCW                     % Output Closure
        PCW                     % Positive Controllability
        NULL                    % Stub Workspace                      
    end
end