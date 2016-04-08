classdef BodyFlags < handle
    %BODYFLAGS This will probably be renamed
    
    properties
        dynamics        % A flag which indicates if dynamics computation is required
        op_space        % A flag which indicates if operational space computation is required
    end
    
    methods
        function occupied = BodyFlags()
           occupied.dynamics = false;
           occupied.op_space = false;
        end
    end
    
end

