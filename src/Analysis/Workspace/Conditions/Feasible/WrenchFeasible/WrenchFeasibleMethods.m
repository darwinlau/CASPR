% An enum class of for static workspace methods
% Author         : Jonathan EDEN
% Created        : 2015
% Description    : Enum class for different wrench feasible workspace methods
classdef WrenchFeasibleMethods
    enumeration 
        CM
    end
    
    methods (Static)
        % Lists the workspace methods for the GUI
        function L = workspace_method_list()
            L = {'capacity_margin'};
        end
    end
end