% Represents the operational space orientation for a paticular link
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   :
classdef OpOrientation < OpSpace
    %OpOrientation OpSpace definition for an orientation
    
    methods
        function o = OpOrientation(selection_matrix)
            o.numOutputDofs     =   sum(diag(selection_matrix));
            % Determine the selection matrix assuming 6 DoF
            temp_selection_matrix = [zeros(3),eye(3)];
            o.selection_matrix  =   temp_selection_matrix(diag(selection_matrix),:);
        end
        
        function y = ExtractOutput(obj,x,~)
            % DETERMINE WHAT IS WANTED HERE
        end
    end
end

