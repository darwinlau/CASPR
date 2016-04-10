% Represents the operational space pose (position + orientation) for a
% paticular link
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   :
classdef OpPose < OpSpace
    %OpPose OpSpace definition for pose
        
    properties (SetAccess = private)
        offset
    end    
    
    methods
        function o = OpPose(offset,selection_matrix)
            o.offset            =   offset;
            o.numOutputDofs     =   sum(diag(selection_matrix));
            % Determine the selection matrix assuming 6 DoF
            temp_selection_matrix = eye(6);
            o.selection_matrix  =   temp_selection_matrix(diag(selection_matrix),:);
        end
        
        function y = ExtractOutput(obj,x,~)
            % DETERMINE WHAT WE WANT TO DO FOR ORIENTATION
            y = obj.selection_matrix(:,1:3)*x;
        end
    end
end

