classdef MatrixOperations 
    %MATRIXOPERATIONS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods (Static)
        function s = SkewSymmetric(v)
            s = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
        end
    end
    
end

