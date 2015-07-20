classdef MatrixOperations 
    %MATRIXOPERATIONS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods (Static)
        function s = SkewSymmetric(v)
            s = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
        end
        
        function C = MatrixProdQuad(A,B)
            ma = size(A,1); na = size(A,2);
            nb = size(B,2);
            C = zeros(ma*nb,nb);
            for i=1:ma
                r = (i-1)*na+1:(i-1)*na+na;
                for j = 1:na
                    r2 = (j-1)*na+1:(j-1)*na+na;
                    C(r,:) = C(r,:) + A(i,j)*B(r2,:);
                end
            end
        end
    end
    
end

