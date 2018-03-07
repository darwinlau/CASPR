% Library of general math operations
%
% Author        : Jonathan EDEN
% Created       : 2018
% Description    :
classdef GeneralMathOperations
    methods(Static)
        function P = PolynomialFit(X,Y,degree)
            CASPR_log.Assert(size(X)==size(Y),'X and Y must be vectors of the same size');
            l_x = length(X);
            T = ones(l_x,degree+1);
            for j = 1:degree
                T(:,j) = X.^(degree-j+1);
            end
            P = T\Y;
        end
        
        function T = ComputeLeastSquareMatrix(X,degree)
            l_x = length(X);
            T = ones(l_x,degree+1);
            for j = 1:degree
                T(:,j) = X.^(degree-j+1);
            end
        end
        
        function v = ComputePolynomialVector(x,degree)
            v = ones(degree+1,1);
            for j = 1:degree
                v(j) = x.^(degree-j+1);
            end
        end
    end
end

