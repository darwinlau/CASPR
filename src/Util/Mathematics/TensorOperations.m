% Library of tensor operation utilities. This can be used to compute the product of tensors used in
% linearisation and hessian computation.
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
classdef TensorOperations 
    methods (Static)
        function C = VectorProduct(A_ten,b)
            s = size(A_ten);
            C = zeros(s(1:2));
            for i = 1:s(2)
                C = C + A_ten(:,:,i)*b(i);
            end
        end
        
%         function C = RightMatrixProduct(A_ten,B)
%             
%         end
%         
%         function C = LeftMatrixProduct(A,B_ten)
%             
%         end
    end
end

