% Library of tensor operation utilities. This can be used to compute the product of tensors used in
% linearisation and hessian computation.
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
classdef TensorOperations 
    methods (Static)
        function C = VectorProduct(A_ten,b,flag)
            s = size(A_ten);
            if(length(s) == 2)
                s(3) = 1;
            end
            C = MatrixOperations.Initialise(s(1:2),flag);
            for l = 1:s(3)
                C = C + A_ten(:,:,l)*b(l);
            end
        end
        
        function C = RightMatrixProduct(A_ten,B,flag)
            s_A = size(A_ten); s_B = size(B);
            C = MatrixOperations.Initialise([s_A(1),s_B(2),s_A(3)],flag);
            for j = 1:s_B(2)
                for l = 1:s_A(2)
                    C(:,j,:) = C(:,j,:) + A_ten(:,l,:)*B(l,j);
                end
            end
        end
        
        function C = LeftMatrixProduct(A,B_ten,flag)
            s_A = size(A); s_B = size(B_ten);
            % I will need to change the check here at a later time
            if(length(s_B) == 2)
                s_B(3) = 1;
            end
            C = MatrixOperations.Initialise([s_A(1),s_B(2),s_B(3)],flag);
            for i = 1:s_A(1)
                for l = 1:s_A(2)
                    C(i,:,:) = C(i,:,:) + A(i,l)*B_ten(l,:,:);
                end
            end
        end
    end
end

