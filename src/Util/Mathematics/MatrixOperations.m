% Library of matrix operation utilities
%
% Author        : Darwin LAU
% Created       : 2014
% Description    :
classdef MatrixOperations
    methods (Static)
        % Given the vector [v(1);v(2);v(3)], arrage the vector into a skew
        % symmetric matrix.
        function s = SkewSymmetric(v)
            s = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
        end

        % Given a set of vectors determine a linearly independent set of
        % the largest possible rank
        function [L,M] = FindLinearlyIndependentSet(A)
            [~,~,E] = qr(A,0); [n,m] = size(A);
            L = A(:,E(1:n));
            M = A(:,E(n+1:m));
        end

        % Matrix multiplication operations for computation of the internal
        % quadratic form matrices.
        function C = MatrixProdLeftQuad(A,B)
            ma = size(A,1); na = size(A,2);
            mb = size(B,1); %nb = size(B,2);
            C = MatrixOperations.Initialise([mb,ma*mb],isa(B,'sym'));
            for i=1:ma
                c = (i-1)*mb+1:(i-1)*mb+mb;
                for j = 1:na
                    c2 = (j-1)*mb+1:(j-1)*mb+mb;
                    C(:,c) = C(:,c) + A(i,j)*B(:,c2);
                end
            end
        end

        % Matrix multiplication operations for computation of the internal
        % quadratic form matrices.
        function C = GenerateMatrixQuadCross(A,B)
            n_q = size(A,2);
            M_A = [zeros(n_q,1),-A(3,:).',A(2,:).',A(3,:).',zeros(n_q,1),-A(1,:).',-A(2,:).',A(1,:).',zeros(n_q,1)];
            C = MatrixOperations.interior_product_right_quad(M_A,B);
        end

        % Initialise a matrix. This calls zeros for real variables (flag =
        % 0) otherwise it calls the equivalent symbolic function.
        function C = Initialise(s,flag)
            if(flag == 1)
                % Symbolic
                C = sym(zeros(s));
            else
                C = zeros(s);
            end
        end
    end

    methods (Static, Access = private)
        % Matrix multiplication operations for computation of the internal
        % quadratic form matrices.
        function C = interior_product_right_quad(A,B)
            ma = size(A,1); na = size(A,2);
            mb = size(B,1); %nb = size(B,2);
            C = MatrixOperations.Initialise([ma,(na*(ma/mb))],isa(A,'sym'));
            for i = 1:na/mb
                c = (i-1)*mb + 1:(i-1)*mb + mb;
                c2 = (i-1)*ma + 1:(i-1)*ma + ma;
                C(:,c2) = A(:,c)*B;
            end
            % Make symetric
            n2 = size(C,2); n1 = size(C,1);
            for i=1:n2/n1
                c = (i-1)*n1 + 1:(i-1)*n1 + n1;
                C(:,c) = 0.5*(C(:,c) + C(:,c).');
            end
        end
    end
end
