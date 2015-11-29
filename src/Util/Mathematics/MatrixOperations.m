classdef MatrixOperations 
    %MATRIXOPERATIONS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods (Static)
        function s = SkewSymmetric(v)
            s = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
        end
        
        function C = MatrixProdLeftQuad(A,B)
            ma = size(A,1); na = size(A,2);
            mb = size(B,1); %nb = size(B,2);
            C = MatrixOperations.Initialise(mb,ma*mb,isa(B,'sym'));            
            for i=1:ma
                c = (i-1)*mb+1:(i-1)*mb+mb;
                for j = 1:na
                    c2 = (j-1)*mb+1:(j-1)*mb+mb;
                    C(:,c) = C(:,c) + A(i,j)*B(:,c2);
                end
            end
        end
        
        function C = MatrixProdRightQuad(A,B)
            ma = size(A,1); na = size(A,2);
            mb = size(B,1); %nb = size(B,2);
            C = MatrixOperations.Initialise(ma,(na*(ma/mb)),isa(A,'sym'));
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
        
        function C = Initialise(m,n,flag)
            if(flag == 1)
                % Symbolic
                C = sym(zeros(m,n));
            else
                C = zeros(m,n);
            end
        end
    end
    
end

