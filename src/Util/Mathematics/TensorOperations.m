% Library of tensor operation utilities. This can be used to compute the product of tensors used in
% linearisation and hessian computation.
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
classdef TensorOperations
    methods (Static)
        % Right multiplication of a tensor of order 3 with a vector to produce a matrix.
        % Three different operations are possible dependent upon which
        % index is given. These operations are consistent with the 1 mode,
        % 2 mode and 3 mode tensor products defined in the literature.
        function C = VectorProduct(A_ten,b,index,flag)
            CASPR_log.Assert((index>=1)&&(index<=3),'index input must be between 1 and 3');
            s = size(A_ten);
            if(length(s) == 2)
                s(3) = 1;
            end
            CASPR_log.Assert(s(index) == length(b),'Invalid multiplication b must be the same length as s(index)');
            switch index
                case 1
                    C = MatrixOperations.Initialise(s(2:3),flag);
                    for l = 1:s(1)
                        % Compute the multiplication
                        temp = A_ten(l,:,:)*b(l);
                        Ab = zeros(s(2:3));
                        for j = 1:s(2)
                            Ab(j,:) = temp(1,j,:);
                        end
                        C = C + Ab;
                    end
                case 2
                    % There appears to be no use for this multiplication at
                    % this point int time.
                    C = MatrixOperations.Initialise(s([1,3]),flag);
                    for l = 1:s(2)
                        % Compute the multiplication
                        temp = A_ten(:,l,:)*b(l);
                        Ab = MatrixOperations.Initialise(s([1,3]),flag);
                        for j = 1:s(1)
                            Ab(j,:) = temp(j,1,:);
                        end
                        C = C + Ab;
                    end
                case 3
                    C = MatrixOperations.Initialise(s(1:2),flag);
                    for l = 1:s(3)
                        C = C + A_ten(:,:,l)*b(l);
                    end
                otherwise
                    CASPR_log.Print('Index must be 1, 2 or 3.',CASPRLogLevel.ERROR);
            end
        end

        % Right multiplication of a tensor of order 3 with a matrix to
        % produce another tensor.
        function C = RightMatrixProduct(A_ten,B,flag)
            s_A = size(A_ten); s_B = size(B);
            C = MatrixOperations.Initialise([s_A(1),s_B(2),s_A(3)],flag);
            for j = 1:s_B(2)
                for l = 1:s_A(2)
                    C(:,j,:) = C(:,j,:) + A_ten(:,l,:)*B(l,j);
                end
            end
        end

        % Left multiplication of a tensor of order 3 with a matrix to
        % produce another tensor.
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

        % Left and right multiplication of a tensor by the same matrix.
        function C = LeftRightMatrixProduct(A,B_ten,flag)
            % I think it should be transpose
            C_temp  =   TensorOperations.RightMatrixProduct(B_ten,A,flag);
            C       =   TensorOperations.LeftMatrixProduct(A,C_temp,flag);
        end

        % Transpose operation for tensors. The indices given by index are
        % switched in an analogous fashion to tensor math.
        function C = Transpose(A,index,flag)
            CASPR_log.Assert(length(index)==2,'Two indices must be given for the transpose');
            CASPR_log.Assert(sum(index>3)+sum(index<0)==0,'Transpose only supported for Tensors of order 3');
            % Acts like a transpose operation for a 3 dimensional array.
            % The indices to switch are assumed to be given.
            s = size(A);
            if(sum(index==1)==0)
                % The transpose if acting on index 2 and 3
                C = MatrixOperations.Initialise([s(1),s(3),s(2)],flag);
                for j =1:s(2)
                    for k = 1:s(3)
                        C(:,k,j) = A(:,j,k);
                    end
                end
            elseif(sum(index==2)==0)
                % The transpose if acting on index 1 and 3
                C = MatrixOperations.Initialise([s(3),s(2),s(1)],flag);
                for j =1:s(1)
                    for k = 1:s(3)
                        C(k,:,j) = A(j,:,k);
                    end
                end
            else
                % The transpose if acting on index 1 and 2
                C = MatrixOperations.Initialise([s(2),s(1),s(3)],flag);
                for j =1:s(1)
                    for k = 1:s(2)
                        C(k,j,:) = A(j,k,:);
                    end
                end
            end
        end
    end
end
