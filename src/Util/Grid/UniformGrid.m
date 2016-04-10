% Uniform Grid defined with beginning and end point
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    :
classdef UniformGrid < Grid
    
    properties (SetAccess = private)
        q_begin
        q_end
        delta_q
        q_length
    end
    
    methods
        function id = UniformGrid(q_begin,q_end,delta_q)
            assert((size(q_begin,2)==1)&&(size(q_end,2)==1)&&(size(delta_q,2)==1),'Input to UniformGrid must be a column vector');
            assert((size(q_begin,1)==size(q_end,1))&&(size(q_begin,1)==size(delta_q,1)),'Inputs must be of the same dimension');
            assert((sum(delta_q > q_end - q_begin)==0)||(sum(delta_q == 0) ~= 0),'Invalid Input Range');
            % Maybe add more checks to ensure
            id.q_begin  =   q_begin;
            id.q_end    =   q_end;
            id.delta_q  =   delta_q;
            id.setNDimensions(size(q_begin,1));
            for i=1:id.n_dimensions
                if(delta_q(i)~=0)
                    id.q_length(i) = (id.q_end(i) - id.q_begin(i))/id.delta_q(i) + 1;
                else
                    id.q_length(i) = 1;
                end
            end
            id.setNPoints(round(prod(id.q_length)))
        end
        
        function q = getGridPoint(obj,index)
            % Convert the index into a column index
            q_index =   zeros(obj.n_dimensions,1);
            q_mult  =   [obj.q_length];
            tol     =   1e-8;
            q_div   =   prod(q_mult);
            for i = 1:obj.n_dimensions
                q_div = q_div/q_mult(i);
                q_index(i,1) = floor((index-1)/q_div + tol);
                index = index - (q_index(i,1))*q_div;
            end
            q = obj.q_begin + q_index.*obj.delta_q;
        end 
    end
    
end

