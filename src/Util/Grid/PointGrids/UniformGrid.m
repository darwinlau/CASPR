% Uniform Grid defined with beginning and end point
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    :
classdef UniformGrid < PointGridBase

    properties (SetAccess = private)
        q_begin     % The lower bound on grid generalised coordinates
        q_end       % The upper bound on grid generalised coordinates
        delta_q     % The step size in generalised coordinates
        q_length    % The length of each index
        q_wrap      % A boolean vector to indicate if the coordinate wraps around at its limits
        dim_disc_ia % array contains the index of dimensions to be discretized
    end

    methods
        % The constructor for the grid.
        function id = UniformGrid(q_begin,q_end,q_info,info_type,q_wrap)
            CASPR_log.Assert((size(q_begin,2)==1)&&(size(q_end,2)==1)&&(size(q_info,2)==1),'Input to UniformGrid must be a column vector');
            CASPR_log.Assert((size(q_begin,1)==size(q_end,1))&&(size(q_begin,1)==size(q_info,1)),'Inputs must be of the same dimension');
            CASPR_log.Assert(sum(q_begin - q_end > 0) == 0,'Invalid input range');
            CASPR_log.Assert(sum(q_info < 0) == 0,'q_info variable contain only non-negative terms');
            % Maybe add more checks to ensure
            id.q_begin  =   q_begin;
            id.q_end    =   q_end;
            id.setNDimensions(size(q_begin,1));
            id.dim_disc_ia = find(q_info ~= 0);
            
            if((nargin==3)||strcmp(info_type,'step_size'))
                id.delta_q = q_info;
                for i = 1:id.n_dimensions
                    if(id.delta_q(i) == 0)
                        id.q_length(i) = 1;
                    else
                        id.q_length(i) = round((id.q_end(i) - id.q_begin(i))/id.delta_q(i))+1;
                    end
                end
            elseif(strcmp(info_type,'number_steps'))
                id.q_length = q_info;
                for i = 1:id.n_dimensions
                    if(id.q_length(i) > 1)
                        id.delta_q(i) = (id.q_end(i) - id.q_begin(i))/(id.q_length(i) - 1);
                    else
                        CASPR_log.Assert(q_begin(i)==q_end(i),'Begin and end points must be equal if the number of steps is 1');
                        id.delta_q(i) = 1;
                    end
                end
            else
                CASPR_log.Error('Unknown info type entered');
            end
            id.setNDimensions(size(q_begin,1));
            id.setNPoints(round(prod(id.q_length)));
            if(nargin <= 4)
                id.q_wrap = false(size(q_begin));
            else
                id.q_wrap = q_wrap;
            end
        end

        % Obtain the grid point from a given index
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
        
        % Obtain the grid index from the current point
        function index = getGridIndex(obj,point)
            % First find q_index
            q_mult  =   [obj.q_length];
            q_div   =   prod(q_mult);
            index = 1;
            for i = 1:obj.n_dimensions
                q_div = q_div/q_mult(i);
                index = index + q_div*(round(point(i) - obj.q_begin(i)/(obj.delta_q(i))));
            end
        end
    end
end
