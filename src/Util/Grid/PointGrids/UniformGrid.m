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
                id.q_length = -1*ones(id.n_dimensions, 1);
                for i = 1:id.n_dimensions
                    if(id.delta_q(i) == 0)
                        id.q_length(i) = 1;
                    else
                        id.q_length(i) = round((id.q_end(i) - id.q_begin(i))/id.delta_q(i))+1;
                    end
                end
            elseif(strcmp(info_type,'number_steps'))
                id.q_length = q_info;
                id.delta_q = -1*ones(id.n_dimensions, 1);
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

        % Obtain the grid coordinate from a given index
        function q_coord = getGridCoordinate(obj,index)
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
            q_coord = q_index + ones(size(q_index));
        end
        
        % Obtain the grid index from the current point
        function index = getGridIndex(obj,point)
            % First find q_index
            q_mult  =   [obj.q_length];
            q_div   =   prod(q_mult);
            index = 1;
            for i = 1:obj.n_dimensions
                q_div = q_div/q_mult(i);
                index = index + q_div*(round((point(i) - obj.q_begin(i))/(obj.delta_q(i))));
            end
        end
        
        % Obtain a single dimension subgrid
        function grid_single_dim = getSingleDimensionSubGrid(obj, dim)
            if dim > obj.n_dimensions
                dim = obj.n_dimensions;
            end
            grid_single_dim = obj.q_begin(dim):obj.delta_q(dim):obj.q_end(dim);
        end
        
        % function that merges subgrids using the corresponding index table
        function resulting_index_table = mergeSubGrids(obj, index_table_1, index_table_2)
            resulting_index_table    =   index_table_2;
            for i = 1:length(index_table_1)
                tmp_index = index_table_1(i);
                no_replicate_found = true;
                cnt = 1;
                while cnt <= length(index_table_2) && no_replicate_found
                    if (tmp_index == index_table_2(cnt))
                        no_replicate_found = false;
                    else
                        cnt = cnt + 1;
                    end
                end
                if no_replicate_found
                    resulting_index_table = [resulting_index_table;tmp_index];
                end
            end
        end
        
        % Obtain a subgrid with a specific size (NOTE: the size is given as
        % integers which indicate number of grid steps)
        function result_index = getSubGrid(obj, center, half_size, existing_grid_index)
            if nargin > 3
                center          =   obj.getGridPoint(obj.getGridIndex(center));
                q_begin_local  	=   center - half_size.*obj.delta_q;
                q_mult          =   2*half_size + ones(size(half_size));
                q_div           =   prod(q_mult);
                numPoints       =   q_div;
                result_index    =   [];
                for i = 1:numPoints
                    % Convert the index into a column index
                    index   =   i;
                    q_div   =   numPoints;
                    q_index =   zeros(obj.n_dimensions,1);
                    tol     =   1e-8;
                    for k = 1:length(center)
                        q_div = q_div/q_mult(k);
                        q_index(k,1) = floor((index-1)/q_div + tol);
                        index = index - (q_index(k,1))*q_div;
                    end
                    q = q_begin_local + q_index.*obj.delta_q;
                    tmp_index   =   obj.getGridIndex(q);
                    no_replicate_found = true;
                    cnt = 1;
                    while cnt <= length(existing_grid_index) && no_replicate_found
                        if (tmp_index == existing_grid_index(cnt))
                            no_replicate_found = false;
                        else
                            cnt = cnt + 1;
                        end
                    end
                    if no_replicate_found
                        result_index = [result_index;tmp_index];
                    end
                end
            else
                center          =   obj.getGridPoint(obj.getGridIndex(center));
                q_begin_local  	=   center - half_size.*obj.delta_q;
                q_mult          =   2*half_size + ones(size(half_size));
                q_div           =   prod(q_mult);
                numPoints       =   q_div;
                result_index    =   zeros(numPoints, 1);
                for i = 1:numPoints
                    % Convert the index into a column index
                    index   =   i;
                    q_div   =   numPoints;
                    q_index =   zeros(obj.n_dimensions,1);
                    tol     =   1e-8;
                    for k = 1:length(center)
                        q_div = q_div/q_mult(k);
                        q_index(k,1) = floor((index-1)/q_div + tol);
                        index = index - (q_index(k,1))*q_div;
                    end
                    q = q_begin_local + q_index.*obj.delta_q;
                    result_index(i) = obj.getGridIndex(q);
                end
            end
        end
        
        
        
        % Obtain a subgrid with a specific size (NOTE: the size is given as
        % integers which indicate number of grid steps)
        function result_index = getSubGridVertices(obj, center, half_size)
            result_index    =   [];
            center          =   obj.getGridPoint(obj.getGridIndex(center));
            q_begin_local   =   center - half_size.*obj.delta_q;
            q_end_local     =   center + half_size.*obj.delta_q;
            cnt = 1;
            numPoints = 2^length(center);
            while cnt <= numPoints
                inner_cnt = 1;
                tmp_cnt = cnt - 1;
                q_tmp = zeros(size(center));
                while inner_cnt <= length(center)
                    if (mod(tmp_cnt, 2) == 0)
                        q_tmp(inner_cnt) = q_begin_local(inner_cnt);
                    else
                        q_tmp(inner_cnt) = q_end_local(inner_cnt);
                    end
                    tmp_cnt = floor(tmp_cnt/2);
                    inner_cnt = inner_cnt + 1;
                end
                result_index = [result_index;obj.getGridIndex(q_tmp)];
                cnt = cnt + 1;
            end
        end
        
        % getters
        function q_begin = getPoseLowerBound(obj)
            q_begin = obj.q_begin;
        end
        function q_end = getPoseUpperBound(obj)
            q_end = obj.q_end;
        end
        function delta_q = getPoseIncrement(obj)
            delta_q = obj.delta_q;
        end
        function q_length = getPoseLength(obj)
            q_length = obj.q_length;
        end
    end
end
