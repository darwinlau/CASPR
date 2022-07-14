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
        connectivity_list
    end
    properties (Hidden)
        include_diagonal = false;      
        save_path = [];
        % false: for closest 2*n point(not include diagonal;
        % true : for closest 3*n point(include diagonal;
        % however, not so support true since its will be extremly slow
    end
    methods
        % The constructor for the grid.
        function id = UniformGrid(q_begin,q_end,q_info,info_type,q_wrap,generate_node_list)
            id.save_path = [CASPR_configuration.LoadHomePath,'\scripts\local\AutoGenScripts'];
            if ~exist(id.save_path)
                mkdir(id.save_path);
                addpath(id.save_path);
            end
            CASPR_log.Assert((size(q_begin,2)==1)&&(size(q_end,2)==1)&&(size(q_info,2)==1),'Input to UniformGrid must be a column vector');
            CASPR_log.Assert((size(q_begin,1)==size(q_end,1))&&(size(q_begin,1)==size(q_info,1)),'Inputs must be of the same dimension');
            CASPR_log.Assert(sum(q_begin - q_end > 0) == 0,'Invalid input range');
            q_info(isnan(q_info)) = 0; 
            CASPR_log.Assert(sum(q_info < 0) == 0,'q_info variable contain only non-negative terms');
            % Maybe add more checks to ensure
            id.q_begin  =   q_begin;
            id.q_end    =   q_end;
            id.n_dimensions = size(q_begin,1);
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
            id.n_dimensions = size(q_begin,1);
            id.n_points = round(prod(id.q_length));
            if(nargin <= 6)
                id.q_wrap = false(size(q_begin));
            else
                id.q_wrap = q_wrap;
            end
            if nargin >= 5
                if generate_node_list == 1
                    point_set = creatNodeList(id);
                    id.getConnectivityList(point_set);
                end
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

        % Generate the connectivity list
        function getConnectivityList(obj,point_set)

            node_number = 1:size(point_set,1);

            include_diagonal = false;
            connectivity_list = [];
            grid_density = reshape(obj.q_length,1,[]); % make sure the intput is row array
            node_num = 1:1:size(point_set,1);
            A = reshape(node_num,fliplr(grid_density)); % mapping the node into the N-Dimensional matrix
            have_save_file = 0;
            disp('Generating node list...')
            file_count = 1;
            tic
            for i = 1:size(point_set,1)
                [neighbour_node_idx,distance] = obj.neighbourND(i, size(A),fliplr(obj.delta_q'));
                connectivity_list_i = [repmat(i,size(neighbour_node_idx));neighbour_node_idx;distance]';
                connectivity_list = [connectivity_list;connectivity_list_i];
                if toc > 4
                    have_save_file = 1;
                    filename = [obj.save_path,'\connectivity_list_',num2str(file_count),'.mat'];
                    save(filename,'connectivity_list');
                    connectivity_list = [];
                    file_count = file_count + 1;
                    percentage = [num2str(round(i/size(point_set,1)*100,2)),'%'];
                    disp(percentage)
                    tic
                end
            end
            if have_save_file == 1
                connectivity_list = [];
                for i = 1:file_count-1
                    filename = [obj.save_path,'\connectivity_list_',num2str(i),'.mat'];
                    tmp_list = load(filename);                    
                    connectivity_list = [connectivity_list;tmp_list.connectivity_list];
                end
                
                for i = 1:file_count-1
                    filename = [obj.save_path,'\connectivity_list_',num2str(i),'.mat'];
                    delete(filename)
                end
                
            end

            %% remove redundant list
            tmp_c = sort(connectivity_list(:,[1,2]),2);
            [~,uidx] = unique(tmp_c,'rows');


            obj.connectivity_list = connectivity_list(uidx,:);
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


        % TODO: given a center shift, shift the box reusing the original
        % box
        function [result_index, removed_index, added_index] = getShiftedSubgrid(obj, ori_center, new_center, half_size, ori_index_set)
            numDim              =   length(ori_center);
            ori_center          =   obj.getGridPoint(obj.getGridIndex(ori_center));
            new_center          =   obj.getGridPoint(obj.getGridIndex(new_center));
            ori_q_begin_local  	=   ori_center - half_size.*obj.delta_q;
            ori_q_end_local  	=   ori_center + half_size.*obj.delta_q;
            % relax the margin of the reference box
            ori_q_begin_local  	=   ori_q_begin_local - 1e-6*ones(size(ori_q_begin_local));
            ori_q_end_local  	=   ori_q_end_local + 1e-6*ones(size(ori_q_end_local));
            ori_q_mult          =   2*half_size + ones(size(half_size));
            ori_q_div           =   prod(ori_q_mult);
            ori_numPoints           =   ori_q_div;
            if length(ori_index_set) ~= ori_numPoints
                CASPR_log.Warn('The number of points in the original index set seems to be wrong, will directly generate the new box');
                result_index = obj.getSubGrid(new_center, half_size);
                removed_index = [];
                added_index = result_index;
            else
                center_val_diff = new_center - ori_center;
                center_step_diff = zeros(numDim, 1);
                for i = 1:numDim
                    if center_val_diff(i) >= 0
                        center_step_diff(i) = floor(center_val_diff(i)/obj.delta_q(i));
                    else
                        center_step_diff(i) = ceil(center_val_diff(i)/obj.delta_q(i));
                    end
                end
                %                 center_step_diff = center_val_diff./obj.delta_q;
                reuse_q_mult = ori_q_mult - center_step_diff;
                reuse_q_div = prod(reuse_q_mult);
                reuse_numPoints = reuse_q_div;
                changed_numPoints = ori_numPoints - reuse_numPoints;

                result_index = ori_index_set;
                removed_index = [];
                added_index = [];
                %                 center_step_diff_magnitude = abs(center_step_diff);
                %                 center_step_diff_sign = sign(center_step_diff);
                new_q_begin_local  	=   ori_q_begin_local + center_val_diff;
                new_q_end_local  	=   ori_q_end_local + center_val_diff;
                flag_inside_ori_box = false;
                flag_inside_new_box = false;

                for i = 1:ori_numPoints
                    % take a simple to implement but computationally less
                    % effcient way first
                    current_index = ori_index_set(i);
                    current_q = obj.getGridPoint(current_index);

                    if prod(current_q >= ori_q_begin_local & current_q <= ori_q_end_local)
                        flag_inside_ori_box = true;
                    else
                        flag_inside_ori_box = false;
                    end
                    if prod(current_q >= new_q_begin_local & current_q <= new_q_end_local)
                        flag_inside_new_box = true;
                    else
                        flag_inside_new_box = false;
                    end

                    if flag_inside_ori_box && ~flag_inside_new_box
                        % indicate the point is not inside the new box, it
                        % needs to be removed and a counterpart needs to be
                        % added
                        counterpart_q = ori_center + new_center - current_q;
                        counterpart_index = obj.getGridIndex(counterpart_q);
                        result_index = result_index(result_index ~= current_index);
                        removed_index = [removed_index; current_index];
                        added_index = [added_index; counterpart_index];
                    elseif flag_inside_ori_box && flag_inside_new_box
                        % indicate the point is inside the new box as well,
                        % nothing needs to be done
                    else
                        tmp1 = current_q >= ori_q_begin_local
                        tmp2 = current_q <= ori_q_end_local
                        CASPR_log.Error('Something went wrong, the point being evaluated should be inside the original box.');
                    end
                end
                % add the new points
                result_index = [result_index;added_index];
                if length(result_index) ~= ori_numPoints
                    CASPR_log.Warn('The number of points in the resulting index set seems to be wrong, will directly generate the new box');
                    result_index = obj.getSubGrid(new_center, half_size);
                    removed_index = [];
                    added_index = result_index;
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
        function node_list = creatNodeList(obj)
            for i = 1:obj.n_points
                pose_set(i,:) = obj.getGridPoint(i);
            end
            node_number = 1:size(pose_set,1);
            node_list  = [node_number',pose_set];
        end
    end

    methods (Static)
        function [Iadj , Radj, Nfound ] = neighbourND( index, sizeA, res )
            % Ronald Ouwerkerk (2022). Neighbour points in a matrix (https://www.mathworks.com/matlabcentral/fileexchange/29330-neighbour-points-in-a-matrix), MATLAB Central File Exchange. Retrieved June 27, 2022.
            % function  [Iadj , Radj, Nfound] = neighbour3D( index,  sizeA, res )
            % Calculate the linear indices for neighboring points in a matrix
            % Second output is and array of distances based on an input resolution vector
            % This resolution vector defaults to ones(1,ndims)
            % The output Nfound reports the number of neighbours found in within the
            % matrix. For 2D we expect up to 8, for 3D up to 26 etc...
            %
            % Example 1:
            % A is a 128x128x16 image data matrix with a spatial resolution of
            % 0.1x 0.25x .375 mm^3
            % to get the neighbouring point linear indices for point 456 we do
            % sizeA = [128 128 16]
            % [ Iadj , Radj, Nfound] = neighbourND( 456, sizeA, [ .10 .25 .375] )
            %
            % NEW: now index can be a column array with linear indices
            % Output Iadj will be Nx8 (2D) or Nx26 (3D) etc and Radj will be
            % a row array 1x8 or 1x26 etc...
            %
            % Example 2:
            % create points near the center of a 144x192x16 matrix
            % spatial resolution .3 x .3x 5 mm^3
            % idx = (-6:1:6)+((144*192*3)+144*96+76)
            %[ Iadj , Radj, Nfound] = neighbourND( idx , [144,192, 32] , [.3, 0.3, 5])
            % Results in 11x26 matrix Iadj,
            % 26 distances in Radj and Nfound is 26
            %
            % The neighbour indices outside the matrix will be zero!
            % when a single index is entered the outside points are still removed so a
            % point in a 3D matrix at the edge can sill return 17 neighbours or even less
            % when it is a corner.
            %==============================================

            %==============================================
            % Ronald Ouwerkerk 2010 NIH/NIDDK
            % New version: Now handles arrays of indices
            % This script is made available on Matlab file exchange by the author
            % for use by other Matlab programmers.
            % This script is not intended for commercial use.
            % If used for published work a reference or acknowledgement is greatly
            % appreciated.
            % The function was tested for several 1D(col and row), 2D, 3D and 4D cases
            % I cannot be sure that it really works for all dimensionalities.
            % Let me know if you find a bug (and feel free to squash it for me)
            %==============================================

            %% Set defaults and process input parameters
            % first two are arbitary values for a demo
            if nargin <1
                % default index [7,6,2]
                index = 128*128+ 128*5+7
            end

            if nargin < 2
                % default size 128x128xN with N big enough for the index
                i3 = floor( index /128/128);
                disp( 'Demo mode')
                sizeA =[128, 128, i3+2]
            end

            % Get dimensionality
            ndimA = length( sizeA );

            %Set default resolution to isotropic distances
            if nargin < 3
                res =ones(1, length( sizeA) );
            else
                if length(res) < ndimA;
                    errstr = sprintf('\nError in %s.\n The length of the resolution array (%d) must equal the number of matrix dimensions (%d)\n', ...
                        mfilename,                                           length(res)  ,                                                         ndimA  );
                    disp(errstr)
                    help( mfilename)
                    return
                else
                    % reduce the resolution array, last digit is probably slice
                    % thickness, irrelevant if we have one slice only
                    res = res( 1:ndimA );
                end
            end

            %% explicit version of ind2sub
            % ind2sub requires multiple output arguments, one for each dimension
            ilin = index(:);
            np = length( ilin );
            imat = ones( np, ndimA);

            for di = ndimA:-1:2
                blocksize = prod( sizeA( 1:(di-1)  ) );
                ndi = 1+ floor( ( ilin-1) / blocksize );
                ilin = ilin- (ndi -1) *blocksize;
                imat(:,di) = ndi;
            end
            imat(:,1) = ilin;

            %% Find the indices of neighbours
            % Get all the index permutations for neighbours ( -1, +1) over all
            % dimensions. The total number of neighbours should be three  to the power Ndim
            % minus one if we discard the original point itself

            % initialize the shift index array
            nneighb = 3^ndimA;
            nbi = zeros( nneighb, ndimA);

            di = ndimA;
            while ( di )
                N = 3^(di-1);
                ni = 1:N;
                while( ni(end) < nneighb+1 )
                    for val=[-1, 0, 1]
                        nbi( ni ,di ) = val;
                        ni = ni+ N;
                    end
                end
                di = di-1;
            end
            % remove diagonal
            diagonal_idx = sum(abs(nbi),2) > 1;
            nbi(diagonal_idx,:) = [];


            %% Create distance matrix
            d = ones(nneighb, 1) * res;
            d(diagonal_idx,:) = [];
            d = d.*abs( nbi );
            % create a row vector with distances
            dvec = sqrt( sum( d.^2, 2))';
            % Get index to exclude the original point: distance = 0
            notorig = logical( dvec > 0 );

            nneighb = size(nbi,1);
            %% Add the input index array to nbi to get all neighbours
            % set up the array for neighbour indices
            nd = length( index);
            Iadj = zeros( nd, nneighb );
            kdo = notorig(ones(nd,1), : );

            for di = 1:ndimA
                indices = imat( :, di );
                shifts = nbi( :, di )';
                neighbindices = indices( :, ones( 1,nneighb)) +shifts( ones(nd, 1), : ) ;
                maxmat = sizeA( di );
                % set up mask matrix to keep indices within limits and excllude the original point
                s = logical( neighbindices <= maxmat );
                s =logical( neighbindices > 0 ) & s;
                kdo = kdo & s;
                % Calculate the linear index
                if di == 1
                    Iadj( kdo ) =  neighbindices( kdo );
                else
                    blocksize = prod( sizeA( 1:(di-1)  ) );
                    m = neighbindices-1;
                    Iadj(kdo )  = Iadj(kdo )+ m(kdo)*blocksize;
                end
            end

            %% Select only the sensible points for the neighbour index and distances matrices
            % Remove columns that have no valid indices anywhere at all (e.g. origin)
            % for shorter index lists with  all points near the edges more may be
            % removed.
            if nd == 1
                allkdo = any( kdo, 1);
                Iadj = Iadj( :, allkdo);
                Radj = dvec( allkdo );
                Nfound = length(  find( allkdo ) );
            else
                Nfound = nneighb-1;
                Radj = dvec;
                iself = (Radj == 0);
                Iadj = Iadj(:,~iself);
                Radj = Radj(~iself);
            end

        end
    end
end
