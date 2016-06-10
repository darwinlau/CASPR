% A function to temporarily add the CASPR libraries
% Author:       Jonathan EDEN
% Created:      2016
% Description:
function initialise_CASPR()
    % Remove any path that contains CASPR
    fprintf('Cleaning Libraries\n')
    p = path;
    p = strsplit(p,':');
    index = false(size(p));
    for i = 1:length(index)
        if(~isempty(strfind(p{i},'CASPR')))
            index(i) = true;
        end
    end
    if(sum(index)>0)
        temp_p = p(index);
        rmpath(temp_p{ : });
    end
    
    % Add the necessaey paths
    fprintf('Adding Libraries\n')
    path_list = genpath(cd);
    path_list = strsplit(path_list,':');
    for i = 1:length(path_list)
        if(~isempty(strfind(path_list{i},'.git'))||~isempty(strfind(path_list{i},'/log/')))
            path_list{i} = '';
        end
    end
    addpath(path_list{:});
    rehash
    fprintf('Libraries have been set up\n')
end

