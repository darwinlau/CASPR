% A function to temporarily add the CASPR libraries
% Author:       Jonathan EDEN
% Created:      2016
% Description:
function initialise_CASPR()
    % Workout the delimiter symbol for PATH string
    if ismac || isunix
        path_delimiter = ':';
    elseif ispc
        path_delimiter = ';';
    else
        error('Platform not supported');
    end
    
    % Remove any path that contains CASPR
    fprintf('Cleaning Libraries\n')
    p = path;
    p = strsplit(p, path_delimiter);
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
    home_path = cd;
    path_list = genpath(home_path);
    path_list = strsplit(path_list, path_delimiter);
    for i = 1:length(path_list)
        if(~isempty(strfind(path_list{i},'.git')))
            path_list{i} = '';
        end
    end
    addpath(path_list{:});
    rehash
    fprintf('Libraries have been set up\n')
    
    % Store the home directory
    if(~exist([home_path,'/logs'],'dir'))
        mkdir('logs')
    end
    save('logs/CASPR_environment.mat','home_path');
end