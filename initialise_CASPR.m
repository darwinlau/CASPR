% A function to temporarily add the CASPR libraries
% Author:       Jonathan EDEN
% Created:      2016
% Description:
function initialise_CASPR()
    clc;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Confirm that initialise has been called from the right folder
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    home_path = cd;
    try
        cd('src/Analysis');
    catch
        error('Incorrect folder structure or you have not called the function from the CASPR root directory');
    end
    cd(home_path);
    % Set the current version
    version = 20161019;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Determine if setup needs to be executed
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if(~exist([home_path,'/data/config'],'dir'))
        mkdir([home_path,'/data/config'])
        save([home_path,'/data/config/CASPR_environment.mat'],...
                                    'home_path','version');
        setup_CASPR;
    else
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Determine if CASPR needs to be updated
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % load the previous version information
        previous_version = load([home_path,'/data/config/CASPR_environment.mat'],'version');
        if(isempty(fieldnames(previous_version))||(version>previous_version.version))
            save([home_path,'/data/config/CASPR_environment.mat'],'home_path','version');
            update_CASPR;
        else
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Add the libraries
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            save([home_path,'/data/config/CASPR_environment.mat'],'home_path','version');
            set_CASPR_environment;
            fprintf('CASPR initialisation complete. Enjoy !\n')
        end
    end
end

function setup_CASPR()
    % Check the status of the dependencies
    fprintf('\n----------------------------------------------------\n')
    fprintf('Checking Environment and Dependency Installation \n')
    fprintf('----------------------------------------------------\n')
    check_system_environment;

    % Run the rest of setup (setup environment and logging, then test the
    % system).
    failed_tests = setup_update_CASPR();
    if(sum(failed_tests)>0)
        fprintf('\n----------------------------------------------------\n')
        fprintf('CASPR Failed to Install. Please view the Unit Test Output\n')
        fprintf('----------------------------------------------------\n\n')
        return;
    end
    
    fprintf('\n----------------------------------------------------\n')
    fprintf('CASPR Setup Complete. Enjoy!\n')
    fprintf('----------------------------------------------------\n\n')
end

function update_CASPR()
    fprintf('\n----------------------------------------------------\n')
    fprintf('Updating CASPR \n')
    fprintf('----------------------------------------------------\n')

    % Check if the old logs folder is present and remove if so
    home_path = cd;
    if(exist([home_path,'/logs'],'dir'))
        fprintf('\n----------------------------------------------------\n')
        fprintf('Removing outdated folder locations.\n')
        fprintf('----------------------------------------------------\n')
        rmdir([home_path,'/logs']); 
    end
    
    failed_tests = setup_update_CASPR;
    if(sum(failed_tests)>0)
        fprintf('\n----------------------------------------------------\n')
        fprintf('CASPR Failed to Update. Please view the Unit Test Output\n')
        fprintf('----------------------------------------------------\n\n')
        return;
    end
    
    fprintf('\n----------------------------------------------------\n')
    fprintf('CASPR Update Complete. Enjoy!\n')
    fprintf('----------------------------------------------------\n\n')
end

% Check the dependencies have been correctly setup.
function old_matlab_version = check_system_environment()
    % First check it is a known operating system
    assert(ismac || isunix || ispc, 'Operating system platform not supported');
    old_matlab_version = 0;
    
    % Code to test the matlab version
    mver = ver('MATLAB');
    fprintf('- Checking MATLAB version... \n\r');
    vv = regexp(mver.Version,'\.','split');
	fprintf('MATLAB version: %s\n\r',mver.Release);
    if(str2double(vv{1}) < 9)
        if(str2double(vv{2}) < 3)
            old_matlab_version = 1; 
            fprintf('WARNING: CASPR is designed for MATLAB versions from 2014a onwards. Certain functionality may not work on this version\n\r');
        end
    end
    
    % Optitoolbox
    fprintf('- Checking OptiToolbox...\n\r');
    if(~strcmp(mexext,'mexw32')&&~strcmp(mexext,'mexw64'))
       fprintf('[WARNING]: OPTI Toolbox is compiled only for Windows systems. Some functionality will be lost for this version. \n\r');
    else
        % Test opti
        if(strcmp(mexext,'mexw32'))
            fprintf('MATLAB %s 32bit (Windows x86) detected\n\r',mver.Release);
        else
            fprintf('MATLAB %s 64bit (Windows x64) detected\n\r',mver.Release);
        end
        % Test if optitoolbox is in the path
        p = path;
        if(isempty(strfind(p,'OptiToolbox')))
            fprintf('[WARNING]: OptiToolbox is not your matlab file path.\n\r');
        else
            fprintf('OptiToolbox found on the path.\n\r');
            %opti_Install_Test(1);
        end
    end
    
    % Test for qhull
    fprintf('- Checking qhull...\n\r');
    if(isunix)
        qhull_file = '/dependencies/qhull-2012.1/bin/qconvex';
    else
        qhull_file = '/dependencies/qhull-2012.1/bin/qconvex.exe';
    end
    if(exist(qhull_file,'file'))
        fprintf('qhull is built to specificiations\n\r')
    else
        fprintf('[WARNING]:  You do not seem to have qhull installed or it is not in the expected location.\n\r');
    end
end

function set_CASPR_environment()
    % Workout the delimiter symbol for PATH string
    if ismac || isunix
        path_delimiter = ':';
    elseif ispc
        path_delimiter = ';';
    else
        error('Platform not supported');
    end
    
    fprintf('---------------------------------------------\n')
    fprintf('Initialising CASPR environment\n')
    fprintf('---------------------------------------------\n')
    % Remove any path that contains CASPR
    fprintf('Cleaning CASPR from library path\n')
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
    
    % Store the home directory
    home_path = cd;
    
    
    % Add the necessary paths
    fprintf('Adding CASPR to library path\n')
    path_list = genpath(home_path);
    path_list = strsplit(path_list, path_delimiter);
    for i = 1:length(path_list)
        if(~isempty(strfind(path_list{i},'.git')))
            path_list{i} = '';
        end
    end
    addpath(path_list{:});
    rehash
    fprintf('CASPR paths have been successfully set up \n')
end

function failed_tests = setup_update_CASPR()
    % Temporarily add the CASPR libraries to the path
    set_CASPR_environment;
    
    % Setup logging
    fprintf('\n----------------------------------------------------\n')
    fprintf('Set up Logging\n')
    fprintf('----------------------------------------------------\n')
    CASPR_log.SetLoggingDetails(CASPRLogLevel.INFO);
    fprintf('Logging sucessively setup\n');
    
    % Test that the models are correctly configured
    fprintf('\n----------------------------------------------------\n')
    fprintf('Testing that the model has been correctly configured\n')
    fprintf('----------------------------------------------------\n')
    % Run unit tests to confirm that the models are correctly setup
    %suite = matlab.unittest.TestSuite.fromFile('ModelConfigTest.m');
    %suite.run;
    failed_tests = CASPRTestScript;
end