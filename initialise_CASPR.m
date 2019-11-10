% A function to temporarily add the CASPR libraries
% Author:       Jonathan EDEN
% Created:      2016
% Description:
function initialise_CASPR()
    clc;    
    scriptname = mfilename('fullpath');
    [CASPR_homepath] = fileparts(scriptname);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Confirm that initialise has been called from the right folder
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    assert(exist([CASPR_homepath '/src'], 'dir') == 7 && exist([CASPR_homepath '/data'], 'dir') == 7, 'Incorrect CASPR folder structure');
    
    cd(CASPR_homepath);
    % Set the current version
    CASPR_version = 20191021;
    CASPR_model_config_path = [CASPR_homepath,'/data/model_config'];
    CASPR_GUI_dev_model_config = 0; % Developmental models are not shown
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Determine if setup needs to be executed
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if (~exist([CASPR_homepath,'/data/config'],'dir') || ~exist([CASPR_homepath,'/data/config/CASPR_environment.mat'], 'file'))
        mkdir([CASPR_homepath,'/data/config'])
        save([CASPR_homepath,'/data/config/CASPR_environment.mat'],...
                                    'CASPR_homepath','CASPR_version','CASPR_model_config_path','CASPR_GUI_dev_model_config');
        setup_CASPR;
    else
        previous_version = load([CASPR_homepath,'/data/config/CASPR_environment.mat'],'CASPR_version');
        if(isempty(fieldnames(previous_version))||(CASPR_version > previous_version.CASPR_version))
            fprintf('\n----------------------------------------------------\n')
            fprintf('Updating the CASPR environment variables\n')
            fprintf('----------------------------------------------------\n')
            mat = load([CASPR_homepath,'/data/config/CASPR_environment.mat']);
            CASPR_homepath = mat.CASPR_homepath;
            CASPR_model_config_path = mat.CASPR_model_config_path;
            CASPR_GUI_dev_model_config = mat.CASPR_GUI_dev_model_config;
            log_level = mat.log_level;
            log_path = mat.log_path;
            save([CASPR_homepath,'/data/config/CASPR_environment.mat'], 'CASPR_homepath', 'CASPR_version', 'CASPR_model_config_path', 'CASPR_GUI_dev_model_config', 'log_level', 'log_path');
        else
            save([CASPR_homepath,'/data/config/CASPR_environment.mat'], 'CASPR_homepath', 'CASPR_version', 'CASPR_model_config_path', '-append');
        end
        set_CASPR_environment;
        fprintf('CASPR initialisation complete. Enjoy !\n');
    end
end

% Function for setting up the CASPR. This first checks the system
% environment and then runs and a setup and update to create the necessary
% folders.
function setup_CASPR()
    % Check the status of the dependencies
    fprintf('\n----------------------------------------------------\n')
    fprintf('Checking Environment and Dependency Installation \n')
    fprintf('----------------------------------------------------\n')
    check_system_environment;
    
    % Temporarily add the CASPR libraries to the path
    set_CASPR_environment;
    
    % Setup logging
    fprintf('\n----------------------------------------------------\n')
    fprintf('Set up Logging\n')
    fprintf('----------------------------------------------------\n')
    CASPR_log.SetLoggingDetails(CASPRLogLevel.INFO);
    fprintf('Logging successfully setup\n');
    
    % Test that the models are correctly configured
    fprintf('\n----------------------------------------------------\n')
    fprintf('Running CASPRTestScript to make sure everything is working\n')
    fprintf('----------------------------------------------------\n')
    % Run unit tests to confirm that the models are correctly setup
    %suite = matlab.unittest.TestSuite.fromFile('ModelConfigTest.m');
    %suite.run;
    num_tests_failed = CASPRTestScript();
    
    fprintf('\n----------------------------------------------------\n')
    fprintf('CASPR Setup Complete. Enjoy!\n');
    if num_tests_failed > 0
        fprintf('WARNING: Some unit tests failed. Please contact the maintainers of CASPR with the test summary information\n');
    end
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
	fprintf('MATLAB version: %s\n\r',mver.Release);
    
    % Version 8.6 is 2015b
    if (verLessThan('matlab', '8.6'))
        old_matlab_version = 1; 
        fprintf('WARNING: CASPR is designed for MATLAB versions from 2015b onwards. Certain functionality may not work on this version\n\r');
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
        if(~contains(p,'OptiToolbox'))
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
    CASPR_homepath = cd;
    
    % Add the necessary paths
    fprintf('Adding CASPR to library path\n')
    path_list = genpath(CASPR_homepath);
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