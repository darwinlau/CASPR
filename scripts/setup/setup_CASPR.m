% Installation file for CASPR. Please run this function to ensure that
% CASPR has been correctly installed and to ensure that all dependencies
% are set up.
% Author:       Jonathan EDEN
% Created:      2016
% Description:
function setup_CASPR
    clc;
    cd('../..'); home_path = cd;
    try
        cd('src/Analysis');
    catch
        error('Incorrect folder structure or you have not called the function from the setup directory');
    end

    fprintf('\n----------------------------------------------------\n')
    fprintf('Checking Environment and Dependency Installation \n')
    fprintf('----------------------------------------------------\n\n')

    cd(home_path);
    % Check the status of the dependencies
    version_flag = check_dependencies();
    
    fprintf('\n----------------------------------------------------\n')
    fprintf('Initialising the Libraries\n')
    fprintf('----------------------------------------------------\n\n')
    
    % Add toolbox path to MATLAB (this is temporary)
    ls
    initialise_CASPR;
    
    fprintf('\n----------------------------------------------------\n')
    fprintf('Testing that the model has been correctly configured\n')
    fprintf('----------------------------------------------------\n\n')

    % Run unit tests to confirm that the models are correctly setup
    %suite = matlab.unittest.TestSuite.fromFile('ModelConfigTest.m');
    %suite.run;
    failed_tests = CASPR_tests('ModelConfigTest.m');
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


% Check the dependencies have been correctly setup.
function old_matlab_version = check_dependencies()
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
            opti_Install_Test(1)
        end
    end
    
    % Test for qhull
    fprintf('- Checking OptiToolbox...\n\r');
    if(isunix)
        qhull_file = '/dependencies/qhull-2012.1/bin/qconvex.exe';
    else
        qhull_file = '/dependencies/qhull-2012.1/bin/qconvex';
    end
    if(exist(qhull_file,'file'))
        fprintf('qhull is built to specificiations\n\r')
    else
        fprintf('[WARNING]:  You do not seem to have qhull installed or it is not in the expected location.\n\r');
    end
end
