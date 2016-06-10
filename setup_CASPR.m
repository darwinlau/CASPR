% Installation file for CASPR. Please run this function to ensure that
% CASPR has been correctly installed and to ensure that all dependencies
% are set up.
% Author:       Jonathan EDEN
% Created:      2016
% Description:
function setup_CASPR
    clc;
    home_path = cd;
    try
        cd('src/Analysis');
    catch
        error('Incorrect folder structure or you have not caled the function from the root directory');
    end

    fprintf('\n----------------------------------------------------\n')
    fprintf('Checking Environment and Dependency Installation \n')
    fprintf('----------------------------------------------------\n\n')

    cd(home_path);
    % Check the status of the dependencies
    if(~check_dependencies())
        return;
    end
    
    fprintf('\n----------------------------------------------------\n')
    fprintf('Initialising the Libraries\n')
    fprintf('----------------------------------------------------\n\n')
    
    % Add toolbox path to MATLAB (this is temporary)
    initialise_CASPR;
    
    fprintf('\n----------------------------------------------------\n')
    fprintf('Testing that the model has been correctly configured\n')
    fprintf('----------------------------------------------------\n\n')

    % Run unit tests to confirm that the models are correctly setup
%     if(~check_unit_tests(home_path))
%         return;
%     end
end


% Check the dependencies have been correctly setup.
function OK = check_dependencies()
    % Code to test the matlab version
    mver = ver('MATLAB');
    fprintf('- Checking MATLAB version... \n\r');
    vv = regexp(mver.Version,'\.','split');
    if(str2double(vv{1}) < 9)
        if(str2double(vv{2}) < 3)
            fprintf('WARNING: CASPR is designed for MATLAB versions from 2014a onwards. Certain functionality may not work on this version\n\r');
        end
    end
    
    % Optitoolbox
    fprintf('- Checking OptiToolbox...\n\r');
    if(~strcmp(mexext,'mexw32')&&~strcmp(mexext,'mexw64'))
       fprintf('WARNING: OPTI Toolbox is compiled only for Windows systems. Some functionality will be lost for this version. \n\r');
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
            fprintf('WARNING: OptiToolbox is not your matlab file path.\n\r');
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
        fprintf('WARNING:  You do not seem to have qhull installed or it is not in the expected location.\n\r');
    end
    OK = 1;
end
