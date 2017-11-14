% Unit test of the model configuration.
%
% Author        : Darwin LAU
% Created       : 2016
% Description    :
%    Unit tests to confirm that the model configuration is correct. 
classdef ScriptTest < matlab.unittest.TestCase
    methods(Test)
        function exampleScriptTest(testCase)
            close all; clc; warning('off');
            % Can be set up to be neater by removing . and .. using
            % cellfunc
            
            % First go through the directories to extract each script in the examples folder
            folder_location = [CASPR_configuration.LoadHomePath(),'/scripts/examples/'];
            dir_list = dir(folder_location);
            % Remove the . and .. options
            s_l = size(dir_list,1);
            script_list = cell(0);
            if(s_l>2)
                dir_list = dir_list(3:s_l);
                s_el = s_l - 2; i = 1;
                script_number = 1;
                while(i <= s_el)
                    if(dir_list(i).isdir)
                        % Add the elements
                        temp_dir_list = dir([folder_location,dir_list(i).name]);
                        t_s_l = size(temp_dir_list,1);
                        if(t_s_l > 2)
                            dir_list(s_el+1:s_el+t_s_l-2) = temp_dir_list(3:t_s_l);
                            s_el = s_el + t_s_l - 2;
                        end
                    else
                        % Assume it is a script
                        script_list(script_number) = {dir_list(i).name};
                        script_number = script_number + 1;
                    end
                    i = i+1;
                end
            end
            for i = 1:size(script_list,2)
                fprintf('Testing %s \r\n',script_list{i})
                assert(ScriptTest.script_parser(script_list{i}),'Test Failed');
            end
            warning('on');
            disp('All script tests passed');
        end
    end
    
    methods(Access = private,Static)
        function [error_free] = script_parser(file_name)
            error_free = true;
            % Open the file
            fid = fopen(file_name);
            file_str = [];
            while(~feof(fid))
                str = fgets(fid);
                if(numel(str)>0&&(~strcmp(str(1),'%'))&&(isempty(strfind(str,'clc;'))))
                    file_str = [file_str,str,';'];
                end
            end
            try
                eval(file_str)
            catch
                error_free = false;
                fclose(fid);
                close all;
                return
            end
            drawnow;
            fclose(fid);
            close all;
            drawnow;
        end
    end
end