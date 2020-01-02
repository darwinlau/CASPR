% Logging facilities for MATLAB.  Log can be written to either console or
% file depending upon the specified input.
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   : A class for CASPR logging. The default that is set on
% initilisation is warning level with console printing.
classdef CASPR_log
    methods (Static)
        % Set logging level
        function SetLoggingDetails(log_level, log_path)
            % Test that log level is valid
            assert(isa(log_level,'CASPRLogLevel'), 'log_level must be an enum from CASPRLogLevel');
            % Open the log file fresh
            if(nargin == 2 && ~isempty(log_path))
                % Reset the log file to be empty
                fid = fopen(log_path,'w'); fclose(fid);
            else
                log_path = [];
            end
            CASPR_homepath = CASPR_configuration.LoadHomePath();
            if(~exist([CASPR_homepath,'/data/config'],'dir'))
                CASPR_log.Error('CASPR environment file must be set');
            end
            save([CASPR_homepath,'/data/config/CASPR_environment.mat'], 'log_level', 'log_path','-append');
        end
        
        function level = GetLogLevel()
            model_config = load('CASPR_environment.mat', 'log_level');
            level = model_config.log_level;            
        end
        
        % Prints debug statement
        function Debug(str)
            CASPR_log.Print(str, CASPRLogLevel.DEBUG);
        end
                
        % Prints 
        function Info(str)
            CASPR_log.Print(str, CASPRLogLevel.INFO);
        end
        
        function Warn(str)
            CASPR_log.Print(str, CASPRLogLevel.WARNING);
        end
        
        function Error(str)
            CASPR_log.Print(str, CASPRLogLevel.ERROR);
        end
        
        function Assert(condition, str)
            if(~condition)
                CASPR_log.Error(str);
            end
        end
        
        % Logging print
        function Print(str, log_level)
            [set_log_level, fid, carrage_return] = CASPR_log.Extract();   
            if (log_level >= set_log_level)
                switch (log_level)
                    case CASPRLogLevel.DEBUG
                        fprintf(fid, '[DEBUG] %s', str);
                        fprintf(fid, newline);
                    case CASPRLogLevel.INFO
                        fprintf(fid, '[INFO] %s', str);
                        fprintf(fid, newline);
                    case CASPRLogLevel.WARNING
                        fprintf(fid, '[WARNING] %s', str);
                        fprintf(fid, newline);
                        warning(str);
                    case CASPRLogLevel.ERROR
                        fprintf(fid, '[ERROR] %s', str);
                        fprintf(fid, newline);
                        error(str);
                    otherwise
                        error('The specified log level is not valid');
                end
            end
            
            if(fid ~= 1)
                fclose(fid);
            end
        end
    end
    
    methods(Access = private, Static)
        function [log_level,fid,carrage_return] = Extract()
            load('data/config/CASPR_environment.mat','log_level','log_path');
            % Determine the what to print to
            if(isempty(log_path))
                fid = 1;
%                 carrage_return = '\n';
                carrage_return = '\n\r';
            else
                fid = fopen(log_path,'a');
                carrage_return = '\n\r';
%                 if(isunix||ismac)
%                     carrage_return = '\n';
%                 else
%                     carrage_return = '\r\n';
%                 end
            end
        end
    end
end
