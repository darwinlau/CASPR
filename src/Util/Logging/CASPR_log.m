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
            home_path = cd;
            if(~exist([home_path,'/data/config'],'dir'))
                mkdir([home_path,'/data/config'])
            end
            save([home_path,'/data/config/log_level.mat'], 'log_level', 'log_path');
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
                        fprintf(fid,['[DEBUG] ', str, carrage_return]);
                    case CASPRLogLevel.INFO
                        fprintf(fid,['[INFO] ', str, carrage_return]);
                    case CASPRLogLevel.WARNING
                        fprintf(fid,['[WARNING] ', str, carrage_return]);
                        warning(str);
                    case CASPRLogLevel.ERROR
                        fprintf(fid,['[ERROR] ', str, carrage_return]);
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
            % Load the logging information and the fprintf location
            home_path = cd;
            log_struct = load([home_path,'/data/config/log_level.mat']);
            % Determine the what to print to
            if(isempty(log_struct.log_path))
                fid = 1;
                carrage_return = '\n';
            else
                fid = fopen(log_struct.log_path,'a');
                if(isunix||ismac)
                    carrage_return = '\n';
                else
                    carrage_return = '\r\n';
                end
            end
            log_level = log_struct.log_level;
        end
    end
end
