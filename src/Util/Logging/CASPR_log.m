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
        function SetLoggingDetails(log_level,log_path)
            % Test that log level is valid
            assert(isa(log_level,'CASPRLogLevel'),'log_level must be an enum from CASPRLogLevel');
            save('logs/log_level.mat','log_level','log_path');
        end
        
        % Logging print
        function Print(str,log_level)
            % Load the logging information and the fprintf location
            log_struct = load('logs/log_level.mat');
            % Determine the what to print to
            if(isempty(log_struct.log_path))
                fid = 1;
                carrage_return = '\n';
            else
                fid = fopen(log_struct.log_path);
                if(isunix)
                    carrage_return = '\n';
                else
                    carrage_return = '\r\n';
                end
            end
            switch(log_level)
                case CASPRLogLevel.DEBUG
                    if(log_struct.log_level == CASPRLogLevel.DEBUG)
                        fprintf(fid,['DEBUG: ',str,carrage_return]);
                    end
                case CASPRLogLevel.INFO
                    if((log_struct.log_level == CASPRLogLevel.DEBUG)||(log_struct.log_level == CASPRLogLevel.INFO))
                        fprintf(fid,['INFO: ',str,carrage_return]);
                    end
                case CASPRLogLevel.WARNING
                    if(log_struct.log_level ~= CASPRLogLevel.ERROR)
                        fprintf(fid,['WARNING: ',str,carrage_return]);
                        warning(str)
                    end
                case CASPRLogLevel.ERROR
                    fprintf(fid,['ERROR: ',str,carrage_return]);
                    error(str)
                otherwise
                    error('The specified log level is not valid');
            end
            if(fid~=1)
                fclose(fid);
            end
        end
        
        function Assert(condition,str)
            if(condition)
                fprintf(fid,['ERROR: ',str,carrage_return]);
                error(str)
            end
        end
    end
end
