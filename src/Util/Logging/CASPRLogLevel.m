% Enum for the level of log
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   :
classdef CASPRLogLevel
    enumeration 
        DEBUG       % Display all information
        INFO        % Display computational information in addition to warnings and errors
        WARNING     % Display any warnings and errors
        ERROR       % Only display fatal warnings.
    end
end