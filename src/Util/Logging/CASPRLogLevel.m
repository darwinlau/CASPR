% Enum for the level of log
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   :
classdef CASPRLogLevel < uint32
    enumeration 
        DEBUG       (1) % Display all information
        INFO        (2) % Display computational information in addition to warnings and errors
        WARNING     (3) % Display any warnings and errors
        ERROR       (4) % Only display errors (this checks all assertions).
    end
end