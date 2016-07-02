% Base constraint with abstract methods defined
%
% Author        : Darwin LAU
% Created       : 2016
% Description	:
classdef IDConstraint < handle
    methods (Abstract)
        % Update the system constraints given the dynamics
        updateConstraint(obj, dynamics);
        % A function to verify that constraints have been satisfied by the
        % candidate.
        satisfied = verifyConstraint(obj, x);
    end
end
