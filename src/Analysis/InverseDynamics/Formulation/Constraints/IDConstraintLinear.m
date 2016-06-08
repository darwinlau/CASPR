% Linear constraints for inverse dynamics
% 
% Author        : Darwin LAU
% Created       : 2016
% Description	: The linear function for ID is of the form A*x <= b,
% where x is the variable, A is the LHS linear matrix and b is the RHS vector
classdef IDConstraintLinear < IDConstraint
    properties
        A 
        b
    end
    
    methods
        % The implementation of constraint verification.
        function satisfied = verifyConstraint(obj, x)
            satisfied = all(obj.A * x <= obj.b);
        end
    end
end

