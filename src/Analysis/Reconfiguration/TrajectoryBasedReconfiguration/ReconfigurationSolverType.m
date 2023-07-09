% Enum for the type of ray workspace condition
%
% Author        : Paul Cheng
% Created       : 2022
% Description   :
classdef ReconfigurationSolverType
    enumeration 
        GloptiPoly
        Fmincon
        Ga
        Paretosearch 
        PSO
    end
end

