% Enum for the type of metric
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   :
classdef WorkspaceMetricType
    enumeration 
        % Acceleration Set
        SEACM
        % Wrench Set
        CAPACITY_MARGIN
        % Dexterity
        TENSION_FACTOR
        TENSION_FACTOR_MODIFIED
        UNILATERAL_DEXTERITY
        UNILATERAL_MAXIMUM_FORCE_AMPLIFICATION
        CONDITION_NUMBER
        %Interference
        MIN_CABLE_CABLE_DISTANCE
    end
end

