% A container class to hold the information from the ray-based trajectory
% verification
% Author: Zeqing ZHANG
% Date: 03/2019


classdef RayTrajectoryElement < handle
    properties(SetAccess = protected)
        conditions              % A cell array of different workspace conditions (enum and intervals)
        number_conditions       % The number of conditions
        translationTrajectory
        quaternionPts
    end
    
    methods
        % Constructor for the class
        function rte = RayTrajectoryElement(n_constraints, QuaternionPts, TranslationTrajectory)
            rte.conditions           =   cell(n_constraints,2);
            rte.number_conditions    =   size(rte.conditions,1);
            rte.translationTrajectory= TranslationTrajectory;
            rte.quaternionPts = QuaternionPts;
        end
        
        % A function to add a new valid trajectory 
        function addTrajectory(obj,condition_type,intervals,index)
            obj.conditions{index,1} = condition_type;
            obj.conditions{index,2} = intervals;
        end
    end
end