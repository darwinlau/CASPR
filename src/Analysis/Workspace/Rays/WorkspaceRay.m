% A container class to hold workspace analysis information for ray based
% generation. NOTE: For the moment this class will assume that the ray is
% generated about one of the principle axes.
%
% Author        : Jonathan EDEN
% Created       : 2017
% Description    : This class contains the known information obtained
% through workspace analysis.  That is the pose and any metrics/workspace
% conditions that have been evalauted at that ray.
classdef WorkspaceRay < handle
    properties(SetAccess = protected)
        fixed_variables         % The pose for the workspace condition to be evaluated at
        metrics                 % A cell array of different metrics (enum and value)
        conditions              % A cell array of different workspace conditions (enum and intervals)
        free_variable_index     % The index that is left free
        free_variable_range     % The range of values that the free variable takes (1st element should be min and 2nd element max)
        % FOR FUTURE EXTENSION
        % CHANGE THE RAY TO BE STORED AS Y=MX+C FORM WHERE STORE THE
        % GRADIENT M (made into a unit vector), THE OFFSET C AND THE
        % RANGE [X_MIN,X_MAX]
    end
    
    methods
        % Constructor for the class
        function wp = WorkspaceRay(fixed_variables,n_metrics,n_constraints,free_variable_index,free_variable_range)
            wp.fixed_variables      =   fixed_variables;
            wp.metrics              =   cell(n_metrics,2);
            wp.conditions           =   cell(n_constraints,2);
            wp.free_variable_index  =   free_variable_index;
            wp.free_variable_range  =   free_variable_range;
            % FOR FUTURE EXTENSION
            % INPUT M, C, AND X
        end
        
        % A function to add the metric information to the point
        function addMetric(obj,metric_type,metric_value,index)
            obj.metrics{index,1} = metric_type;
            obj.metrics{index,2} = metric_value;
        end
        
        % A function to add a new condition to the point
        function addCondition(obj,condition_type,intervals,index)
            obj.conditions{index,1} = condition_type;
            obj.conditions{index,2} = intervals;
        end
        
        % A function to determine if two rays intersect
        function is_intersected = intersect(obj,workspace_ray)
            % MODIFY FOR INTERSECTION VERSUS UNION
            if(obj.free_variable_index == workspace_ray.free_variable_index)
                % The rays are parallel
                if(norm(obj.fixed_variables - workspace_ray.free_variable_index) < 1e-10)
                    % Covers the case where the same ray is sent
                    is_intersected = 1;
                else
                    is_intersected = 0;
                end
            else
                % The rays are not parallel
                % Determine if the rays are coplanar
                numDofs = length(obj.fixed_variables)+1;
                ones_vec = true(numDofs,1); 
                q_obj = zeros(numDofs,1);
                selection_vec_obj = ones_vec; selection_vec_obj(obj.free_variable_index) = false;
                q_obj(selection_vec_obj) = obj.fixed_variables; 
                fixed_variable_ray = q_obj(workspace_ray.free_variable_index);
                q_obj(workspace_ray.free_variable_index) = 0;
                q_ray = zeros(numDofs,1);
                selection_vec_ray = ones_vec; selection_vec_ray(workspace_ray.free_variable_index) = false;
                q_ray(selection_vec_ray) = workspace_ray.fixed_variables; 
                fixed_variable_obj = q_ray(obj.free_variable_index);
                q_ray(obj.free_variable_index) = 0;
                if(norm(q_obj - q_ray) < 1e-10)
                    % The two rays are coplanar
                    % Determine if they intersect with overlapping region
                    numConditionsObj = size(obj.conditions,1);
                    objRayIntersect = 0; % Does the fixed value of ray intersect with any of the intervals of obj
                    for i = 1:numConditionsObj
                        intervals = obj.conditions{i,2};
                        for j = 1:size(intervals,1)
                            if((intervals(j,1) <= fixed_variable_obj)&&(intervals(j,2) >= fixed_variable_obj))
                                objRayIntersect = 1;
                                break; 
                            end
                        end
                        if(objRayIntersect)
                            break;
                        end
                    end
                    numConditionsRay = size(workspace_ray.conditions,1);
                    rayObjIntersect = 0; % Does the fixed value of obj intersect with any of the intervals of ray
                    for i = 1:numConditionsRay
                        intervals = workspace_ray.conditions{i,2};
                        for j = 1:size(intervals,1)
                            if((intervals(j,1) <= fixed_variable_ray)&&(intervals(j,2) >= fixed_variable_ray))
                                rayObjIntersect = 1;
                                break; 
                            end
                        end
                        if(rayObjIntersect)
                            break;
                        end
                    end                    
                    is_intersected = objRayIntersect*rayObjIntersect;
                else
                    % The two rays are not coplanar
                    is_intersected = 0;
                end
            end
            % THIS METHODOLOGY NEEDS TO BE MODIFIED FOR THE TRANSFORMATION
            % COORDINATES. IN PARTICULAR THE TWO RAYS MAY NOT POSSESS THE
            % SAME COORDINATES AND THEREFORE THE RAY SHOULD BE DESCRIBED IN
            % TERMS OF A UNIT GRADIENT, RAY BOUNDS AND OFFSET TERM
        end
    end    
end