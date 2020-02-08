% Class to compute whether a pose (dynamics) is within the wrench feasible
% workspace (WFW)
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    : Class for evalaution of WFC.
classdef DynamicFeasibleCondition < WorkspaceConditionBase
    properties (Constant)
        % Type of workspace condition (WorkspaceConditionType enum)
        type = WorkspaceConditionType.DYNAMIC_FEASIBLE;
    end
    
    properties (SetAccess = protected)
        % vertices of the corresponding sets, each row corresponds to a
        % vectex
        desiredWrenchSet
        desiredAccSet
        desiredVelSet
    end
    
    properties (SetAccess = protected, GetAccess = protected)
        % pre-processing result - velocity minimum velocity enclosing
        % ellipsoid (MVEE)
        cv
        Av      % (v - c)'*A*(v - c) = 1
        Pv      % Av = Pv'*Pv
        Pv_inv  % Pv_inv = inv(Pv)
    end
    
    methods
        % Constructor for dynamic feasible workspace
        function w = DynamicFeasibleCondition(desiredVelSet, desiredAccSet, desiredWrenchSet, method)
            if(nargin < 4 || isempty(method))
                w.method = DynamicFeasibleMethodType.M_CONSTANT_VELOCITY_MATLAB;
            else
                w.method = method; 
            end
            w.desiredWrenchSet = desiredWrenchSet;
            w.desiredAccSet = desiredAccSet;
            w.desiredVelSet = desiredVelSet;
            w.Av = [];
            w.cv = [];
            w.Pv = [];
            w.Pv_inv = [];
            
            % pre-processing and sanity check
            switch(w.method)
                case DynamicFeasibleMethodType.M_CONSTANT_VELOCITY_MATLAB
                    if size(w.desiredVelSet,1) > 1
                        CASPR_log.Error('A single velocity is expected, not a set.');
                    end
                case DynamicFeasibleMethodType.M_VELOCITY_SET_QUADRATIC_C_MATLAB
                    % pre-processing: calculate the minimum volume
                    % enclosing ellipsoid for joint velocity
                    tol = 1e-3;
                    [w.Av, w.cv] = MinVolEllipse(desiredVelSet', tol);
                    % D is a diagonal matrix and since w.Av is symmetric,
                    % the eigenvectors are digonal to each other hence V is
                    % a orthogonal matrix satisfying: A = V*D*V';
                    [V,D] = eig(w.Av);
                    w.Pv = sqrt(D)*V';
                    w.Pv_inv = w.Pv\eye(size(w.Pv));
                    if norm(w.cv) > 1e-10
                        CASPR_log.Warn('The given velocity set is not symmetric with respect to the origin, the algorithm works better with a velocity set that is.');
                    end
                case DynamicFeasibleMethodType.M_VELOCITY_SET_QUADRATIC_C_CPLEX_INTERVAL
                    if ~(isa(w.desiredVelSet,'intval') && isa(w.desiredAccSet,'intval') && isa(w.desiredWrenchSet,'intval'))
                        CASPR_log.Error('The Velocity/Acceleration/External Wrench sets should be intervals.');
                    end
                    % pre-processing: calculate the minimum volume
                    % enclosing ellipsoid for joint velocity
                    % step 1: find the vertices 
                    n = length(desiredVelSet);
                    vel_lb = desiredVelSet.inf';
                    vel_ub = desiredVelSet.sup';
                    numVertices = 2^n;
                    vertices = zeros(numVertices, n);
                    offset = double('0');
                    for i = 1:numVertices
                        beta = double(dec2bin(i-1,n)) - offset;
                        vertices(i,:) = vel_lb.*(~beta) + vel_ub.*beta;
%                         % an alternative approach but seems to be slower
%                         flag_set = dec2bin(i-1,numDofs);
%                         flag_vec = str2num(flag_set')';
%                         vertices(i,:) = vel_lb.*(~flag_vec) + vel_ub.*flag_vec;
                    end
                    tol = 1e-3;
                    [w.Av, w.cv] = MinVolEllipse(vertices', tol);
                    % D is a diagonal matrix and since w.Av is symmetric,
                    % the eigenvectors are digonal to each other hence V is
                    % a orthogonal matrix satisfying: A = V*D*V';
                    [V,D] = eig(w.Av);
                    w.Pv = sqrt(D)*V';
                    w.Pv_inv = w.Pv\eye(size(w.Pv));
                    if norm(w.cv) > 1e-10
                        CASPR_log.Warn('The given velocity set is not symmetric with respect to the origin, the algorithm works better with a velocity set that is.');
                    end
                otherwise
                    CASPR_log.Print('Wrench feasible method is not defined',CASPRLogLevel.ERROR);
            end
        end
        
        % Evaluate the wrench closure condition return true if satisfied 
        function inWorkspace = evaluateFunction(obj, dynamics, ~)
            switch(obj.method)
                case DynamicFeasibleMethodType.M_CONSTANT_VELOCITY_MATLAB
                    if size(obj.desiredVelSet,1) > 1
                        CASPR_log.Error('A single velocity is expected, not a set.');
                    end
                    inWorkspace = dynamic_feasible_constant_velocity_MATLAB(obj.desiredVelSet, obj.desiredAccSet, obj.desiredWrenchSet,dynamics);
                case DynamicFeasibleMethodType.M_VELOCITY_SET_QUADRATIC_C_MATLAB
                    inWorkspace = dynamic_feasible_velocity_set_quadratic_C_MATLAB(obj.Pv_inv, obj.cv, obj.desiredAccSet, obj.desiredWrenchSet,dynamics);
                case DynamicFeasibleMethodType.M_VELOCITY_SET_QUADRATIC_C_CPLEX_INTERVAL
                    if ~(isa(obj.desiredVelSet,'intval') && isa(obj.desiredAccSet,'intval') && isa(obj.desiredWrenchSet,'intval'))
                        CASPR_log.Error('The Velocity/Acceleration/External Wrench sets should be intervals.');
                    end
                    inWorkspace = dynamic_feasible_velocity_set_quadratic_C_interval_CPLEX(obj.Pv_inv, obj.cv, obj.desiredAccSet, obj.desiredWrenchSet,dynamics);
                otherwise
                    CASPR_log.Print('Wrench feasible method is not defined',CASPRLogLevel.ERROR);
            end
        end
    end
end