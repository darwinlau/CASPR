% The simulator to run an inverse dynamics simulation
%
% Author        : Darwin LAU
% Created       : 2013
% Description    :
%   The inverse dynamics simulator solves for the cable forces required to
%   perform a prescribed joint space trajectory. The IDSolver is provided
%   as an input to the simulator to specify the ID algorithm to be used.
classdef InverseDynamicsSimulator < DynamicsSimulator

    properties (SetAccess = protected)
        compTime            % computational time for each time step
        IDFunctionCost      % Cost value for optimisation at each point in time
        IDExitType          % Exit type at each point in time (IDSolverExitType)
        IDSolver
    end

    methods
        function id = InverseDynamicsSimulator(model, id_solver)
            id@DynamicsSimulator(model);
            id.IDSolver = id_solver;
        end

        function run(obj, trajectory)
            errorFlag = 0;
            
            obj.trajectory = trajectory;
            obj.timeVector = obj.trajectory.timeVector;
            % Runs the simulation over the specified trajectory
            obj.cableForces = cell(1, length(obj.trajectory.timeVector));

            obj.IDFunctionCost = zeros(length(obj.timeVector), 1);
            obj.IDExitType = cell(length(obj.timeVector), 1);
%            obj.IDInfo = cell(length(obj.timeVector), 1);

            obj.compTime = zeros(length(obj.timeVector), 1);
%            obj.compIterations = zeros(length(obj.timeVector), 1);

            for t = 1:length(obj.timeVector)
                fprintf('Time : %f\n', obj.timeVector(t));
                % The model is already updated within the resolve function
                [obj.cableForces{t}, obj.IDFunctionCost(t), obj.IDExitType{t}, obj.compTime(t), obj.model] = obj.IDSolver.resolve(obj.trajectory.q{t}, obj.trajectory.q_dot{t}, obj.trajectory.q_ddot{t}, zeros(obj.model.numDofs,1));
                obj.interactionWrench{t} = obj.model.interactionWrench;
                obj.cableLengths{t} = obj.model.cableLengths;
                obj.cableLengthsDot{t} = obj.model.cableLengthsDot;
                
                if (obj.IDExitType{t} ~= IDSolverExitType.NO_ERROR)
                    warning('No feasible solution for the ID');
                    errorFlag = 1;
                end
            end
            
            if (errorFlag == 1)
                warning('At least one point on the trajectory resulted in no feasible solution for the ID');
            end
        end

        % Plots the cost associated with the ID solver (for solvers that
        % aim to minimise some objective cost).
        function plotIDCost(obj)
            figure;
            plot(obj.timeVector, obj.IDFunctionCost, 'LineWidth', 1.5, 'Color', 'k');
            title('ID function cost');
        end

        % Plots the left and right sides of the EoM to show whether the
        % solution matches the desired EoM.
        function verifyEoMConstraint(obj)
            assert(~isempty(obj.trajectory), 'Cannot verify the EoM since trajectory is empty');
            assert(~isempty(obj.cableForces), 'Cannot verify the EoM since trajectory is empty');

            w_left = zeros(obj.model.numDofs, length(obj.timeVector));
            w_right = zeros(obj.model.numDofs, length(obj.timeVector));

            for t = 1:length(obj.timeVector)
                obj.model.update(obj.trajectory.q{t}, obj.trajectory.q_dot{t}, obj.trajectory.q_ddot{t});
                w_left(:,t) = obj.model.M*obj.model.q_ddot + obj.model.C + obj.model.G;
                w_right(:,t) = -obj.model.L'*obj.cableForces{t};
            end

            figure; plot(obj.timeVector, w_left, 'LineWidth', 1.5, 'Color', 'k'); title('ID verify w left hand side');
            figure; plot(obj.timeVector, w_right, 'LineWidth', 1.5, 'Color', 'k'); title('ID verify w right hand side');
        end
    end
end