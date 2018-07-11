% Optimisation function for cable attachment optimisation, the objective is
% the sum of ID cost for a specified ID solver and trajectory. If there is
% no valid solution to the ID a default errorCost will be applied.
%
% Please cite the following paper when using this for multilink cable
% robots:
% D. Lau, K. Bhalerao, D. Oetomo, and S. K. Halgamuge, "On the Task
% Specific Evaluation and Optimisation of Cable-Driven Manipulators",
% Advances in Reconfigurable Mechanisms and Robots I : Part 6, pp. 707-716,
% 2012.
%
% Author        : Darwin LAU
% Created       : 2016
% Description	:
classdef CableAttachmentOptimisationMinID < CableAttachmentOptimisationFnBase
    properties
        errorCost
        parameters
        model
    end

    methods
        function caofn = CableAttachmentOptimisationMinID(model, param, errorVal)
            caofn.model = model;
            caofn.errorCost = errorVal;
            caofn.parameters = param;
        end

        % The cost is
        function Q = evaluate(obj, x, idsolver, trajectory)
            Q = 0;

            obj.parameters.updateCableAttachments(x);

            for t = 1:length(trajectory.timeVector)
                % The model is already updated within the resolve function
                [~, ~, Q_t, idExit] = idsolver.resolve(trajectory.q{t}, trajectory.q_dot{t}, trajectory.q_ddot{t}, zeros(obj.model.numDofs,1));

                if idExit == IDSolverExitType.NO_ERROR
                    Q = Q + Q_t;
                else
                    Q = Q + obj.errorCost;
                end
            end
        end
    end
end
