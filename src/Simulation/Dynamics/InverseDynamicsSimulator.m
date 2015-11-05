classdef InverseDynamicsSimulator < DynamicsSimulator
    %InverseDynamicsSimulation Simulation for Inverse Dynamics (cable force
    %resolution)
    
    properties (SetAccess = protected)
        compTime            % computational time for each time step
        compIterations      % Number of computational iterations for each time step
                
        IDFunctionCost      % Cost value for optimisation at each point in time
        IDExitType          % Exit type at each point in time
        IDInfo              % ID optimiser information at each point in time
        
        IDSolver
    end
    
    methods
        function id = InverseDynamicsSimulator(model, id_solver)
            id@DynamicsSimulator(model);
            id.IDSolver = id_solver;
        end
        
        function run(obj, trajectory)
            obj.trajectory = trajectory;
            obj.timeVector = obj.trajectory.timeVector;
            % Runs the simulation over the specified trajectory
            obj.cableForces = cell(1, length(obj.trajectory.timeVector));
            
            obj.IDFunctionCost = zeros(length(obj.timeVector), 1);
            obj.IDExitType = cell(length(obj.timeVector), 1);
            obj.IDInfo = cell(length(obj.timeVector), 1);
            
            obj.compTime = zeros(length(obj.timeVector), 1);
            obj.compIterations = zeros(length(obj.timeVector), 1);
            
            for t = 1:length(obj.timeVector)
                fprintf('Time : %f\n', obj.timeVector(t));
                obj.model.update(obj.trajectory.q{t}, obj.trajectory.q_dot{t}, obj.trajectory.q_ddot{t});        
                [obj.IDFunctionCost(t), obj.IDExitType{t}, obj.IDInfo{t}] = obj.IDSolver.resolve(obj.model);
                obj.interactionWrench{t} = obj.model.interactionWrench;
                obj.cableForces{t} = obj.model.cableForces;            
                obj.lengths{t} = obj.model.cableLengths;
                obj.lengths_dot{t} = obj.model.cableLengthsDot;
                obj.compTime(t) = obj.IDInfo{t}.Time;
                obj.compIterations(t) = obj.IDInfo{t}.Iterations;
            end
        end
        
        function plotIDCost(obj)            
            figure; 
            plot(obj.timeVector, obj.IDFunctionCost, 'LineWidth', 1.5, 'Color', 'k'); 
            title('ID function cost'); 
        end
        
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

