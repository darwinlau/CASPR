classdef InverseDynamicsSimulator < DynamicsSimulator
    %InverseDynamicsSimulation Simulation for Inverse Dynamics (cable force
    %resolution)
    
    properties (SetAccess = protected)        
        trajectory          % Trajectory object for inverse problems only (input)
        cableForces         % cable force vector output from ID (output)
        
        compTime            % computational time for each time step
        compIterations      % Number of computational iterations for each time step
                
        IDFunctionCost      % Cost value for optimisation at each point in time
        IDExitType          % Exit type at each point in time
        IDInfo              % ID optimiser information at each point in time
        
        IDSolver
    end
    
    methods
        function id = InverseDynamicsSimulator(id_solver)
            id.IDSolver = id_solver;
        end
        
        function run(obj, trajectory, sdConstructor)
            obj.trajectory = trajectory;
            
            obj.timeVector = obj.trajectory.timeVector;
            obj.IDFunctionCost = zeros(length(obj.timeVector), 1);
            obj.IDExitType = cell(length(obj.timeVector), 1);
            obj.IDInfo = cell(length(obj.timeVector), 1);
            
            obj.compTime = zeros(length(obj.timeVector), 1);
            obj.compIterations = zeros(length(obj.timeVector), 1);
            
            % Create SystemDynamics cell array
            obj.states = CellOperations.CreateCellArray(sdConstructor, [1 length(obj.trajectory.timeVector)]);
            for t = 1:length(obj.timeVector)
                fprintf('Time : %f\n', obj.timeVector(t));
                obj.states{t}.update(obj.trajectory.q{t}, obj.trajectory.q_dot{t}, obj.trajectory.q_ddot{t});        
                [obj.IDFunctionCost(t), obj.IDExitType{t}, obj.IDInfo{t}] = obj.IDSolver.resolve(obj.states{t});
                obj.cableForces{t} = obj.states{t}.cableForces;
                obj.compTime(t) = obj.IDInfo{t}.Time;
                obj.compIterations(t) = obj.IDInfo{t}.Iterations;
            end
            
%             dynamics = sdConstructor();
%             for t = 1:length(obj.timeVector)
%                 fprintf('Time : %f\n', obj.timeVector(t));
%                 dynamics.update(obj.trajectory.q{t}, obj.trajectory.q_dot{t}, obj.trajectory.q_ddot{t});        
%                 [obj.IDFunctionCost(t), obj.IDExitType{t}, obj.IDInfo{t}] = obj.IDSolver.resolve(dynamics);
%                 obj.cableForces{t} = dynamics.cableForces;
%                 obj.compTime(t) = obj.IDInfo{t}.Time;
%                 obj.compIterations(t) = obj.IDInfo{t}.Iterations;
%             end
        end
        
        function plotIDCost(obj)            
            figure; plot(obj.timeVector, obj.IDFunctionCost, 'LineWidth', 1.5, 'Color', 'k'); title('ID function cost'); 
        end
        
        
        function verifyEoMConstraint(obj)
            assert(~isempty(obj.states), 'State dynamics cell array is empty and nothing to plot.');
            
            w_left = zeros(obj.states{1}.numDofs, length(obj.timeVector));
            w_right = zeros(obj.states{1}.numDofs, length(obj.timeVector));
            
            for t = 1:length(obj.timeVector)
                w_left(:,t) = obj.states{t}.M*obj.states{t}.q_ddot + obj.states{t}.C + obj.states{t}.G;
                w_right(:,t) = -obj.states{t}.L'*obj.states{t}.cableDynamics.forces;
            end
            
            figure; plot(obj.timeVector, w_left, 'LineWidth', 1.5, 'Color', 'k'); title('ID verify w left hand side'); 
            figure; plot(obj.timeVector, w_right, 'LineWidth', 1.5, 'Color', 'k'); title('ID verify w right hand side'); 
        end
    end
end

