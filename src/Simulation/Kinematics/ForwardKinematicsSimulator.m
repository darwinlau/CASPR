classdef ForwardKinematicsSimulator < MotionSimulator
    %InverseDynamicsSimulation Simulation for Inverse Dynamics (cable force
    %resolution)
    
    properties (SetAccess = protected) 
        FKSolver
    end
    
    methods
        function fk = ForwardKinematicsSimulator(model, fk_solver)
            fk@MotionSimulator(model);
            fk.FKSolver = fk_solver;
        end
        
        function run(obj, lengths, lengths_dot, time_vector, q0_approx, q0_prev_approx)
            obj.timeVector = time_vector;
            obj.lengths = lengths;
            obj.lengths_dot = lengths_dot;
            
            obj.trajectory = JointTrajectory;
            obj.trajectory.timeVector = obj.timeVector;
            obj.trajectory.q = cell(1, length(obj.timeVector));
            obj.trajectory.q_dot = cell(1, length(obj.timeVector));
            % Does not compute q_ddot (set it to be empty)
            obj.trajectory.q_ddot = cell(1, length(obj.timeVector));
            obj.trajectory.q_ddot(:) = {zeros(size(q0_approx))};
            
            % Runs the simulation over the specified trajectory
            q_prev = q0_approx;
            q_d_prev = q0_prev_approx;
            lengths_prev = lengths{1};
            lengths_prev_2 = lengths{1};
            
            time_prev = 0;
            
            for t = 1:length(obj.trajectory.timeVector)
                fprintf('Time : %f\n', obj.trajectory.timeVector(t));
                [q, q_dot] = obj.FKSolver.compute(lengths{t}, lengths_prev_2, q_prev, q_d_prev, obj.trajectory.timeVector(t) - time_prev, obj.model);
                
                obj.trajectory.q{t} = q;
                obj.trajectory.q_dot{t} = q_dot;
                
                q_prev = q;
                q_d_prev = q_dot;
                time_prev =  obj.trajectory.timeVector(t);
                lengths_prev_2 = lengths_prev;
                lengths_prev = lengths{t};
            end
        end
    end
end

