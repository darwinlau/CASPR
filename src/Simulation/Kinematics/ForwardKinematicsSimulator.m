classdef ForwardKinematicsSimulator < MotionSimulator
    %InverseDynamicsSimulation Simulation for Inverse Dynamics (cable force
    %resolution)
    
    properties (SetAccess = protected)   
    end
    
    methods
        function fk = ForwardKinematicsSimulator(model)
            fk@MotionSimulator(model);
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
            q_prev_dot = q0_prev_approx;
            l_dot = zeros(size(lengths{1}));
            l_dot_prev = zeros(size(lengths{1}));
            time_prev = 0;
            
            for t = 1:length(obj.trajectory.timeVector)
                fprintf('Time : %f\n', obj.trajectory.timeVector(t));
                
%                 % Compute length_dot_prev
                if t > 1
                    l_dot = (lengths{t} - lengths{t-1})/(obj.timeVector(t) - obj.timeVector(t-1));
                end
                
                % Compute length_dot_prev
                if t > 2
                    l_dot_prev = (lengths{t} - lengths{t-2})/(obj.timeVector(t) - obj.timeVector(t-2));
                end
                
                %[q, q_dot] = ForwardKinematics.ComputeGeneralisedCoordinates1(obj.model, lengths{t}, l_dot, q_prev, q_prev_dot, obj.trajectory.timeVector(t) - time_prev);
                %[q, q_dot] = ForwardKinematics.ComputeGeneralisedCoordinates2(obj.model, lengths{t}, l_dot_prev, q_prev, obj.trajectory.timeVector(t) - time_prev);
                [q, q_dot] = ForwardKinematics.ComputeGeneralisedCoordinates3(obj.model, lengths{t}, q_prev, q_prev_dot, obj.trajectory.timeVector(t) - time_prev);
                %[q, q_dot] = ForwardKinematics.ComputeGeneralisedCoordinates4(obj.model, lengths{t}, q_prev, obj.trajectory.timeVector(t) - time_prev);
                %[q, q_dot] = ForwardKinematics.ComputeGeneralisedCoordinates5(obj.model, l_dot_prev, q_prev, obj.trajectory.timeVector(t) - time_prev);
                
                obj.trajectory.q{t} = q;
                obj.trajectory.q_dot{t} = q_dot;
                
                q_prev = q;
                q_prev_dot = q_dot;
                time_prev =  obj.trajectory.timeVector(t);
            end
        end
    end
end

