classdef (Abstract) MotionSimulator < Simulator
    %DYNAMICSSIMULATION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties     
        timeVector          % time vector
    end
    
    methods
        function ms = MotionSimulator(kinObj)
            ms@Simulator(kinObj);
        end
        
        function plotMovie(obj, joint_trajectory, plot_axis, filename, time, width, height)
            fps = 30;
            
            writerObj = VideoWriter(filename);
            %writerObj.Quality = 100;
            writerObj.open();
            
            plot_handle = figure('Position', [10, 10, width, height]);
            for i = 1:round(fps*time)
                t = round(length(obj.timeVector)/round(fps*time)*i);
                if t == 0
                    t = 1;
                end
                obj.kinematicsObj.update(joint_trajectory.q{t}, joint_trajectory.q_dot{t}, joint_trajectory.q_ddot{t});
                MotionSimulator.PlotFrame(obj.kinematicsObj, plot_axis, plot_handle)
                frame = getframe(plot_handle);
                writerObj.writeVideo(frame);
                clf(plot_handle);
            end
            
            writerObj.close();
            close(plot_handle);
        end
        
        function plotCableLengths(obj, length_trajectory, length_dot_trajectory, cables_to_plot)
            if nargin == 3 || isempty(cables_to_plot)
                cables_to_plot = 1:obj.kinematicsObj.numCables;
            end
            length_array = cell2mat(length_trajectory);
            length_dot_array = cell2mat(length_dot_trajectory);
            
            figure; plot(obj.timeVector, length_array(cables_to_plot, :), 'LineWidth', 1.5, 'Color', 'k'); title('Cable Lengths'); 
            figure; plot(obj.timeVector, length_dot_array(cables_to_plot, :), 'LineWidth', 1.5, 'Color', 'k'); title('Cable Lengths Derivative'); 
            
            % ONLY USED IN DEBUGGING START
%             lengths_dot_num = zeros(obj.states{1}.numCables, length(obj.timeVector));
%             for t = 2:length(obj.timeVector)
%                 lengths_dot_num(:, t) = (lengths(:, t) - lengths(:, t-1))/(obj.timeVector(t) - obj.timeVector(t-1));
%             end
%             figure; plot(obj.timeVector, lengths_dot_num(cables_to_plot, :), 'LineWidth', 1.5, 'Color', 'k');       
            % ONLY USED IN DEBUGGING END   
        end
        
        function plotJointSpace(obj, joint_trajectory, states_to_plot)            
            n_dof = obj.kinematicsObj.numDofs;
            
            if nargin == 2 || isempty(states_to_plot)
                states_to_plot = 1:n_dof;
            end
            
            q_array = cell2mat(joint_trajectory.q);
            q_dot_array = cell2mat(joint_trajectory.q_dot);
            
            % Plots joint space variables q(t)
            figure;
            title('Joint space variables');
            plot(obj.timeVector, q_array(states_to_plot, :), 'Color', 'k', 'LineWidth', 1.5);
            
            % Plots derivative joint space variables q_dot(t)
            figure;
            title('Joint space derivatives');
            plot(obj.timeVector, q_dot_array(states_to_plot, :), 'Color', 'k', 'LineWidth', 1.5);
        end
        
        function plotBodyCOG(obj, joint_trajectory, bodies_to_plot)            
            % Plots absolute position, velocity and acceleration of COG
            % with respect to {0}
            if nargin == 2 || isempty(bodies_to_plot)
                bodies_to_plot = 1:obj.kinematicsObj.numLinks;
            end
            
            pos0 = zeros(3*length(bodies_to_plot), length(obj.timeVector));
            pos0_dot = zeros(3*length(bodies_to_plot), length(obj.timeVector));
            pos0_ddot = zeros(3*length(bodies_to_plot), length(obj.timeVector));
            for t = 1:length(obj.timeVector)
                obj.kinematicsObj.bodyKinematics.update(joint_trajectory.q{t}, joint_trajectory.q_dot{t}, joint_trajectory.q_ddot{t});
                for ki = 1:length(bodies_to_plot)
                    pos0(3*ki-2:3*ki, t) = obj.kinematicsObj.bodyKinematics.bodies{bodies_to_plot(ki)}.R_0k*obj.kinematicsObj.bodyKinematics.bodies{bodies_to_plot(ki)}.r_OG;
                    pos0_dot(3*ki-2:3*ki, t) = obj.kinematicsObj.bodyKinematics.bodies{bodies_to_plot(ki)}.R_0k*obj.kinematicsObj.bodyKinematics.bodies{bodies_to_plot(ki)}.v_OG;
                    pos0_ddot(3*ki-2:3*ki, t) = obj.kinematicsObj.bodyKinematics.bodies{bodies_to_plot(ki)}.R_0k*obj.kinematicsObj.bodyKinematics.bodies{bodies_to_plot(ki)}.a_OG;
                end
            end
            figure; 
            title('Position of CoG');
            plot(obj.timeVector, pos0, 'Color', 'k', 'LineWidth', 1.5);
            
            figure; 
            title('Velocity of CoG');
            plot(obj.timeVector, pos0_dot, 'Color', 'k', 'LineWidth', 1.5);
            
            figure; 
            title('Acceleration of CoG');
            plot(obj.timeVector, pos0_ddot, 'Color', 'k', 'LineWidth', 1.5);
                        
            % ONLY USED IN DEBUGGING START
            % Numerical derivative must be performed in frame {0}
%             pos0_dot_num = zeros(size(pos0));
%             pos0_ddot_num = zeros(size(pos0));
%             for t = 2:length(obj.timeVector)
%                 pos0_dot_num(:, t) = (pos0(:, t) - pos0(:, t-1))/(obj.timeVector(t)-obj.timeVector(t-1));
%                 pos0_ddot_num(:, t) = (pos0_dot_num(:, t) - pos0_dot_num(:, t-1))/(obj.timeVector(t)-obj.timeVector(t-1));
%             end
%             figure;title('Numerical velocity of CoG');
%             plot(obj.timeVector, pos0_dot_num, 'Color', 'k');
%             figure;title('Numerical acceleration of CoG');
%             plot(obj.timeVector, pos0_ddot_num, 'Color', 'k');
            % ONLY USED IN DEBUGGING END
        end
        
        function plotAngularAcceleration(obj, joint_trajectory, bodies_to_plot)
            % Plots absolute position, velocity and acceleration of COG
            % with respect to {0}            
            if nargin == 2 || isempty(bodies_to_plot)
                bodies_to_plot = 1:obj.kinematicsObj.numLinks;
            end
            
            
            ang0 = zeros(3*length(bodies_to_plot), length(obj.timeVector));
            ang0_dot = zeros(3*length(bodies_to_plot), length(obj.timeVector));
            for t = 1:length(obj.timeVector)
                obj.kinematicsObj.bodyKinematics.update(joint_trajectory.q{t}, joint_trajectory.q_dot{t}, joint_trajectory.q_ddot{t});
                for ki = 1:length(bodies_to_plot)
                    ang0(3*ki-2:3*ki, t) = obj.kinematicsObj.bodyKinematics.bodies{bodies_to_plot(ki)}.R_0k*obj.kinematicsObj.bodyKinematics.bodies{bodies_to_plot(ki)}.w;
                    ang0_dot(3*ki-2:3*ki, t) = obj.kinematicsObj.bodyKinematics.bodies{bodies_to_plot(ki)}.R_0k*obj.kinematicsObj.bodyKinematics.bodies{bodies_to_plot(ki)}.w_dot;
                end
            end
            
            figure;
            title('Angular velocity of rigid bodies');
            plot(obj.timeVector, ang0, 'Color', 'k', 'LineWidth', 1.5);
            
            figure;
            title('Angular acceleration of rigid bodies');
            plot(obj.timeVector, ang0_dot, 'Color', 'k', 'LineWidth', 1.5);
            
            % ONLY USED IN DEBUGGING START
%             % Numerical derivative must be performed in frame {0}
%             ang0_dot_num = zeros(size(ang0));
%             for t = 2:length(obj.timeVector)
%                 ang0_dot_num(:, t) = (ang0(:, t) - ang0(:, t-1))/(obj.timeVector(t)-obj.timeVector(t-1));
%             end
%             figure; title('Numerical angular acceleration of rigid bodies');
%             plot(obj.timeVector, ang0_dot_num, 'Color', 'k');
            % ONLY USED IN DEBUGGING START
        end
            
    end
    methods (Static)        
        function PlotFrame(kinematics, plot_axis, fig_handle)
            if nargin < 3
                fig_handle = figure;
            end
            
            figure(fig_handle);
            
            axis(plot_axis);
            axis equal;
            grid on;
            hold on;
            
            xlabel('x');
            ylabel('y');
            
            body_kinematics = kinematics.bodyKinematics;
            for k = 1:body_kinematics.numLinks
                r_OP0 = body_kinematics.bodies{k}.R_0k*body_kinematics.bodies{k}.r_OP;
                r_OG0 = body_kinematics.bodies{k}.R_0k*body_kinematics.bodies{k}.r_OG;
                r_OPe0 = body_kinematics.bodies{k}.R_0k*body_kinematics.bodies{k}.r_OPe;
                plot3(r_OP0(1), r_OP0(2), r_OP0(3), 'Color', 'k', 'Marker', 'o', 'LineWidth', 2);
                plot3(r_OG0(1), r_OG0(2), r_OG0(3), 'Color', 'b', 'Marker', 'o', 'LineWidth', 2);
                line([r_OP0(1) r_OPe0(1)], [r_OP0(2) r_OPe0(2)], [r_OP0(3) r_OPe0(3)], 'Color', 'k', 'LineWidth', 3);
            end
            
            cable_kinematics = kinematics.cableKinematics;
            for i = 1:cable_kinematics.numCables
                for j = 1:cable_kinematics.cables{i}.numSegments
                    for k = 1:cable_kinematics.numLinks+1
                        if cable_kinematics.getCRMTerm(i,j,k) == -1
                            r_OAa0 = cable_kinematics.cables{i}.segments{j}.attachmentsAbs{k};
                            if k > 1
                                r_OAa0 = body_kinematics.bodies{k-1}.R_0k*r_OAa0;
                            end
                        elseif cable_kinematics.getCRMTerm(i,j,k) == 1
                            r_OAb0 = cable_kinematics.cables{i}.segments{j}.attachmentsAbs{k};
                            if k > 1
                                r_OAb0 = body_kinematics.bodies{k-1}.R_0k*r_OAb0;
                            end
                        end
                    end
                    line([r_OAa0(1) r_OAb0(1)], [r_OAa0(2) r_OAb0(2)], [r_OAa0(3) r_OAb0(3)], 'Color', 'r', 'LineWidth', 1);
                end
            end
            
            hold off;
        end
    end    
end

