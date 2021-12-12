% Base class for different simulators that deal with the study of motion
% for CDPRs.
%
% Author        : Darwin LAU
% Created       : 2013
% Description    :
%   Motion simulators are essentially simulators that deal with
%   trajectories, such as IK, FK, ID, FD and control. The simulator
%   provides a lot of plotting functionality to make the plotting of the
%   results more convenient. The plotting methods that are available
%   include:
%       - plotMovie: to plot an avi animation of the CDPR motion
%       - plotCableLengths: plot the length of a set (or all) of the cables
%       - plotJointPose: plot the joint space variables
%       - plotBodyCOG: plot the COG position of the CDPR in inertial frame
%       - plotAngularAcceleration: plot the angular acceleration in {0}
%       - plotFrame: plots a single frame of the CDPR, can be used to plot
%       a particular pose or used within the plotMovie
classdef (Abstract) MotionSimulatorBase < SimulatorBase

    properties
        timeVector          % time vector (needed for forward problems since there is no trajectory)
        trajectory          % Trajectory object for inverse problems only (joint space)
        trajectory_op      	% Trajectory object for inverse problems only (operational space)
        cableLengths        % cell array of cable lengths
        cableLengthsDot     % cell array of cable lengths dot            
    end
    
    properties (Constant)
        MOVIE_DEFAULT_WIDTH = 700;
        MOVIE_DEFAULT_HEIGHT = 700;
    end   

    methods
        % The motion simulator constructor
        function ms = MotionSimulatorBase(model)
            ms@SimulatorBase(model);
        end
    end

    % Set of plotting methods
    methods
        % Plots an avi movie file of the trajector motion of the robot.
        % Uses the static function implementation
        function plotMovie(obj, plot_axis, view_angle, filename, time, isHistory, width, height)            
            MotionSimulatorBase.PlotMovie(obj.model, obj.trajectory, plot_axis, view_angle, filename, time, isHistory, width, height);
        end

        % Plots the cable lengths of the CDPR over the trajectory. Users
        % need to specify the plot axis and also which cables (as an array
        % of numbers) to plot (it is possible to default to plot all cables
        % if the array is []).
        function plotCableLengths(obj, cables_to_plot, plot_ax)
            CASPR_log.Assert(~isempty(obj.cableLengths), 'Cannot plot since lengths vector is empty');
            if nargin < 2 || isempty(cables_to_plot)
                cables_to_plot = 1:obj.model.numCables;
            end
            length_array = cell2mat(obj.cableLengths);
            if(nargin < 3 || isempty(plot_ax))
                figure;
                plot(obj.timeVector, length_array(cables_to_plot, :), 'LineWidth', 1.5, 'Color', 'k');
                title('Cable Lengths');
            else
                plot(plot_ax,obj.timeVector, length_array(cables_to_plot, :), 'LineWidth', 1.5, 'Color', 'k'); 
            end
            xlabel('Time (s)');
            ylabel('Length (m)');
        end
        
        % Plots the cable lengths derivative of the CDPR over the trajectory. Users
        % need to specify the plot axis and also which cables (as an array
        % of numbers) to plot (it is possible to default to plot all cables
        % if the array is []).
        function plotCableLengthsVelocity(obj, cables_to_plot, plot_ax)
            CASPR_log.Assert(~isempty(obj.cableLengthsDot), 'Cannot plot since lengths_dot vector is empty');
            if nargin < 2 || isempty(cables_to_plot)
                cables_to_plot = 1:obj.model.numCables;
            end
            length_dot_array = cell2mat(obj.cableLengthsDot);
            if(nargin < 3 || isempty(plot_ax))
                figure;
                plot(obj.timeVector, length_dot_array(cables_to_plot, :), 'LineWidth', 1.5, 'Color', 'k');
                title('Cable Lengths Velocity');
            else
                plot(plot_ax,obj.timeVector, length_dot_array(cables_to_plot, :), 'LineWidth', 1.5, 'Color', 'k'); 
            end
            xlabel('Time (s)');
            ylabel('Velocity (m/s)');
            
            % ONLY USED IN DEBUGGING START
%             length_array = cell2mat(obj.cableLengths);
%             lengths_dot_num = zeros(obj.model.numCables, length(obj.timeVector));
%             for t = 2:length(obj.timeVector)
%                 lengths_dot_num(:, t) = (length_array(:, t) - length_array(:, t-1))/(obj.timeVector(t) - obj.timeVector(t-1));
%             end
%             figure;
%             plot(obj.timeVector, lengths_dot_num(cables_to_plot, :), 'LineWidth', 1.5, 'Color', 'k');
%             title('Cable Lengths Derivative (Numerical)');
            % ONLY USED IN DEBUGGING END
        end
        
        % Plots the joint space (pose, velocity and acceleration) of the
        % CDPR over the trajectory.
        function plotJointSpace(obj, states_to_plot, plot_ax)
            CASPR_log.Assert(~isempty(obj.trajectory), 'Cannot plot since trajectory is empty');
            if nargin < 2
                states_to_plot = [];
            end
            if nargin < 3
                plot_ax = [];
            end
            obj.trajectory.plotJointSpace(states_to_plot, plot_ax);
        end

        % Plots the joint pose of the CDPR over the trajectory. Users
        % need to specify the plot axis and also which states (as an array
        % of numbers) to plot (it is possible to default to plot all states
        % if the array is []).
        function plotJointPose(obj, states_to_plot, plot_ax)
            CASPR_log.Assert(~isempty(obj.trajectory), 'Cannot plot since trajectory is empty');
            if nargin < 2
                states_to_plot = [];
            end
            if nargin < 3
                plot_ax = [];
            end
            obj.trajectory.plotJointPose(states_to_plot, plot_ax);       
        end
        
        % Plots the joint velocity of the CDPR over the trajectory. Users
        % need to specify the plot axis and also which states (as an array
        % of numbers) to plot (it is possible to default to plot all states
        % if the array is []).
        function plotJointVelocity(obj, states_to_plot, plot_ax)
            CASPR_log.Assert(~isempty(obj.trajectory), 'Cannot plot since trajectory is empty');      
            if nargin < 2
                states_to_plot = [];
            end
            if nargin < 3
                plot_ax = [];
            end   
            obj.trajectory.plotJointVelocity(states_to_plot, plot_ax);          
        end
        
        % Plots the joint acceleration of the CDPR over the trajectory. Users
        % need to specify the plot axis and also which states (as an array
        % of numbers) to plot (it is possible to default to plot all states
        % if the array is []).
        function plotJointAcceleration(obj, states_to_plot, plot_ax)
            CASPR_log.Assert(~isempty(obj.trajectory), 'Cannot plot since trajectory is empty');        
            if nargin < 2
                states_to_plot = [];
            end
            if nargin < 3
                plot_ax = [];
            end    
            obj.trajectory.plotJointAcceleration(states_to_plot, plot_ax);
        end

        % Plots the CoG of the links of the CDPR over the trajectory. Users
        % need to specify the plot axis and also which bodies (as an array
        % of numbers) to plot (it is possible to default to plot all bodies
        % if the array is []).
        function plotBodyCoG(obj, bodies_to_plot, plot_ax)
            CASPR_log.Assert(~isempty(obj.trajectory), 'Cannot plot since trajectory is empty');
            % Plots absolute position, velocity and acceleration of COG
            % with respect to {0}
            if nargin < 2 || isempty(bodies_to_plot)
                bodies_to_plot = 1:obj.model.numLinks;
            end
            pos0 = zeros(3*length(bodies_to_plot), length(obj.timeVector));
            for t = 1:length(obj.timeVector)
                obj.model.bodyModel.update(obj.trajectory.q{t}, obj.trajectory.q_dot{t}, obj.trajectory.q_ddot{t},zeros(size(obj.trajectory.q_dot{t})));
                for ki = 1:length(bodies_to_plot)
                    pos0(3*ki-2:3*ki, t) = obj.model.bodyModel.bodies{bodies_to_plot(ki)}.R_0k*obj.model.bodyModel.bodies{bodies_to_plot(ki)}.r_OG;
                end
            end
            if(nargin < 3 ||isempty(plot_ax))
                figure;
                plot(obj.timeVector, pos0, 'Color', 'k', 'LineWidth', 1.5);
                title('Position of CoG');
            else
                plot(plot_ax,obj.timeVector, pos0, 'LineWidth', 1.5, 'Color', 'k'); 
            end
            xlabel('Time (s)');
            ylabel('Position (m)');            
        end       
        
        % Plots the CoG of the links of the CDPR over the trajectory. Users
        % need to specify the plot axis and also which bodies (as an array
        % of numbers) to plot (it is possible to default to plot all bodies
        % if the array is []).
        function plotBodyCoGVelocity(obj, bodies_to_plot, plot_ax)
            CASPR_log.Assert(~isempty(obj.trajectory), 'Cannot plot since trajectory is empty');
            % Plots absolute position, velocity and acceleration of COG
            % with respect to {0}
            if nargin < 2 || isempty(bodies_to_plot)
                bodies_to_plot = 1:obj.model.numLinks;
            end
            pos0_dot = zeros(3*length(bodies_to_plot), length(obj.timeVector));
            for t = 1:length(obj.timeVector)
                obj.model.bodyModel.update(obj.trajectory.q{t}, obj.trajectory.q_dot{t}, obj.trajectory.q_ddot{t},zeros(size(obj.trajectory.q_dot{t})));
                for ki = 1:length(bodies_to_plot)
                    pos0_dot(3*ki-2:3*ki, t) = obj.model.bodyModel.bodies{bodies_to_plot(ki)}.R_0k*obj.model.bodyModel.bodies{bodies_to_plot(ki)}.v_OG;
                end
            end
            if(nargin < 3 ||isempty(plot_ax))
                figure;
                plot(obj.timeVector, pos0_dot, 'Color', 'k', 'LineWidth', 1.5);
                title('Velocity of CoG');
            else 
                plot(plot_ax,obj.timeVector, pos0_dot, 'LineWidth', 1.5, 'Color', 'k'); 
            end
            xlabel('Time (s)');
            ylabel('Velocity (m/s)');
            % ONLY USED IN DEBUGGING START
            % Numerical derivative must be performed in frame {0}
%             pos0_dot_num = zeros(size(pos0));
%             for t = 2:length(obj.timeVector)
%                 pos0_dot_num(:, t) = (pos0(:, t) - pos0(:, t-1))/(obj.timeVector(t)-obj.timeVector(t-1));
%             end
%             figure;
%             plot(obj.timeVector, pos0_dot_num, 'Color', 'k', 'LineWidth', 1.5);
%             title('Velocity of CoG (numerical)');
            % ONLY USED IN DEBUGGING END
        end
                
        % Plots the CoG of the links of the CDPR over the trajectory. Users
        % need to specify the plot axis and also which bodies (as an array
        % of numbers) to plot (it is possible to default to plot all bodies
        % if the array is []).
        function plotBodyCoGAcceleration(obj, bodies_to_plot, plot_ax)
            CASPR_log.Assert(~isempty(obj.trajectory), 'Cannot plot since trajectory is empty');
            % Plots absolute position, velocity and acceleration of COG
            % with respect to {0}
            if nargin < 2 || isempty(bodies_to_plot)
                bodies_to_plot = 1:obj.model.numLinks;
            end
            pos0_ddot = zeros(3*length(bodies_to_plot), length(obj.timeVector));
            for t = 1:length(obj.timeVector)
                obj.model.bodyModel.update(obj.trajectory.q{t}, obj.trajectory.q_dot{t}, obj.trajectory.q_ddot{t},zeros(size(obj.trajectory.q_dot{t})));
                for ki = 1:length(bodies_to_plot)
                    pos0_ddot(3*ki-2:3*ki, t) = obj.model.bodyModel.bodies{bodies_to_plot(ki)}.R_0k*obj.model.bodyModel.bodies{bodies_to_plot(ki)}.a_OG;
                end
            end
            if(nargin < 3 || isempty(plot_ax))
                figure;
                plot(obj.timeVector, pos0_ddot, 'Color', 'k', 'LineWidth', 1.5);
                title('Acceleration of CoG');
            else
                plot(plot_ax,obj.timeVector, pos0_ddot, 'LineWidth', 1.5, 'Color', 'k'); 
            end
            xlabel('Time (s)');
            ylabel('Acceleration (m/s^2)');            
            % ONLY USED IN DEBUGGING START
            % Numerical derivative must be performed in frame {0}
%             pos0_ddot_num = zeros(size(pos0));
%             for t = 2:length(obj.timeVector)
%                 pos0_ddot_num(:, t) = (pos0_dot_num(:, t) - pos0_dot_num(:, t-1))/(obj.timeVector(t)-obj.timeVector(t-1));
%             end
%             figure;
%             plot(obj.timeVector, pos0_ddot_num, 'Color', 'k', 'LineWidth', 1.5);
%             title('Acceleration of CoG (numerical)');
            % ONLY USED IN DEBUGGING END
        end
                
        % Plots the angular acceleration of the links of the CDPR. Users
        % need to specify the plot axis and also which bodies (as an array
        % of numbers) to plot (it is possible to default to plot all bodies
        % if the array is []).
        function plotAngularVelocity(obj, bodies_to_plot, plot_ax)
            CASPR_log.Assert(~isempty(obj.trajectory), 'Cannot plot since trajectory is empty');
            % Plots absolute position, velocity and acceleration of COG
            % with respect to {0}
            if nargin < 2 || isempty(bodies_to_plot)
                bodies_to_plot = 1:obj.model.numLinks;
            end
            ang0 = zeros(3*length(bodies_to_plot), length(obj.timeVector));
            for t = 1:length(obj.timeVector)
                obj.model.bodyModel.update(obj.trajectory.q{t}, obj.trajectory.q_dot{t}, obj.trajectory.q_ddot{t},zeros(size(obj.trajectory.q_ddot{t})));
                for ki = 1:length(bodies_to_plot)
                    ang0(3*ki-2:3*ki, t) = obj.model.bodyModel.bodies{bodies_to_plot(ki)}.R_0k*obj.model.bodyModel.bodies{bodies_to_plot(ki)}.w;
                end
            end
            if(nargin < 3 || isempty(plot_ax))
                figure;
                plot(obj.timeVector, ang0, 'Color', 'k', 'LineWidth', 1.5);
                title('Angular velocity of rigid bodies');          
            else
                plot(plot_ax,obj.timeVector, ang0, 'LineWidth', 1.5, 'Color', 'k'); 
            end
            xlabel('Time (seconds)');
            ylabel('Angular Velocity (rad/s)');            
        end
        
        % Plots the angular acceleration of the links of the CDPR. Users
        % need to specify the plot axis and also which bodies (as an array
        % of numbers) to plot (it is possible to default to plot all bodies
        % if the array is []).
        function plotAngularAcceleration(obj, bodies_to_plot, plot_ax)
            CASPR_log.Assert(~isempty(obj.trajectory), 'Cannot plot since trajectory is empty');
            % Plots absolute position, velocity and acceleration of COG
            % with respect to {0}
            if nargin < 2 || isempty(bodies_to_plot)
                bodies_to_plot = 1:obj.model.numLinks;
            end
            ang0_dot = zeros(3*length(bodies_to_plot), length(obj.timeVector));
            for t = 1:length(obj.timeVector)
                obj.model.bodyModel.update(obj.trajectory.q{t}, obj.trajectory.q_dot{t}, obj.trajectory.q_ddot{t},zeros(size(obj.trajectory.q_ddot{t})));
                for ki = 1:length(bodies_to_plot)
                    ang0_dot(3*ki-2:3*ki, t) = obj.model.bodyModel.bodies{bodies_to_plot(ki)}.R_0k*obj.model.bodyModel.bodies{bodies_to_plot(ki)}.w_dot;
                end
            end
            if (nargin < 3 || isempty(plot_ax))
                figure;
                plot(obj.timeVector, ang0_dot, 'Color', 'k', 'LineWidth', 1.5);
                title('Angular acceleration dot of rigid bodies');                
            else
                plot(plot_ax,obj.timeVector, ang0_dot, 'LineWidth', 1.5, 'Color', 'k'); 
            end
            xlabel('Time (seconds)');
            ylabel('Angular Acceleration (rad/s^3)');            
            % ONLY USED IN DEBUGGING START
%             % Numerical derivative must be performed in frame {0}
%             ang0_dot_num = zeros(size(ang0));
%             for t = 2:length(obj.timeVector)
%                 ang0_dot_num(:, t) = (ang0(:, t) - ang0(:, t-1))/(obj.timeVector(t)-obj.timeVector(t-1));
%             end
%             figure;
%             plot(obj.timeVector, ang0_dot_num, 'Color', 'k');
%             title('Aangular acceleration of rigid bodies (numerical)');
            % ONLY USED IN DEBUGGING START
        end
    end
    
    methods (Static)
        % Plots a single image of the CDPR at the specified kinematics.
        % Users can specify the plotting axis and also which figure handle
        % to plot too. If no or empty figure handle is specified then a new
        % figure plot will be created.
        function [op_point] = PlotFrame(kinematics, plot_axis, view_angle, fig_handle, axis_handle)
            if nargin < 4 || isempty(fig_handle)
                figure;
            else
                figure(fig_handle);
            end
            if nargin < 5 || isempty(axis_handle)
                clf;
                ax = gca;
            else
                ax = axis_handle;
                axes(ax);
            end
            
            axis(plot_axis);
            view(view_angle); 
            hold on;
            grid on;
            xlabel('x');
            ylabel('y');
            zlabel('z');
          
            body_model = kinematics.bodyModel;
            for k = 1:body_model.numLinks
                r_OP0 = body_model.bodies{k}.R_0k*body_model.bodies{k}.r_OP;
                r_OG0 = body_model.bodies{k}.R_0k*body_model.bodies{k}.r_OG;
                r_OPe0 = body_model.bodies{k}.R_0k*body_model.bodies{k}.r_OPe;
                plot3(ax,r_OP0(1), r_OP0(2), r_OP0(3), 'Color', 'k', 'Marker', 'o', 'LineWidth', 2);
                plot3(ax,r_OG0(1), r_OG0(2), r_OG0(3), 'Color', 'b', 'Marker', 'o', 'LineWidth', 2);
                plot3(ax,[r_OP0(1) r_OPe0(1)], [r_OP0(2) r_OPe0(2)], [r_OP0(3) r_OPe0(3)], 'Color', 'k', 'LineWidth', 3);
            end
            
            op_point = zeros(body_model.numOperationalSpaces, 3);
            for k = 1:body_model.numOperationalSpaces
                % TODO: check types of operational spaces and perform accordingly, let's just
                % assume now it is only cartesian
                %if (body_model.bodies{operationalSpaceBodyIndices(k)}
                r_OY0 = body_model.bodies{body_model.operationalSpaceBodyIndices(k)}.R_0k*body_model.bodies{body_model.operationalSpaceBodyIndices(k)}.r_OY;
                plot3(ax, r_OY0(1), r_OY0(2), r_OY0(3), 'Color', 'g', 'Marker', 'o', 'LineWidth', 2); 
                op_point(k,:) = r_OY0;
            end

            cable_model = kinematics.cableModel;
            for i = 1:cable_model.numCables
                cable_model.cables{i}.plotCable(ax);
            end
            hold off;            
        end
        
        % Plots the history of operational space 
        function PlotHistory(operationalHistory, fig_handle, axis_handle)
            if nargin < 2 || isempty(fig_handle)
                figure;
            else
                figure(fig_handle);
            end
            
            if nargin < 3 || isempty(axis_handle)
                ax = gca;
            else
                ax = axis_handle;
            end
            axes(ax);
            hold on;
            % Plot History   
            for i = 1:size(operationalHistory,1)
                history_array = cell2mat(operationalHistory(i,:));
                plot3(ax, history_array(1,:), history_array(2,:), history_array(3,:),...
                    'Color', [0.5 0.5 0.5], 'LineWidth', 2, 'LineStyle', '--'); 
            end            
            hold off;
        end
        
        % Plots a avi movie for the CDPR joint space trajectory specified. 
        % The plot axis, filename, width and height of the movie file must be
        % provided. The "time" variable specifies the total time of the
        % video. The video will be recorded at 30 fps so the number of
        % frames will be computed to confirm to the total time of the
        % video.
        function PlotMovie(modelObj, trajectory, plot_axis, view_angle, filename, time, isHistory, width, height)
            CASPR_log.Assert(~isempty(trajectory), 'Cannot plot since trajectory is empty');
            if (nargin < 6)
                % Default time to be time of trajectory (no speed up or
                % slow down)
                time = trajectory.totalTime;
            end
            if (nargin < 7)
                isHistory = false;
            end
            if (nargin < 8)
                width = MotionSimulatorBase.MOVIE_DEFAULT_WIDTH;
                height = MotionSimulatorBase.MOVIE_DEFAULT_HEIGHT;
            end
            fps = 30;
            writerObj = VideoWriter(filename);
            %writerObj.Quality = 100;
            writerObj.open();
            plot_handle = figure('Position', [10, 10, width, height]);
            
            % Create a cell array to hold the operational space history
            operationalHistory = cell(modelObj.bodyModel.numOperationalSpaces, 0);
            
            % Use default mode to update individual bodies
            % TODO: RE-CHECK THAT THIS IS REQUIRED TO USE DEFAULT MODE 
            % (COMPARED WITH JUST USING COMPILED MODE OR SYMBOLICS)
            model_config = ModelConfig(modelObj.robotName);
            model_default_mode = model_config.getModel(modelObj.cableSetName, modelObj.operationalSpaceName);
            
            for i = 1:round(fps*time)
                t = round(length(trajectory.timeVector)/round(fps*time)*i);
                if t == 0
                    t = 1;
                end
                model_default_mode.update(trajectory.q{t}, trajectory.q_dot{t}, trajectory.q_ddot{t}, zeros(size(trajectory.q_dot{t})));
                op_point = MotionSimulatorBase.PlotFrame(model_default_mode, plot_axis, view_angle, plot_handle); 
                if isHistory
                    for j=1:size(op_point,1)
                        operationalHistory{j, end+1} = op_point(j, :)';
                    end
                    MotionSimulatorBase.PlotHistory(operationalHistory, plot_handle);
                end
                frame = getframe(plot_handle);
                writerObj.writeVideo(frame);
            end
            writerObj.close();
            close(plot_handle);
        end
        
        % Plots a model running a joint trajectory in Rviz through the
        % CASPR-RViz interface. The model and the joint trajectory have to
        % be provided, together with the ROS_MASTER_URI and local ROS_IP
        % for the CASPR-RViz interface.
        % As an optional argument, a trajectory of cable forces can also be
        % used to visualize cable forces in RViz as arrows
        function PlotRviz(modelObj, joint_trajectory, force_trajectory)
            % Assert the size of force_trajectory
            if nargin > 2
                CASPR_log.Assert(size(joint_trajectory.timeVector,2)==size(force_trajectory, 2), ...
                    'Size of joint_trajectory and force trajectory do not match.');
            end
            % Create CASPR-RViz Interface Object
            rviz_in = CASPRRVizInterface();
            % Set robot name rosparam            
            rosparam('set','/robot_name',modelObj.robotName);
            rosparam('set','/deleteall', true);            
            pause(1);
            % Use the period in trajectory
            period = (joint_trajectory.timeVector(2) - joint_trajectory.timeVector(1));
            % Plot initial pose and wait
            modelObj.update(joint_trajectory.q{1}, joint_trajectory.q_dot{1}, joint_trajectory.q_ddot{1}, ...
                zeros(modelObj.numDofs,1));
            rviz_in.visualize(modelObj);
            pause(1);
            for t = 1:length(joint_trajectory.timeVector)
                freq_tic = tic;
                q = joint_trajectory.q{t};
                q_d = joint_trajectory.q_dot{t};                
                % Update cdpr
                modelObj.update(q, q_d, zeros(modelObj.numDofs,1), zeros(modelObj.numDofs,1));
                % Print info without affecting the frequency regulation
                if t ~= 1
                    CASPR_log.Info(sprintf('t: %d Freq: %.2f', t, 1/elapsed));
                end
                % Send to RViz
                if nargin > 2
                    % Visualize also force                    
                    rviz_in.visualize(modelObj, force_trajectory{t});
                else
                    % Only visualize kinematics
                    rviz_in.visualize(modelObj);
                end
                
                % Wait until meeting the period 
                elapsed = toc(freq_tic);
                while elapsed < period
                    elapsed = toc(freq_tic);            
                end 
            end
        end
        
        % Plots a model running a joint trajectory in Rviz through the
        % CARDSFlow interface. The model and the joint trajectory have to
        % be provided, together with the ROS_MASTER_URI and local ROS_IP
        % for the CARDSFlow interface.
        function PlotCARDSFlow(modelObj, trajectory)
            % Create CARDSFlow Interface Object
            card_in = CARDSFlowInterface();
            % Set robot name rosparam
            rosparam('set','/robot_name',modelObj.robotName);            
            % Ensure the model mode is default            
            modelObj.setModelMode(ModelModeType.DEFAULT);
            % Use the period in trajectory
            period = (trajectory.timeVector(2) - trajectory.timeVector(1));
            % Loop through the trajectory
            for t = 1:length(trajectory.timeVector)
                freq_tic = tic;
                q = trajectory.q{t};
                q_d = trajectory.q_dot{t};                
                % Update cdpr
                modelObj.update(q, q_d, zeros(modelObj.numDofs,1), zeros(modelObj.numDofs,1));
                % Print info without affecting the frequency regulation
                if t ~= 1
                    CASPR_log.Info(sprintf('t: %d Freq: %.2f', t, 1/elapsed));
                end
                % Send to CARDSFlow
                card_in.visualize(modelObj);
                % Wait until meeting the period 
                elapsed = toc(freq_tic);
                while elapsed < period
                    elapsed = toc(freq_tic);            
                end 
            end
        end
    end
    
    % Set of GUI plotting functions
    methods
        function guiPlotCableLengths(obj, plot_ax)
            obj.plotCableLengths([], plot_ax);
        end        
        function guiPlotCableLengthsVelocity(obj, plot_ax)
            obj.plotCableLengthsVelocity([], plot_ax);
        end        
        function guiPlotJointPose(obj, plot_ax)
            obj.plotJointPose([], plot_ax);
        end        
        function guiPlotJointVelocity(obj, plot_ax)
            obj.plotJointVelocity([], plot_ax);
        end        
        function guiPlotJointAcceleration(obj, plot_ax)
            obj.plotJointAcceleration([], plot_ax);
        end        
        function guiPlotBodyCoG(obj, plot_ax)
            obj.plotBodyCoG([], plot_ax);
        end        
        function guiPlotBodyCoGVelocity(obj, plot_ax)
            obj.plotBodyCoGVelocity([], plot_ax);
        end        
        function guiPlotBodyCoGAcceleration(obj, plot_ax)
            obj.plotBodyCoGAcceleration([], plot_ax);
        end        
        function guiPlotAngularVelocity(obj, plot_ax)
            obj.plotAngularVelocity([], plot_ax);
        end        
        function guiPlotAngularAcceleration(obj, plot_ax)
            obj.plotAngularAcceleration([], plot_ax);
        end
    end
end
