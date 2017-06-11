classdef PoCaBotExperiment < ExperimentBase
    %POCABOTEXPERIMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        l_feedback_traj    % Temporary variable to store things for now
        l_cmd_traj         % Temporary variable to store things for now
        
        q_feedback         % Temporary variable to store things for now
        q_d_feedback       % Temporary variable to store things for now
        modelConfig
        forwardKin
        numMotor

        MAX_HAND_ANGLE = 180;
        MIN_HAND_ANGLE = 70;
        BEST_HAND_ANGLE = 94;
        MAX_ARM_ANGLE  = 180;
        MIN_ARM_ANGLE  = 0;
    end
    
    methods
        function eb = PoCaBotExperiment(numMotor_for_test)
            % Create the config
            model_config = ModelConfig('PoCaBot spatial');
            cable_set_id = 'dualcables_simple_gripper_demo_on_larger_scale_frame';
            % Load the SystemKinematics object from the XML
            modelObj = model_config.getModel(cable_set_id);
            % Create the hardware interface
            if(nargin == 0)
                numMotor = 8;
            else
                numMotor = numMotor_for_test;
            end
            cableLengths_full = ones(numMotor,1)*4.05;
            hw_interface = PoCaBotCASPRInterface('COM3', numMotor, cableLengths_full,false);  %1
            eb@ExperimentBase(hw_interface, modelObj);
            eb.modelConfig = model_config;
            eb.numMotor = numMotor;
            %            eb.forwardKin = FKDifferential(modelObj);
        end
        
        function motorSpoolTest(obj)
            % Open the hardware interface
            obj.openHardwareInterface();
            
            % Just detect the device to see if it is correct (should change
            % it later to exit cleanly and throw an error in the future
            obj.hardwareInterface.detectDevice();
            
            % Send the initial lengths to the hardware
            segments = [0.25;0.5;0.75;1.00;1.25;1.5;1.75;2;2.25];
            start_index = 1;
            end_index = start_index + 1;
            dir = sign(end_index - start_index);
            
            obj.hardwareInterface.lengthInitialSend(segments(start_index)); %(1)
            obj.hardwareInterface.systemOnSend();
            input('Ready to go? [Y]:','s');
            
            while(true)
                if(start_index ~= end_index)
                    len = segments(start_index):dir*0.007:segments(end_index);
                    len = [len,segments(end_index)];
                    obj.l_feedback_traj = [];
                    for t = 1:length(len)
                        tic;
                        %send command length to Arduino Mega
                        obj.hardwareInterface.lengthCommandSend(len(t)); %(1)
                        obj.l_feedback_traj(:, t) = obj.hardwareInterface.lengthFeedbackRead;
                        
                        elapsed = toc * 1000;
                        if(elapsed < 50)
                            java.lang.Thread.sleep(50 - elapsed);
                        else
                            toc
                        end
                    end
                    figure;
                    
                    % Pause some time here to make sure the dynamixel reach the last
                    % command position.
                    pause(2);
                    obj.l_feedback_traj(:, t+1) = obj.hardwareInterface.lengthFeedbackRead;
                    len = [len, len(end)];
                    
                    plot(len,'b');
                    hold on;
                    plot(obj.l_feedback_traj,'r');
                    legend('Length CMD','Length Feedback','Location','NorthEastOutside');
                    title(sprintf('from %.3f to %.3f',len(1),len(end)));
                    hold off;
                    start_index = end_index;
                else
                    disp('Cannot move due to inappropriate set! Please try another direction!');
                end
                reply = input('Do you want to Draw or Loose? D/L [L]:','s');
                if isempty(reply)
                    reply = 'L';
                end
                if(reply == 'E' || reply == 'e')
                    break;
                elseif(reply == 'D' || reply == 'd')
                    end_index = end_index - 1;
                    dir = -1;
                else
                    end_index = end_index + 1;
                    dir = 1;
                end
                if(end_index<1 || end_index>length(segments))
                    end_index = start_index;
                end
            end
            % Stop the feedback
            obj.hardwareInterface.systemOffSend();
            % Close the hardware interface
            obj.closeHardwareInterface();
            disp('Application terminated correctly!');
        end
        
        function motorTest(obj)
            % Open the hardware interface
            obj.openHardwareInterface();
            
            % Just detect the device to see if it is correct (should change
            % it later to exit cleanly and throw an error in the future
            obj.hardwareInterface.detectDevice();
            obj.hardwareInterface.switchOperatingMode2CURRENT();
            pause(0.5);
            current = ones(obj.numMotor,1)*20;
            obj.hardwareInterface.systemOnSend();
            input('Ready to go? [Y]:','s');
            index = 1;
            while(index < 500)
                tic;
                current = current*(1);
                obj.hardwareInterface.currentCommandSend(current);
                obj.hardwareInterface.currentFeedbackRead();
                
                index = index + 1;
                elapsed = toc * 1000;
                if(elapsed < 50)
                    java.lang.Thread.sleep(50 - elapsed);
                else
                    elapsed
                end
            end
            % Stop the feedback
            obj.hardwareInterface.systemOffSend();
            % Close the hardware interface
            obj.closeHardwareInterface();
            disp('Application terminated correctly!');
        end
        
        function motorTest_Mode_Current_based_Position(obj)
            % Open the hardware interface
            obj.openHardwareInterface();
            
            % Just detect the device to see if it is correct (should change
            % it later to exit cleanly and throw an error in the future
            obj.hardwareInterface.detectDevice();
            obj.hardwareInterface.switchOperatingMode2POSITION_LIMITEDCURRENT();
            pause(0.5);
            current = ones(obj.numMotor,1)*20;
            obj.hardwareInterface.lengthInitialSend(1); %(1)
            
            obj.hardwareInterface.currentCommandSend(current);
            
            obj.hardwareInterface.systemOnSend();
            input('Ready to go? [Y]:','s');
            
            %send command length to Arduino Mega
            obj.hardwareInterface.lengthCommandSend(1.2); %(1)
            len = obj.hardwareInterface.lengthFeedbackRead;
            
            
            % Stop the feedback
            obj.hardwareInterface.systemOffSend();
            % Close the hardware interface
            obj.closeHardwareInterface();
            disp('Application terminated correctly!');
        end
        
        function runTrajectory(obj, trajectory)
            gripper = Gripper('COM6');
            gripper.initialize();
            gripper.setHandAngle( eb.BEST_HAND_ANGLE +50);
            gripper.setArmAngle( eb.MIN_ARM_ANGLE + 90);
            % Open the hardware interface
            obj.openHardwareInterface();
            
            % Just detect the device to see if it is correct (should change
            % it later to exit cleanly and throw an error in the future
            obj.hardwareInterface.detectDevice();
            
            obj.l_feedback_traj = [];
            obj.l_cmd_traj = [];
            % this procedure is to regulate the pose of the endeffector and
            % make sure that the cable is under the tension.
            obj.hardwareInterface.switchOperatingMode2CURRENT();
            % pause(0.5);
            obj.hardwareInterface.systemOnSend();
            current = ones(obj.numMotor,1)*20;
            obj.hardwareInterface.currentCommandSend(current);
            
%             input('Regulate the pose of the end effector, press any key to continue!');
%             
            str = input('Read the initial position from the file? [Y]:','s');
            if isempty(str)
                str = 'Y';
            else
                str = 'N';
            end
            if(str == 'Y')
                fileID = fopen('C:\Users\Tristan\Desktop\initstate.ini','r');
                formatSpec = '%f';
                position_cmd = fscanf(fileID,formatSpec);
                fclose(fileID);
                
                obj.hardwareInterface.switchOperatingMode2POSITION_LIMITEDCURRENT();
                % Start the system to get feedback
                obj.hardwareInterface.systemOnSend();
                current = ones(obj.numMotor,1)*300;
                obj.hardwareInterface.currentCommandSend(current);
                
                profileAcc = ones(obj.numMotor,1)*100;
                obj.hardwareInterface.setProfileAcceleration(profileAcc);
                profileVel = ones(obj.numMotor,1)*100;
                obj.hardwareInterface.setProfileVelocity(profileVel);
                
                obj.hardwareInterface.motorPositionCommandSend(position_cmd);
                present_position = obj.hardwareInterface.motorPositionFeedbackRead();
                error_position = sum(abs(present_position - position_cmd));
                while (error_position>30)
                    pause(1);
                    present_position = obj.hardwareInterface.motorPositionFeedbackRead();
                    error_position = sum(abs(present_position - position_cmd));
                end
            else
                fileID = fopen('C:\Users\Tristan\Desktop\initstate.ini','w');
                present_position = obj.hardwareInterface.motorPositionFeedbackRead();
                fprintf(fileID,'%d \n',present_position);
                fclose(fileID);
            end
            
            % Update the model with the initial point so that the obj.model.cableLength has the initial lengths
            obj.model.update(trajectory.q{1}, trajectory.q_dot{1}, trajectory.q_ddot{1},zeros(size(trajectory.q_dot{1})));
            % Send the initial lengths to the hardware
            obj.hardwareInterface.lengthInitialSend(obj.model.cableLengths); %(1)
            
            obj.hardwareInterface.switchOperatingMode2POSITION_LIMITEDCURRENT();
            
            % Start the system to get feedback
            obj.hardwareInterface.systemOnSend();
            current = ones(obj.numMotor,1)*350;
            obj.hardwareInterface.currentCommandSend(current);
            
            profileAcc = ones(obj.numMotor,1)*150;
            obj.hardwareInterface.setProfileAcceleration(profileAcc);
            profileVel = ones(obj.numMotor,1)*360;
            obj.hardwareInterface.setProfileVelocity(profileVel);
            
            %             l_prev = obj.model.cableLengths;
            %             q_prev = trajectory.q{1};
            %             q_d_prev = trajectory.q_dot{1};
            
            %     obj.hardwareInterface.lengthCommandSend(obj.model.cableLengths - 0.005);
            t= 1;
            while t <= length(trajectory.timeVector)
                time = trajectory.timeVector(t)
                timing = mod(time,27);
                switch timing
                    case 0
                        gripper.setHandAngle( obj.MAX_HAND_ANGLE);
                    case 13.0
                        gripper.setHandAngle( obj.BEST_HAND_ANGLE);
                    otherwise
                end
                switch time
                    case 134.0
                        gripper.setArmAngle(0);
                    case 139.0
                        gripper.setArmAngle(90);
                    case 161.0
                        gripper.setArmAngle(0);
                    case 166.0
                        gripper.setArmAngle(90);
                    otherwise
                end
                
                % Print time for debugging
                tic;
                %send command length to Arduino Mega
                if (t < length(trajectory.timeVector - 2))
                    %tighten cables while running trajectory
                    %obj.hardwareInterface.lengthCommandSend(obj.model.cableLengths - [0.005;0.002;0.005;0.002;0.005;0.002;0.005;0.002]);   %(1)
                    obj.hardwareInterface.lengthCommandSend(obj.model.cableLengths *(1-0.004));   %(1)
                    %no tightening
                    % obj.hardwareInterface.lengthCommandSend(obj.model.cableLengths); %(1)
                else
                    %loosen all cables at the end
                    obj.hardwareInterface.lengthCommandSend(obj.model.cableLengths); %(1)
                end
                % update cable lengths for next command from trajectory
                obj.model.update(trajectory.q{t}, trajectory.q_dot{t}, trajectory.q_ddot{t},zeros(size(trajectory.q_dot{t})));
                obj.l_cmd_traj(:, t) = obj.model.cableLengths; %(1)
                obj.l_feedback_traj(:, t) = obj.hardwareInterface.lengthFeedbackRead;
                
                elapsed = toc * 1000;
                if(elapsed < 50)
                    java.lang.Thread.sleep(50 - elapsed);
                else
                    toc * 1000
                end
                t=t+1;
            end                        % end of "if handsoff = false"
            % Stop the feedback
            obj.hardwareInterface.systemOffSend();
            % Close the hardware interface
            obj.closeHardwareInterface();
            gripper.disconnect();
            disp('Application terminated normally!');
        end
        %% BELOW METHODS ARE FOR THE LONG TIME CONSTRUCTING TASK
        % init_pos is a vector with a size of 8 by 1 which is the initial
        % position of the motors. q0 is also a vetor with the same size but
        % it is the initial state of the end effector.
        function application_preparation(obj, fo, q0)
            % Open the hardware interface
            obj.openHardwareInterface();
            
            % Just detect the device to see if it is correct (should change
            % it later to exit cleanly and throw an error in the future
            obj.hardwareInterface.detectDevice();
            
            % this procedure is to regulate the pose of the endeffector and
            % make sure that the cable is under the tension.
            obj.hardwareInterface.switchOperatingMode2CURRENT();
            % pause(0.5);
            obj.hardwareInterface.systemOnSend();
            current = ones(obj.numMotor,1)*20;
            obj.hardwareInterface.currentCommandSend(current);
            
            str = input('Read the initial position from the file? [Y]:','s');
            if isempty(str)
                str = 'Y';
            else
                str = 'N';
            end
            if(str == 'Y')
                init_pos = fo.readInitPos_Motors();
                obj.hardwareInterface.switchOperatingMode2POSITION_LIMITEDCURRENT();
                % Start the system to get feedback
                obj.hardwareInterface.systemOnSend();
                current = ones(obj.numMotor,1)*200;
                obj.hardwareInterface.currentCommandSend(current);
                
                profileAcc = ones(obj.numMotor,1)*100;
                obj.hardwareInterface.setProfileAcceleration(profileAcc);
                profileVel = ones(obj.numMotor,1)*100;
                obj.hardwareInterface.setProfileVelocity(profileVel);
                
                obj.hardwareInterface.motorPositionCommandSend(init_pos);
                
                error_position = 100;
                while (error_position>30)
                    pause(0.5);
                    present_position = obj.hardwareInterface.motorPositionFeedbackRead();
                    error_position = sum(abs(present_position - init_pos));
                end
            else
                present_position = obj.hardwareInterface.motorPositionFeedbackRead();
                fo.writeInitPos_Motors(present_position);
            end
            
            % Update the model with the initial point so that the obj.model.cableLength has the initial lengths
            obj.model.update(q0, zeros(size(q0)), zeros(size(q0)),zeros(size(q0)));
            % Send the initial lengths to the hardware
            obj.hardwareInterface.lengthInitialSend(obj.model.cableLengths);
            
            obj.hardwareInterface.switchOperatingMode2POSITION_LIMITEDCURRENT();
            
            % Start the system to get feedback
            obj.hardwareInterface.systemOnSend();
            current = ones(obj.numMotor,1)*350;
            obj.hardwareInterface.currentCommandSend(current);
            
            profileAcc = ones(obj.numMotor,1)*150;
            obj.hardwareInterface.setProfileAcceleration(profileAcc);
            profileVel = ones(obj.numMotor,1)*360;
            obj.hardwareInterface.setProfileVelocity(profileVel);
        end
        
        % run the trajectory directly no need to inilize the hardware which
        % has been done beforehand.
        function runTrajectoryDirectly(obj, trajectory)
            obj.l_cmd_traj = zeros(obj.numMotor,length(trajectory.timeVector));
            obj.l_feedback_traj = zeros(obj.numMotor,length(trajectory.timeVector));
            for t = 1:1:length(trajectory.timeVector)
                % Print time for debugging
                time = trajectory.timeVector(t);
                
                tic;
                % update cable lengths for next command from trajectory
                obj.model.update(trajectory.q(:,t), trajectory.q_dot(:,t), trajectory.q_ddot(:,t),zeros(size(trajectory.q_dot(:,t))));
                
                obj.hardwareInterface.lengthCommandSend(obj.model.cableLengths *(1-0.004));
                
                
                obj.l_cmd_traj(:, t) = obj.model.cableLengths; %(1)
                obj.l_feedback_traj(:, t) = obj.hardwareInterface.lengthFeedbackRead;
                
                elapsed = toc * 1000;
                if(elapsed < 50)
                    java.lang.Thread.sleep(50 - elapsed);
                else
                    toc * 1000
                end
            end
        end
        
        function application_termination(obj)
            obj.hardwareInterface.systemOffSend();
            % Close the hardware interface
            obj.closeHardwareInterface();
            disp('Application terminated normally!');
        end
    end
    
    methods (Static)
        % The arguments time_blend_s and time_blend_e can be only used to
        % decide the acceleration of the triangular/trapezoidal profile.
        function [trajectory] = generateTrajectoryParabolicBlend(q_s, q_e, time_step, time_blend_s, time_blend_e, v_max)
            CASPR_log.Assert(length(q_s) == length(q_e), 'Length of input states are inconsistent!');
            n_dof = length(q_s);
            vmax = abs(v_max);
            delta_q = q_e-q_s;
            distance = norm(delta_q);
            acc_s = vmax/time_blend_s;
            acc_e = vmax/time_blend_e;
            if(1/2*vmax*(time_blend_s+time_blend_e)>=distance)
                % Triangular Profile
                time_acc_s = ceil(sqrt(2*acc_e*distance/acc_s/(acc_s+acc_e))/time_step)*time_step;
                time_acc_e = ceil(sqrt(2*acc_s*distance/acc_e/(acc_s+acc_e))/time_step)*time_step;
                time_const_speed = 0;
            else
                % Trapezoidal Profile
                time_acc_s = ceil(time_blend_s/time_step)*time_step;
                time_acc_e = ceil(time_blend_e/time_step)*time_step;
                distance_const_speed = distance - (vmax*(time_acc_s+time_acc_e)/2);
                if(distance_const_speed<=0)
                    time_const_speed = 0
                else
                    time_const_speed = ceil(distance_const_speed/vmax/time_step)*time_step;
                end
            end
            time_vector = 0 : time_step : time_acc_s+time_acc_e+time_const_speed;
            
            q = zeros(n_dof, length(time_vector));
            q_dot = zeros(n_dof, length(time_vector));
            q_ddot = zeros(n_dof, length(time_vector));
            
            v_max_true = delta_q/(1/2*(time_acc_s+time_acc_e)+time_const_speed);
            acc_true_s = v_max_true/time_acc_s;
            acc_true_e = v_max_true/time_acc_e;
            for t_ind = 1:length(time_vector)
                t = time_vector(t_ind);
                if (t <= time_acc_s)
                    q(:,t_ind) = q_s + acc_true_s*t^2/2;
                    q_dot(:,t_ind) = acc_true_s*t;
                    q_ddot(:,t_ind) = acc_true_s;
                elseif (t <= time_acc_s + time_const_speed)
                    q(:,t_ind) = q_s + v_max_true*time_acc_s/2 + v_max_true * (t-time_acc_s);
                    q_dot(:,t_ind) = v_max_true;
                    q_ddot(:,t_ind) = 0;
                else
                    q(:,t_ind) = -acc_true_e*(t-time_acc_s-time_const_speed-time_acc_e)^2/2 + q_e;
                    q_dot(:,t_ind) = -acc_true_e*(t-time_acc_s-time_const_speed-time_acc_e);
                    q_ddot(:,t_ind) = -acc_true_e;
                end
            end
            trajectory.q = q;
            trajectory.q_dot = q_dot;
            trajectory.q_ddot = q_ddot;
            trajectory.time_vector = time_vector;
        end
    end
    
    methods (Static)
        % A sample experiment to run a trajectory and plot some data
        function ExperimentRunTrajectory()
            clc;
            clear;
            close all;
            trajectory_id = 'traj_1';
            %             trajectory_id = 'up';
            %             trajectory_id = 'down';
            %             trajectory_id = '3DPrinting';
            
            
            exp = PoCaBotExperiment();
            trajectory = exp.modelConfig.getTrajectory(trajectory_id);
            exp.runTrajectory(trajectory);
            
            figure;
            plot(trajectory.timeVector, exp.l_cmd_traj);  hold on;
            % exp.l_cmd_traj(:,1)
            
            %    figure;
            plot(trajectory.timeVector, exp.l_feedback_traj);
            
        end
        
        % A simpler example to just lengthen each cable by a sine wave
        function ExperimentLengthenCableSin()
            clc;
            clear;
            close all;
            exp = PoCaBotExperiment();
            % Open the hardware interface
            exp.openHardwareInterface();
            
            % Detect the device to see if it is correct (should change
            % it later to exit cleanly and throw an error in the future
            exp.hardwareInterface.detectDevice()
            
            
            l0 = [0.5; 0.5; 0.5; 0.5; 0.5; 0.5; 0.5; 0.5];
            exp.hardwareInterface.lengthInitialSend(l0);
            
            % Start the system to get feedback
            exp.hardwareInterface.systemOnSend();
            % Should drive the system for 3 seconds (60 * 0.05s)
            counter = 0;
            for t=1:300
                % Send the 1cm movement down
                counter = counter + 1;
                rad = counter * 0.025;
                length = sin(rad) * 0.05;
                
                exp.hardwareInterface.lengthCommandSend(l0 + [length; length; length; length; length; length; length; length;]);
                % Store the feedback received
                exp.l_feedback_traj(:, t) = exp.hardwareInterface.lengthFeedbackRead;
            end
            
            % Stop the feedback
            exp.hardwareInterface.systemOffSend();
            % Close the hardware interface
            exp.closeHardwareInterface();
            plot(0.05:0.05:3, exp.l_feedback_traj);
        end
    end
end

