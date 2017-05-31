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
        
        gripper
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
            cable_set_id = 'dualcables_simple_gripper_demo';
            % Load the SystemKinematics object from the XML
            modelObj = model_config.getModel(cable_set_id);
            % Create the hardware interface
            if(nargin == 0)
                numMotor = 8;
            else
                numMotor = numMotor_for_test;
            end
            cableLengths_full = ones(numMotor,1)*2.05;
            hw_interface = PoCaBotCASPRInterface('COM3', numMotor, cableLengths_full,false);  %1
            eb@ExperimentBase(hw_interface, modelObj);
            eb.modelConfig = model_config;
            eb.numMotor = numMotor;
            %            eb.forwardKin = FKDifferential(modelObj);
            
            eb.gripper = Gripper('COM6');
            eb.gripper.initialize();
            eb.gripper.setHandAngle( eb.BEST_HAND_ANGLE +50);
            eb.gripper.setArmAngle( eb.MIN_ARM_ANGLE);
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
            
            input('Regulate the pose of the end effector, press any key to continue!');
            % Update the model with the initial point so that the obj.model.cableLength has the initial lengths
            obj.model.update(trajectory.q{1}, trajectory.q_dot{1}, trajectory.q_ddot{1},zeros(size(trajectory.q_dot{1})));
            % Send the initial lengths to the hardware
            obj.hardwareInterface.lengthInitialSend(obj.model.cableLengths); %(1)
            
            obj.hardwareInterface.switchOperatingMode2POSITION_LIMITEDCURRENT();
            
            % Start the system to get feedback
            obj.hardwareInterface.systemOnSend();
            current = ones(obj.numMotor,1)*300;
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
                
                switch time
                    case 4.0
                        obj.gripper.setArmAngle(0);
                    case 6.0
                        obj.gripper.setHandAngle( obj.BEST_HAND_ANGLE);
                    case 10.0
                        obj.gripper.setArmAngle(90);
                    case 12.0
                        obj.gripper.setHandAngle( obj.BEST_HAND_ANGLE+50);
                    case 16.0
                        obj.gripper.setArmAngle(0);
                    case 18.0
                        obj.gripper.setHandAngle( obj.BEST_HAND_ANGLE);
                    case 22.0
                        obj.gripper.setArmAngle(90);
                    case 24.0
                        obj.gripper.setHandAngle( obj.BEST_HAND_ANGLE+50);
                    case 28.0
                        obj.gripper.setArmAngle(0);
                    case 30.0
                        obj.gripper.setHandAngle( obj.BEST_HAND_ANGLE);
                    case 34.0
                        obj.gripper.setArmAngle(90);
                    case 36.0
                        obj.gripper.setHandAngle( obj.BEST_HAND_ANGLE+50);
                    case 40.0
                        obj.gripper.setArmAngle(0);
                    otherwise
                end
                
                % Print time for debugging
                tic;
                %send command length to Arduino Mega
                if (t < length(trajectory.timeVector - 2))
                    %tighten cables while running trajectory
                    %obj.hardwareInterface.lengthCommandSend(obj.model.cableLengths - [0.005;0.002;0.005;0.002;0.005;0.002;0.005;0.002]);   %(1)
                    obj.hardwareInterface.lengthCommandSend(obj.model.cableLengths - 0.002);   %(1)
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
            obj.gripper.disconnect();
            disp('Application terminated normally!');
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

