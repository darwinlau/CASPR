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
    end
    
    methods
        function eb = PoCaBotExperiment(numMotor_for_test)
            % Create the config
%             model_config = DevModelConfig('PoCaBot spatial');
%             cable_set_id = 'original';
%             % Load the SystemKinematics object from the XML
%             modelObj = model_config.getModel(cable_set_id);
            % Create the hardware interface
            if(isempty(numMotor_for_test))
                numMotor = 8;
            else
                numMotor = numMotor_for_test;
            end
            cableLengths_full = ones(numMotor,1)*2.075;
            hw_interface = PoCaBotCASPRInterface('COM3', numMotor, cableLengths_full,true);  %1
            modelObj = 1;
            eb@ExperimentBase(hw_interface, modelObj);
%             eb.modelConfig = model_config;
%             eb.forwardKin = FKDifferential(modelObj);
            %eb.forwardKin = FKLeastSquares(modelObj, FK_LS_ApproxOptionType.FIRST_ORDER_INTEGRATE_QDOT, FK_LS_QdotOptionType.FIRST_ORDER_DERIV);
        end
        
        function motorSpoolTest(obj)
           % Open the hardware interface
           obj.openHardwareInterface();
           
           % Just detect the device to see if it is correct (should change
           % it later to exit cleanly and throw an error in the future
           obj.hardwareInterface.detectDevice();
           
           % Send the initial lengths to the hardware
           segments = [0.11;0.5;0.75;1.00;1.25;1.5;1.75];
           start_index = length(segments)-2;
           end_index = start_index - 1;
           dir = -1;
           
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
        
        function runTrajectory(obj, trajectory)
            % Open the hardware interface
            obj.openHardwareInterface();
            
            % Just detect the device to see if it is correct (should change
            % it later to exit cleanly and throw an error in the future
            obj.hardwareInterface.detectDevice()
            
            % Update the model with the initial point so that the obj.model.cableLength has the initial lengths
            obj.model.update(trajectory.q{1}, trajectory.q_dot{1}, trajectory.q_ddot{1},zeros(size(trajectory.q_dot{1})));
            % Send the initial lengths to the hardware
            obj.hardwareInterface.lengthInitialSend(obj.model.cableLengths); %(1)
            % Start the system to get feedback
            obj.hardwareInterface.systemOnSend();
            
            %             l_prev = obj.model.cableLengths;
            %             q_prev = trajectory.q{1};
            %             q_d_prev = trajectory.q_dot{1};
            
            %     obj.hardwareInterface.lengthCommandSend(obj.model.cableLengths - 0.005);
            t= 1;
            while t < length(trajectory.timeVector)
                trajectory.timeVector(t)
                % Print time for debugging
                tic;
                %send command length to Arduino Mega
                if (t < length(trajectory.timeVector - 2))
                    %tighten cables while running trajectory
                    obj.hardwareInterface.lengthCommandSend(obj.model.cableLengths - 0.002);   %(1)
                    
                    %no tightening
                    %                     obj.hardwareInterface.lengthCommandSend(obj.model.cableLengths); %(1)
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
            exp.l_cmd_traj(:,1)
            
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

