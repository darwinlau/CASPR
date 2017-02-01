% Will make abstract later
classdef DingbotExperiment < ExperimentBase
    properties (SetAccess = private)
        l_feedback_traj    % Temporary variable to store things for now
        l_cmd_traj         % Temporary variable to store things for now
        
        q_feedback         % Temporary variable to store things for now
        q_d_feedback       % Temporary variable to store things for now
        modelConfig
        forwardKin
    end
    
    methods
        function eb = DingbotExperiment()
            % Create the config
            model_config = DevModelConfig(DevModelConfigType.D_CUHK_DINGBOT);
            cable_set_id = 'crossZX';
%             model_config = DevModelConfig(DevModelConfigType.D_CUHK_DINGBOT_PLANAR);
%             cable_set_id = 'original';
            % Load the SystemKinematics object from the XML
            modelObj = model_config.getModel(cable_set_id);
            % Create the hardware interface
            hw_interface = ArduinoCASPRInterface('COM6', 8);  %1
            eb@ExperimentBase(hw_interface, modelObj);
            eb.modelConfig = model_config;
            eb.forwardKin = FKDifferential(modelObj);
            %eb.forwardKin = FKLeastSquares(modelObj, FK_LS_ApproxOptionType.FIRST_ORDER_INTEGRATE_QDOT, FK_LS_QdotOptionType.FIRST_ORDER_DERIV);
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
            for t = 1:length(trajectory.timeVector)
                trajectory.timeVector(t)
                % Print time for debugging
                tic;
                %send command length to Arduino Mega
                if (t < length(trajectory.timeVector))
                    %tighten cables while running trajectory
                    %obj.hardwareInterface.lengthCommandSend(obj.model.cableLengths(1) - 0.003 * ones((1),1));   %(1)
                    
                    %no tightening
                    obj.hardwareInterface.lengthCommandSend(obj.model.cableLengths); %(1)
                else
                    %loosen all cables at the end
                    obj.hardwareInterface.lengthCommandSend(obj.model.cableLengths); %(1)
                end
                %read incoming feedback from Arduino Mega
                obj.hardwareInterface.cmdRead();
                % update cable lengths for next command from trajectory
                obj.model.update(trajectory.q{t}, trajectory.q_dot{t}, trajectory.q_ddot{t},zeros(size(trajectory.q_dot{t})));
                obj.l_cmd_traj(:, t) = obj.model.cableLengths; %(1)
                obj.hardwareInterface.feedback
                obj.l_feedback_traj(:, t) = obj.hardwareInterface.feedback; 

                               % testing 17th Nov, Peter
                % update end-effector postition and rotation
                %[q, q_dot] = obj.forwardKin.compute(obj.hardwareInterface.feedback, l_prev, 1:8,  q_prev, q_d_prev, 0.05);
                %obj.q_feedback(:,t) = q;
                %obj.q_d_feedback(:,t) = q_dot;
                %l_prev = obj.model.cableLengths;
                %q_prev = q;
                %q_d_prev = q_dot;
                                % testing17th Nov, Peter
                
                elapsed = toc * 1000;
                if(elapsed < 50)
                    java.lang.Thread.sleep(50 - elapsed);
                else
                    elapsed = toc * 1000
                end

            end
            
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
%             trajectory_id = 'noMovement';
            
            
            exp = DingbotExperiment();
            trajectory = exp.modelConfig.getTrajectory(trajectory_id);
            exp.runTrajectory(trajectory); 
            
            figure;
            plot(trajectory.timeVector, exp.l_cmd_traj);  hold on;
            exp.l_cmd_traj(:,1)

       %    figure;
            plot(trajectory.timeVector, exp.l_feedback_traj);
            
              %New function, need testing
      %      figure;
     %       plot(trajectory.timeVector, exp.q_feedback);
     %       figure;
     %       plot(trajectory.timeVector,exp.q_d_feedback);
            %testing,Peter
        end
        
        % A simpler example to just lengthen each cable by a sine wave
        function ExperimentLengthenCableSin()
            clc;
            clear;
            close all;
            exp = DingbotExperiment();
            % Open the hardware interface
            exp.openHardwareInterface();
            
            % Detect the device to see if it is correct (should change
            % it later to exit cleanly and throw an error in the future
            exp.hardwareInterface.detectDevice()
            
            % Set the initial lengths all to be 0.5m
            %l0 = [0.5];
            %l0 = [0.5; 0.5; 0.5; 0.5; 0.5; 0.5; 0.5; 0.5];
            %exp.hardwareInterface.lengthInitialSend(l0);
            
            l0 = [0.5; 0.5; 0.5; 0.5; 0.5; 0.5; 0.5; 0.5];
            exp.hardwareInterface.lengthInitialSend(l0);
            
            % Start the system to get feedback
            exp.hardwareInterface.systemOnSend();
            % Should drive the system for 3 seconds (60 * 0.05s)
            counter = 0;
            for t=1:300
                % Wait for feedback to start the 50ms loop
                exp.hardwareInterface.cmdRead();
                % Send the 1cm movement down
                counter = counter + 1;
                rad = counter * 0.025;
                length = sin(rad) * 0.05;
                
                exp.hardwareInterface.lengthCommandSend(l0 + [length; length; length; length; length; length; length; length;]);
                % Store the feedback received
                %exp.l_feedback_traj(:, t) = exp.hardwareInterface.feedback;
            end
            
            % Stop the feedback
            exp.hardwareInterface.systemOffSend();
            % Close the hardware interface
            exp.closeHardwareInterface();
            plot(0.05:0.05:3, exp.l_feedback_traj);
            
        end
        
    end
end