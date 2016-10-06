% Will make abstract later
classdef DingbotExperiment < ExperimentBase
    properties (SetAccess = private)
        l_feedback_traj    % Temporary variable to store things for now
        l_cmd_traj         % Temporary variable to store things for now
        modelConfig
    end
        
    methods
        function eb = DingbotExperiment()
            % Create the config
            model_config = DevModelConfig(DevModelConfigType.D_CUHK_DINGBOT);
            cable_set_id = 'original';            
            % Load the SystemKinematics object from the XML
            modelObj = model_config.getModel(cable_set_id);
            % Create the hardware interface
            hw_interface = ArduinoCASPRInterface('COM4', 8);            
            eb@ExperimentBase(hw_interface, modelObj);
            eb.modelConfig = model_config;
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
            obj.hardwareInterface.lengthInitialSend(obj.model.cableLengths);                  
            % Start the system to get feedback     
            obj.hardwareInterface.systemOnSend();
            for t = 1:length(trajectory.timeVector)
                % Print time for debugging
                t
                % Wait for feedback to start the 50ms loop
                obj.hardwareInterface.cmdRead();                
                % Update the model with the trajectory to get lengths to
                % send to the hardware
                obj.model.update(trajectory.q{t}, trajectory.q_dot{t}, trajectory.q_ddot{t},zeros(size(trajectory.q_dot{t})));
                % Send the hardware
                obj.hardwareInterface.lengthCommandSend(obj.model.cableLengths);                
                % Store the command and the feedback
                obj.l_cmd_traj(:, t) = obj.model.cableLengths;
                obj.l_feedback_traj(:, t) = obj.hardwareInterface.feedback;
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
            
            exp = DingbotExperiment();
            trajectory = model_config.getTrajectory(trajectory_id);
            exp.runTrajectory(trajectory);
            figure; 
            plot(trajectory.timeVector, exp.l_cmd_traj);
            figure;
            plot(trajectory.timeVector, exp.l_feedback_traj);
        end
        
        % A simpler example to just lengthen each cable by 1cm
        function ExperimentLengthenCableOneCm()
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
            l0 = [0.5; 0.5; 0.5; 0.5; 0.5; 0.5; 0.5; 0.5];
            exp.hardwareInterface.lengthInitialSend(l0);
            
            % Start the system to get feedback
            exp.hardwareInterface.systemOnSend();
            % Should drive the system for 3 seconds (60 * 0.05s)
            for t=1:60
                % Wait for feedback to start the 50ms loop
                exp.hardwareInterface.cmdRead();
                % Send the 1cm movement down
                exp.hardwareInterface.lengthCommandSend(l0 + [0.010; 0.010; 0.010; 0.010; 0.010; 0.010; 0.010; 0.010]);
                % Store the feedback received
                exp.l_feedback_traj(:, t) = exp.hardwareInterface.feedback;
            end
            
            % Stop the feedback
            exp.hardwareInterface.systemOffSend();
            % Close the hardware interface
            exp.closeHardwareInterface();
            
            plot(0.05:0.05:3, exp.l_feedback_traj);
        end
    end
end