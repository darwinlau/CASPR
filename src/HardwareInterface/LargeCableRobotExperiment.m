% Will make abstract later
classdef LargeCableRobotExperiment < ExperimentBase
    properties (SetAccess = private)
        l_feedback_traj    % Temporary variable to store things for now
        l_cmd_traj         % Temporary variable to store things for now
        
        modelConfig
    end
        
    methods
        function eb = LargeCableRobotExperiment()
            % Create the config
            model_config = DevModelConfig(DevModelConfigType.D_CUHK_CUCABLEROBOT);
            cable_set_id = 'original';            
            % Load the SystemKinematics object from the XML
            modelObj = model_config.getModel(cable_set_id);
            % Create the hardware interface
            hw_interface = GSK_EightAxisStaticCASPRInterface(8, '', '');            
            eb@ExperimentBase(hw_interface, modelObj);
            eb.modelConfig = model_config;
        end
        
        function runTrajectory(obj, trajectory)
            % Open the hardware interface
            obj.openHardwareInterface();
            
            % Just detect the device to see if it is correct (should change
            % it later to exit cleanly and throw an error in the future
            obj.hardwareInterface.detectDevice();
            
            % Update the model with the initial point so that the obj.model.cableLength has the initial lengths
            obj.model.update(trajectory.q{1}, trajectory.q_dot{1}, trajectory.q_ddot{1},zeros(size(trajectory.q_dot{1})));
            % Send the initial lengths to the hardware
            l0 = obj.model.cableLengths;
            obj.hardwareInterface.lengthInitialSend(l0);                  
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
                %l_prev = obj.model.cableLengths;
                %l_dot_prev = obj.model.cableLengthsDot;
                
                % Store the command and the feedback
                %obj.l_cmd_traj(:, t) = obj.model.cableLengths;
                %obj.l_feedback_traj(:, t) = obj.hardwareInterface.feedback;
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
            trajectory_id = 'O00014';
            
            exp = LargeCableRobotExperiment();
            trajectory = exp.modelConfig.getTrajectory(trajectory_id);
            exp.hardwareInterface.filefolder = 'data/temp/curobot_trajectories/';
            exp.hardwareInterface.filename = trajectory_id;
            exp.hardwareInterface.timeStep = trajectory.timeStep;
            exp.runTrajectory(trajectory);
        end
    end
end