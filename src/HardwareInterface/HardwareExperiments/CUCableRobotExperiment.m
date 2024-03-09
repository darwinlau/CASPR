% Will make abstract later
classdef CUCableRobotExperiment < ExperimentBase
    properties (Constant)
        ZERO_CABLE_LENGTHS = [1.80548719; 1.80548719; 1.80548719; 1.80548719; 1.80548719; 1.80548719; 1.80548719; 1.80548719];
    end
    
    properties (SetAccess = private)
        l_feedback_traj    % Temporary variable to store things for now (NOT USED HERE YET)
        l_cmd_traj         % Temporary variable to store things for now (NOT USED HERE YET)
        
        modelConfig
        modelObj
    end
        
    methods
        function eb = CUCableRobotExperiment(hw_interface, model_config, model_obj)
            % Create the hardware interface          
            eb@ExperimentBase(hw_interface, model_obj);
            eb.modelConfig = model_config;
            eb.modelObj = model_obj;
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
                %t
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
        % cable_set_id: The cable set ID to use 'H_frame' for example
        % trajectory_id: The ID of trajectory that would also affect the
        % file names on the hardware machine. Suggest to follow convention
        % 'O00XX' where XX is the trajectory number, such as 01. Additional
        % codes would be added, such as O00010, O00018 and O00019 for
        % different purposes
        function ExperimentRunTrajectory(cable_set_id, trajectory_id)
            clc;
            clear;
            close all;
            
            % Create the config properties
            model_config = DevModelConfig('CUCableRobot');
            %cable_set_id = 'H_frame';
            %trajectory_id = 'O0008';
            
            % Load the SystemKinematics object from the XML
            model_obj = model_config.getModel(cable_set_id);
            
            % Setup the hardware interface
            hw_interface = GSK_EightAxisStaticCASPRInterface(model_obj, 'data/temp/curobot_trajectories/', trajectory_id, CUCableRobotExperiment.ZERO_CABLE_LENGTHS);
            exp = CUCableRobotExperiment(hw_interface, model_config, model_obj);
            
            % Setup the trajectory and run the experiment
            trajectory = model_config.getJointTrajectory(trajectory_id);
            exp.hardwareInterface.timeStep = trajectory.timeVector(2)-trajectory.timeVector(1);
            trajectory.plotJointPose();
            exp.runTrajectory(trajectory);
        end
    end
end