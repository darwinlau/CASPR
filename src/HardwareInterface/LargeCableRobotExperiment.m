% Will make abstract later
classdef LargeCableRobotExperiment < ExperimentBase
    properties (Constant)
        ZERO_CABLE_LENGTHS = [1.80548719; 1.80548719; 1.533880; 1.80548719; 1.80548719; 1.80548719; 1.533880; 1.80548719];
    end
    
    properties (SetAccess = private)
        l_feedback_traj    % Temporary variable to store things for now (NOT USED HERE YET)
        l_cmd_traj         % Temporary variable to store things for now (NOT USED HERE YET)
        
        modelConfig
        modelObj
    end
        
    methods
        function eb = LargeCableRobotExperiment(hw_interface, model_config, model_obj)
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
        function ExperimentRunTrajectory()
            clc;
            clear;
            close all;
            
%             % Create the config properties
            model_config = DevModelConfig('CUCableRobot');
            cable_set_id = 'H_frame';
            trajectory_id = 'O0007';
            
            % Create the config properties
%             model_config = DevModelConfig(DevModelConfigType.D_CUHK_CUCABLEROBOT_PLANAR);
%             cable_set_id = 'vertical_XZ';
%             trajectory_id = 'O1005';
            
            
            % Load the SystemKinematics object from the XML
            model_obj = model_config.getModel(cable_set_id);
            
            % Setup the hardware interface
            hw_interface = GSK_EightAxisStaticCASPRInterface(model_obj, 'data/temp/curobot_trajectories/', trajectory_id, LargeCableRobotExperiment.ZERO_CABLE_LENGTHS);
            exp = LargeCableRobotExperiment(hw_interface, model_config, model_obj);
            
            % Setup the trajectory and run the experiment
            trajectory = model_config.getTrajectory(trajectory_id);
            exp.hardwareInterface.timeStep = trajectory.timeVector(2)-trajectory.timeVector(1);
            trajectory.plotJointSpace();
            exp.runTrajectory(trajectory);
        end
    end
end