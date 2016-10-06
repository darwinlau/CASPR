% Will make abstract later
classdef DingbotExperiment < ExperimentBase
    properties (SetAccess = private)
        l_over_time
    end
        
    methods
        function eb = DingbotExperiment(hw_interface, model)
            eb@ExperimentBase(hw_interface, model);
        end
        
        function runTrajectory(obj, trajectory)
            clc;
            obj.openHardwareInterface();
            obj.hardwareInterface.detectDevice()
            
            obj.model.update(trajectory.q{1}, trajectory.q_dot{1}, trajectory.q_ddot{1},zeros(size(trajectory.q_dot{1})));
            obj.hardwareInterface.lengthInitialSend(obj.model.cableLengths);
            
            
            %l = [0.5; 0.5; 0.5; 0.5; 0.5; 0.5; 0.5; 0.5];
%            obj.hardwareInterface.lengthInitialSend(l);
                       
            obj.hardwareInterface.systemOnSend();
            for t = 1:length(trajectory.timeVector)
                t
                obj.hardwareInterface.cmdRead();
                %obj.hardwareInterface.feedback
                %obj.hardwareInterface.lengthCommandSend(l + [0.010; 0.010; 0.010; 0.010; 0.010; 0.010; 0.010; 0.010]);
                %obj.hardwareInterface.lengthCommandSend(obj.model.cableLengths + [0.010; 0.010; 0.010; 0.010; 0.010; 0.010; 0.010; 0.010]);
                %
                obj.model.update(trajectory.q{t}, trajectory.q_dot{t}, trajectory.q_ddot{t},zeros(size(trajectory.q_dot{t})));
                obj.model.cableLengths
                obj.hardwareInterface.lengthCommandSend(obj.model.cableLengths);
                obj.l_over_time(:, t) = obj.model.cableLengths;
            end
            
            obj.hardwareInterface.systemOffSend();
            obj.closeHardwareInterface();
        end
    end
    
    methods (Static)
        function ExperimentTestOne()
            model_config = DevModelConfig(DevModelConfigType.D_CUHK_DINGBOT);
            cable_set_id = 'original';
            trajectory_id = 'traj_1';

            % Load the SystemKinematics object from the XML
            modelObj = model_config.getModel(cable_set_id);
            trajectory = model_config.getTrajectory(trajectory_id);
            
            hw_interface = ArduinoCASPRInterface('COM4', 8);
            exp = DingbotExperiment(hw_interface, modelObj);
            
            exp.runTrajectory(trajectory);
            
            plot(trajectory.timeVector, exp.l_over_time);
            
        end
    end
end