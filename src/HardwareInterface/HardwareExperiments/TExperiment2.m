classdef TExperiment2 < ExperimentBase

    properties
        %         sim              % Inverse dynamics simulator
        % Given commanded length l_cmd, a length of l_cmd/(1+elongation_per_Newton*force)
        l_cmd_traj
        time_abs_traj
        time_rel_traj
        initialLength
        error_position
        q_present
        q_offset_tuned
        q_cmd_traj
        numofmotor
    end
    methods
        function exp = TExperiment2(modelObj,motorarray)
            hw_interface =motorarray;

            exp@ExperimentBase(hw_interface, modelObj);
            %             iksim_actual = InverseKinematicsSimulator(modelObj);
            exp.q_present = NaN;
            % exp.q_offset_tuned = zeros(modelObj.numDofVars);
        end
        function preparation(obj)
            for i = 1:obj.model.numActuators
                obj.hardwareInterface(i).open;
            end
            % current = Obj.cmd_current;
            %obj.hardwareInterface.forceCommandSend(current);
            %            present_position = obj.hw_interface.motorPositionFeedbackRead();
            % Send the initial commands to the hardware
            obj.initialLength = obj.model.cableLengths;
            for i = 1:obj.model.numActuators
                obj.hardwareInterface(i).lengthInitialSend(obj.model.cableLengths(i));
            end
        end
        %              function motorTest(obj)
        %                       obj.hw_interface.open();
        % %       %%%%%
        %                        obj.hw_interface.close();
        %               end
        function runTrajectoryDirectly(obj, trajectory)
            obj.l_cmd_traj = zeros(length(trajectory.timeVector));
            obj.time_rel_traj = trajectory.timeVector';
            obj.q_cmd_traj = trajectory.q';

            for t = 1:1:length(trajectory.timeVector)
                    try
                        trajectory.q = cell2mat(trajectory.q);
                          trajectory.q_dot = cell2mat(trajectory.q_dot);
                        trajectory.q_ddot = cell2mat(trajectory.q_ddot);
                          catch
                            end
            obj.q_present = trajectory.q(:,t);
              trajectory.q(:,t) = obj.q_present ;%+ obj.q_offset_tuned;
            tic;
            % update cable lengths for next command from trajectory
            obj.model.update(trajectory.q(:,t), trajectory.q_dot(:,t), trajectory.q_ddot(:,t),zeros(size(trajectory.q_dot(:,t))));
            lengthCommand = obj.model.cableLengths;
            %                   lengthTuned = 0;
            while (toc<trajectory.timeStep)
                for i = 1:obj.model.numActuators
                    obj.hardwareInterface(i).lengthCommandSend(lengthCommand(i)); %  obj.hw_interface.lengthCommandSend(lengthCommand + lengthTuned);
                end
            end
            end
        end

        function application_termination(obj)
              for i = 1:obj.model.numActuators
                  obj.hardwareInterface.close();
              end
              disp('Application terminated normally!');
        end
    end
end
