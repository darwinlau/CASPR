classdef DynamixelExperiment < ExperimentBase
    %POCABOTEXPERIMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        idsim              % Inverse dynamics simulator
        fksolver           % Forward kinematics solver
        forwardKin
        numMotor
        timestep
        
        % variables for plotting
        isPlot = false;
        ee_ideal
        ee_real
        hText
        ax_plot
        gripper
        
        server
        % This variable is for recording the initial original length for 
        % each cable. So it is a numMotor by 1 vector
        initialLength
    end
    
    properties
        q_present
        
        % Usually the elongation is a linear function of the cable tension, 
        % so here we assume that the stretch of the cable 
        % is linear to the tension.
        % l_original*elongation_per_Newton*force = l_delta
        % l_all = l_original+l_delta = (1+elongation_per_Newton*force)*l_original
        % Given commanded length l_cmd, a length of l_cmd/(1+elongation_per_Newton*force)
        % should be set when under tension force.
        % For the orange PE fiber cable with diameter 0.6mm, its elongation is 6e-4/N,
        % For the orange PE fiber cable with diameter 0.8mm, its elongation is 4.1478e-4/N,
        % For the orange PE fiber cable with diameter 1.2mm, its elongation is 4.7847e-4/N,
        % for the STEALTH-BRAID cable, its elongation is 3.2825e-5/N
%         elongation_per_Newton = [1;0;1;0;1;0;1;0]*3.2825e-5 + [0;1;0;1;0;1;0;1]*4.1478e-4; 
        elongation_per_Newton = ones(8,1)*3.2825e-5;
        
        l_feedback_traj    % Temporary variable to store things for now
        l_cmd_traj         % Temporary variable to store things for now
        time_abs_traj
        time_rel_traj
        q_feedback_traj         % Temporary variable to store things for now
        q_d_feedback_traj       % Temporary variable to store things for now
        q_cmd_traj
        
        % the real time adjustment of the generalised coordinate which
        % could be considered as an interface for real time tuning. This
        % variable could be useful when the input trajectory is not
        % accurate. Generally, this variable is a zero vector.
        q_offset_tuned
    end
    
    methods
        % strCOMPort should be an array of cells, each one of which
        % indicate a COM port with a string format. e.g. strCOMPort = {'COM3','COM5'};
        function exp = DynamixelExperiment(modelObj, strCOMPort, actuatorMotor, timestep, gripperOn, server)
            hw_interface = DynamixelCASPRInterface(strCOMPort, actuatorMotor, modelObj.numActuators,false, gripperOn);  %1
            exp@ExperimentBase(hw_interface, modelObj);
            exp.numMotor = modelObj.numActuators;
            %            eb.forwardKin = FKDifferential(modelObj);
            exp.q_present = NaN;
            exp.q_offset_tuned = zeros(modelObj.numDofVars,1);
            %             id_objective = IDObjectiveMinLinCableForce(ones(modelObj.numActuatorsActive,1));
            %             id_solver = IDSolverLinProg(modelObj, id_objective, ID_LP_SolverType.MATLAB);
            
            id_objective = IDObjectiveMinQuadCableForce(ones(modelObj.numActuatorsActive,1));
            id_solver = IDSolverQuadProg(modelObj, id_objective, ID_QP_SolverType.MATLAB);
            
            exp.idsim = InverseDynamicsSimulator(modelObj, id_solver);
            
            % Initialise the least squares solver for the forward kinematics
            exp.fksolver = FKLeastSquares(exp.model, FK_LS_ApproxOptionType.FIRST_ORDER_INTEGRATE_PSEUDOINV, FK_LS_QdotOptionType.PSEUDO_INV);
            
            if(nargin >=3 && exist('timestep','var'))
                exp.timestep = timestep;
            else
                exp.timestep = 0.05;
            end
            
            if(exp.isPlot)
                figure(100);
                clf;
                ax_bg = axes('Position',[0 0 1 1],'Visible','off');
                
                exp.ax_plot = axes('Position',[.1 .1 .7 .8]);
                title('Comparison between ideal pose and actual pose of the end effector');
                exp.ee_ideal = EndEffector();
                exp.ee_real = EndEffector();
                exp.ee_ideal.plot(1);
                hold on;
                exp.ee_real.plot(0);
                hold off;
                axis equal;
                xlabel('x');ylabel('y');zlabel('z');

                axes(ax_bg);
                descr = {'{\delta{\itq}}:'};
                exp.hText = text(0.77,0.65,descr);
                
                axes(exp.ax_plot);
            end
            if(nargin >=3 && exist('server','var'))
                exp.server = server;
            end
            if(nargin >=3 && exist('gripperOn','var'))
                exp.gripper = gripperOn;
            end

        end
        
%         function motorTest(obj)
%             % Open the hardware interface
%             obj.openHardwareInterface();
%             
%             % Just detect the device to see if it is correct (should change
%             % it later to exit cleanly and throw an error in the future
%             obj.hardwareInterface.detectDevice();
%             obj.hardwareInterface.switchOperatingMode2CURRENT();
%             pause(0.5);
%             current = ones(obj.numMotor,1)*20;
%             obj.hardwareInterface.systemOnSend();
%             input('Ready to go? [Y]:','s');
%             index = 1;
%             while(index < 500)
%                 tic;
%                 current = current*(1);
%                 obj.hardwareInterface.forceCommandSend(current);
%                 obj.hardwareInterface.forceFeedbackRead();
%                 
%                 index = index + 1;
%                 elapsed = toc * 1000;
%                 if(elapsed < 50)
%                     java.lang.Thread.sleep(50 - elapsed);
%                 else
%                     elapsed
%                 end
%             end
%             % Stop the feedback
%             obj.hardwareInterface.systemOffSend();
%             % Close the hardware interface
%             obj.closeHardwareInterface();
%             disp('Application terminated correctly!');
%         end
%         
%         function [q_initial] = initialLenQCalibration(obj, l0_guess, sample_duration)
%             % Open the hardware interface
%             obj.openHardwareInterface();
%             % Just detect the device to see if it is correct (should change
%             % it later to exit cleanly and throw an error in the future
%             obj.hardwareInterface.detectDevice();
%             % this procedure is to regulate the pose of the endeffector and
%             % make sure that the cable is under the tension.
%             obj.hardwareInterface.switchOperatingMode2CURRENT();
%             obj.hardwareInterface.systemOnSend();
%             current = ones(obj.numMotor,1)*20;
%             obj.hardwareInterface.forceCommandSend(current);
%             
%             input('Ready to sample the relative length? Press enter to get started.');
%             obj.hardwareInterface.lengthInitialSend(l0_guess);
%             samples_r = [];
%             tic;
%             sample_time = 0; % seconds
%             fprintf('Sampling begins! This will last for %.1f seconds\n',sample_duration);
%             length_r_pre = zeros(obj.numMotor,1);
%             while(1)
%                 timestamp = toc;
%                 [length] = obj.hardwareInterface.lengthFeedbackRead();
%                 length_r = length - l0_guess;
%                 
%                 rotating_direction = (length_r - length_r_pre)>0;
%                 obj.hardwareInterface.switchEnable(~rotating_direction);
%                 obj.hardwareInterface.forceCommandSend((~rotating_direction).*current);
%                 length_r_pre = length_r;
%                 
%                 sample_present_r = [timestamp length_r'];
%                 samples_r = [samples_r;sample_present_r];
%                 % output the time
%                 if(floor(timestamp)>sample_time)
%                     sample_time = floor(timestamp);
%                     fprintf('%ds ',sample_time);
%                 end
%                 if(timestamp>=sample_duration)
%                     break;
%                 end
%             end
%             obj.hardwareInterface.systemOnSend();
%             obj.hardwareInterface.forceCommandSend(current);
%             fprintf('\nSampling ends!\n');
%             fprintf('%d data have been collected within %0.1f seconds!\n',size(samples_r,1),sample_duration);
%             
%             % Read time data (first column)
%             time = samples_r(:,1);
%             % Read the lengths (all other columns)
%             lengths_r = num2cell(samples_r(:, 2:size(samples_r,2))', 1);
%             
%             % Sample num reduce the number of data points (select every X data point)
%             sample_num = 10;
%             % Recompute the time and length data by sampling
%             time_sampled = time(1:sample_num:numel(time));
%             lengths_r_sampled = lengths_r(1:sample_num:numel(lengths_r));
%             
%             % Some random joint space trajectory guess
%             q_guess = cell(1,numel(time_sampled));
%             q_guess(:) = {ones(obj.model.numDofs,1)*0.3};
%             
%             % Now we are ready to solve for the initial lengths
%             disp('Start Running Solver for Initial Lengths:');
%             start_tic = tic;
%             % IMPORTANT LINE HERE
%             [l0_solved, q_solved] = FKLeastSquares.ComputeInitialLengths(obj.model, lengths_r_sampled, l0_guess, 1:obj.model.numCables, q_guess);
%             time_elapsed = toc(start_tic);
%             fprintf('Finished. It took %.1fs for the computation.\n',time_elapsed);
%             q_initial = q_solved(:,1);
%             for i=1:numel(lengths_r)
%                 lengths_solved{i} = l0_solved + lengths_r{i};
%             end
%             
%             % Get the present length and set the hardware accordingly.
%             [length_present] = obj.hardwareInterface.lengthFeedbackRead();
%             length_present_r = length_present - l0_guess;
%             length_present_solved = l0_solved + length_present_r;
%             
%             % Initialize the hardware and the initial state
%             obj.hardwareInterface.lengthInitialSend(length_present_solved);
%             obj.hardwareInterface.switchOperatingMode2POSITION_LIMITEDCURRENT();
%             % Start the system to get feedback
%             obj.hardwareInterface.systemOnSend();
%             current = ones(obj.numMotor,1)*400;%400
%             obj.hardwareInterface.forceCommandSend(current);
%             obj.hardwareInterface.lengthCommandSend(length_present_solved);
%             
%             profileAcc = ones(obj.numMotor,1)*150;
%             profileAcc = profileAcc/(obj.timestep/0.05);
%             obj.hardwareInterface.setProfileAcceleration(profileAcc);
%             profileVel = ones(obj.numMotor,1)*360;
%             obj.hardwareInterface.setProfileVelocity(profileVel);
%             
%             
%             disp('Start Running FK Solver for Present Q:');
%             % Get the present q_present by Foward Kinematics
%             %             % Initialise the three inverse/forward kinematics solvers
%             %             iksim_actual = InverseKinematicsSimulator(obj.model);
%             %             fksim_guess = ForwardKinematicsSimulator(obj.model, fksolver);
%             %             fksim_corrected = ForwardKinematicsSimulator(obj.model, fksolver);
%             q_guess = ones(obj.model.numDofs,1)*0.2;
%             [q_present_solved, q_dot_present, compTime] = obj.fksolver.compute(length_present_solved, length_present_solved, 1:obj.model.numCables, q_guess, zeros(size(q_guess)), 1);
%             fprintf('FK has been done which cost %.3f seconds.\n',compTime);
%             fprintf('The present q is solved which is [');
%             fprintf('%.3f  ',q_present_solved);
%             fprintf(']\n');
%             fprintf('The according cable length is [');
%             fprintf('%.3f  ',length_present_solved);
%             fprintf(']\n');
%             
%             obj.q_present = q_present_solved;
%             
%             
%             % disp('Start Running Forward Kinematics Simulation for Solved l0 Lengths');
%             % start_tic = tic;
%             % fksim_corrected.run(lengths_solved, iksim_actual.cableLengthsDot, iksim_actual.timeVector, q0_guess, iksim_actual.trajectory.q_dot{1});
%             % time_elapsed = toc(start_tic);
%             % fprintf('End Running Forward Kinematics Simulation for Solved l0 Lengths : %f seconds\n', time_elapsed);
%             
%             %             q0_capture = [20.264; 0.350; 17.593; 0.0]*pi/180;
%             %             obj.model.update(q0_capture, [0; 0; 0; 0], [0; 0; 0; 0], [0; 0; 0; 0]);
%             %
%             %             l0_solved
%             %             obj.model.cableLengths
%             %             l0_solved(1:4) - obj.model.cableLengths(1:4)
%             %             norm(l0_solved(1:4) - obj.model.cableLengths(1:4))
%             %             q0_capture
%             %             q_solved(:,1)
%         end
%         
%         function rehabilitation_preparation(obj)
%             % Open the hardware interface
%             obj.openHardwareInterface();
%             
%             % Just detect the device to see if it is correct (should change
%             % it later to exit cleanly and throw an error in the future
%             obj.hardwareInterface.detectDevice();
%             
%             % this procedure is to regulate the pose of the endeffector and
%             % make sure that the cable is under the tension.
%             obj.hardwareInterface.switchOperatingMode2CURRENT();
%             obj.hardwareInterface.systemOnSend();
%             current = ones(obj.numMotor,1)*20;
%             obj.hardwareInterface.forceCommandSend(current);
%         end
%         
%         function rehabilitation_run(obj, duration)
%             fac = [8 1 3 2;4 5 7 6;2 3 5 4;6 7 1 8;1 7 5 3;6 8 2 4];
%             maxCurrent = 20;
%             omega = pi*4;%ran/s
%             tstart = tic;
%             current = ones(8,1);
%             while(1)
%                 timestamp = toc(tstart);
%                 if(timestamp>=duration)
%                     break;
%                 end
%                 tloop = tic;
%                 
%                 direction = 1;%random('unid',6);
%                 if(floor(direction/2) == direction/2)
%                     direction_counterpart = direction - 1;
%                 else
%                     direction_counterpart = direction + 1;
%                 end
%                 
% %                 current(fac(direction,:))             = (maxCurrent-20)*(sin(omega*timestamp)>0)+20;
% %                 current(fac(direction_counterpart,:)) = (maxCurrent-20)*(sin(omega*timestamp)<=0)+20;
%                 current = ones(8,1)*15;
%                 obj.hardwareInterface.forceCommandSend(current);
%                 
%                 elapsed = toc(tloop);
%                 if(elapsed < 0.05)
%                     java.lang.Thread.sleep((0.05 - elapsed)*1000);
%                 else
%                     toc
%                 end
%             end
%         end
        
        % init_pos is a vector with a size of 8 by 1 which is the initial
        % position of the motors. q0 is also a vetor with the same size but
        % it is the initial state of the end effector.
        function application_preparation(obj, fo, q0)
            obj.q_present = q0;
            
            % Open the hardware interface
            obj.openHardwareInterface();
            
            % Just detect the device to see if it is correct (should change
            % it later to exit cleanly and throw an error in the future
            obj.hardwareInterface.detectDevice();
            
            % this procedure is to regulate the pose of the endeffector and
            % make sure that the cable is under the tension(the very 
            % tension to keep the cable from slack).
            obj.hardwareInterface.switchOperatingMode2CURRENT();
            obj.hardwareInterface.systemOnSend();
            
            initialCurrent = obj.hardwareInterface.ActuatorParas.MAX_WORK_CURRENT/15;
            current = ones(obj.numMotor,1)*initialCurrent;
            cnt = 10;
            for i = 1:cnt
                obj.hardwareInterface.forceCommandSend(current*i/cnt);
                pause(0.1);
            end
            % Update the system state when the end effector is at the initial position.
            obj.model.update(q0, zeros(size(q0)), zeros(size(q0)),zeros(size(q0)));
            [tension] = obj.hardwareInterface.getCableTensionByCurrent(initialCurrent);
            originalLength = obj.model.cableLengths ./(1+obj.elongation_per_Newton.*tension);
            
            profileVel = ones(obj.numMotor,1)*obj.hardwareInterface.ActuatorParas.PROFILE_VEL;
            profileAcc = ones(obj.numMotor,1)*obj.hardwareInterface.ActuatorParas.PROFILE_ACC;
            
            
            str = input('Read the initial position from the file? [Y]:','s');
            if isempty(str) || str == 'Y' || str == 'y'
                str = 'Y';
            else
                str = 'N';
            end
            if(str == 'Y')
                init_pos = fo.readInitPos_Motors();
                obj.hardwareInterface.switchOperatingMode2POSITION_LIMITEDCURRENT();
                % Start the system to get feedback
                obj.hardwareInterface.systemOnSend();
                obj.hardwareInterface.setKpP(ones(obj.numMotor,1)*obj.hardwareInterface.ActuatorParas.KpP);
                
                obj.hardwareInterface.setProfileAcceleration(profileAcc);
                obj.hardwareInterface.setProfileVelocity(profileVel);
                
                obj.hardwareInterface.motorPositionCommandSend(init_pos);
                
                current = ones(obj.numMotor,1)*obj.hardwareInterface.ActuatorParas.MAX_WORK_CURRENT/3;
                obj.hardwareInterface.forceCommandSend(current);
                
                error_position = 1000;
                while (error_position>120)
                    pause(0.5);
                    present_position = obj.hardwareInterface.motorPositionFeedbackRead();
                    error_position = sum(abs(present_position - init_pos));
                end
            else
                present_position = obj.hardwareInterface.motorPositionFeedbackRead();
                
                % Try to get the motor position in case of no tension or
                % tension is zero.
                [motorPosOffset] = obj.hardwareInterface.getMotorPosInitOffset(obj.model.cableLengths - originalLength);
                motorPosOffset = motorPosOffset.*0; %%%%%%% zero offset for initial pos??
                init_pos = present_position + motorPosOffset;
                
                fo.writeInitPos_Motors(init_pos);
            end
            
            % Send the initial commands to the hardware
            obj.initialLength = obj.model.cableLengths;
            obj.hardwareInterface.lengthInitialSend(obj.model.cableLengths);
            obj.hardwareInterface.motorPosInitialSend(init_pos);
            
            obj.hardwareInterface.switchOperatingMode2POSITION_LIMITEDCURRENT();
            obj.hardwareInterface.setKpP(ones(obj.numMotor,1)*obj.hardwareInterface.ActuatorParas.KpP);
            obj.hardwareInterface.setKpI(ones(obj.numMotor,1)*obj.hardwareInterface.ActuatorParas.KpI);
            obj.hardwareInterface.setKpD(ones(obj.numMotor,1)*obj.hardwareInterface.ActuatorParas.KpD);
            % Get the system ready to work
            obj.hardwareInterface.systemOnSend();
            current = ones(obj.numMotor,1)*obj.hardwareInterface.ActuatorParas.MAX_WORK_CURRENT;
            obj.hardwareInterface.forceCommandSend(current);
            
            profileAcc = profileAcc/(obj.timestep/0.05)/2;
            obj.hardwareInterface.setProfileAcceleration(profileAcc);
            obj.hardwareInterface.setProfileVelocity(profileVel/2);
            
            % PID Check;
            %fprintf('Check the PID parameters!\n');
            [KpD] = obj.hardwareInterface.getKpD()
            [KpI] = obj.hardwareInterface.getKpI()
            [KpP] = obj.hardwareInterface.getKpP()
        end
        
        % This function initialize the hardware directly from its guess
        % pose
        
        function initialisingPocabot(obj, q0)
            obj.q_present = q0;
            obj.model.update(q0, zeros(size(q0)), zeros(size(q0)),zeros(size(q0)));
            profileVel = ones(obj.numMotor,1)*obj.hardwareInterface.ActuatorParas.PROFILE_VEL;
            profileAcc = ones(obj.numMotor,1)*obj.hardwareInterface.ActuatorParas.PROFILE_ACC;
            present_position = obj.hardwareInterface.motorPositionFeedbackRead();
            initialCurrent = obj.hardwareInterface.ActuatorParas.MAX_WORK_CURRENT/15;
            [tension] = obj.hardwareInterface.getCableTensionByCurrent(initialCurrent);
            originalLength = obj.model.cableLengths ./(1+obj.elongation_per_Newton.*tension);

%             [motorPosOffset] = obj.hardwareInterface.getMotorPosInitOffset(obj.model.cableLengths - originalLength);
%             init_pos = present_position + motorPosOffset; % in motor encoder measurement

            init_pos = present_position;
            
            % Send the initial commands to the hardware
            obj.initialLength = obj.model.cableLengths;
            obj.hardwareInterface.lengthInitialSend(obj.model.cableLengths);
            obj.hardwareInterface.motorPosInitialSend(init_pos);
            
            obj.hardwareInterface.switchOperatingMode2POSITION_LIMITEDCURRENT();
            % %%%%
            obj.hardwareInterface.setKpP(ones(obj.numMotor,1)*obj.hardwareInterface.ActuatorParas.KpP);
%             obj.hardwareInterface.setKpP([2000;1000;2000;1000;2000;1000;2000;1000;]);
            % %%%%
            try
                obj.hardwareInterface.setKpI(ones(obj.numMotor,1)*obj.hardwareInterface.ActuatorParas.KpI);
                obj.hardwareInterface.setKpD(ones(obj.numMotor,1)*obj.hardwareInterface.ActuatorParas.KpD);
            catch
            end
            % Get the system ready to work
            obj.hardwareInterface.systemOnSend();
            current = ones(obj.numMotor,1)*obj.hardwareInterface.ActuatorParas.MAX_WORK_CURRENT;
            obj.hardwareInterface.forceCommandSend(current);
            
            profileAcc = profileAcc/(obj.timestep/0.05)/2;
            obj.hardwareInterface.setProfileAcceleration(profileAcc);
            obj.hardwareInterface.setProfileVelocity(profileVel/2);        
        end
        
        function preparePosLimitedCurrent(obj,currentsize)
            if nargin < 2
                posLimitedCurrent = obj.hardwareInterface.ActuatorParas.MAX_WORK_CURRENT;
            else
                posLimitedCurrent = currentsize;
            end
            step_current = 10;
            present_position = obj.hardwareInterface.motorPositionFeedbackRead();
            initialCurrent = obj.hardwareInterface.ActuatorParas.MAX_WORK_CURRENT/15;
            [tension] = obj.hardwareInterface.getCableTensionByCurrent(initialCurrent);
            originalLength = obj.model.cableLengths ./(1+obj.elongation_per_Newton.*tension);

            [motorPosOffset] = obj.hardwareInterface.getMotorPosInitOffset(obj.model.cableLengths - originalLength);
            init_pos = present_position + motorPosOffset; % in motor encoder measurement
            obj.initialLength = obj.model.cableLengths;
            obj.hardwareInterface.lengthInitialSend(obj.model.cableLengths);
            obj.hardwareInterface.motorPosInitialSend(init_pos);
  
            obj.hardwareInterface.systemOnSend();
            current = ones(obj.numMotor,1)*obj.hardwareInterface.ActuatorParas.MAX_WORK_CURRENT;
            motorMode = obj.hardwareInterface.getOperatingMode;
            motorCurrentTarget = obj.hardwareInterface.goalForceFeedbackRead;
            index = [];
            for j = 1: length(motorCurrentTarget)
                if motorMode(j) == obj.hardwareInterface.ActuatorParas.OPERATING_MODE_CURRENT_BASED_POSITION
                    index = [index;j];
                end
            end
            for i = 1: sum(motorMode(:)==obj.hardwareInterface.ActuatorParas.OPERATING_MODE_CURRENT_BASED_POSITION)
                motorCurrentTarget(index(i)) = posLimitedCurrent;
            end
%             for i = 1: round(max(abs(obj.hardwareInterface.goalForceFeedbackRead) - abs(motorCurrentTarget))/step_current,-1)
%             end
            obj.hardwareInterface.forceCommandSend(motorCurrentTarget);
        end
        
        % run the trajectory directly no need to inilize the hardware which
        % has been done beforehand.
        function runTrajectoryDirectly(obj, trajectory)
            obj.l_cmd_traj = zeros(length(trajectory.timeVector),obj.numMotor);
            obj.l_feedback_traj = zeros(length(trajectory.timeVector),obj.numMotor);
            obj.time_abs_traj = zeros(length(trajectory.timeVector),1);
            obj.q_feedback_traj = zeros(length(trajectory.timeVector),obj.model.numDofs);
            obj.time_rel_traj = trajectory.timeVector';
            obj.q_cmd_traj = trajectory.q';
            if(isrow(obj.time_rel_traj))
                obj.time_rel_traj = trajectory.timeVector;
            end
            
            for t = 1:1:length(trajectory.timeVector)
                % Print time for debugging
                % time = trajectory.timeVector(t);
                try
                    trajectory.q = cell2mat(trajectory.q);
                    trajectory.q_dot = cell2mat(trajectory.q_dot);
                    trajectory.q_ddot = cell2mat(trajectory.q_ddot);
                catch
                end
                obj.q_present = trajectory.q(:,t);
                trajectory.q(:,t) = obj.q_present + obj.q_offset_tuned;
                tic;
                % update cable lengths for next command from trajectory
                obj.model.update(trajectory.q(:,t), trajectory.q_dot(:,t), trajectory.q_ddot(:,t),zeros(size(trajectory.q_dot(:,t))));
                
%                 [~, model_temp, ~, ~, ~] = obj.idsim.IDSolver.resolve(trajectory.q(:,t), trajectory.q_dot(:,t), trajectory.q_ddot(:,t), zeros(obj.idsim.model.numDofs,1));
%                 [~] = obj.hardwareInterface.getCableOffsetByTension(model_temp.cableForces);
%                 [offset] = obj.hardwareInterface.IDcableOffset(model_temp.cableForces,model_temp.cableLengths);
%                 if(any(model_temp.cableForces<0))
%                     fprintf('The force solved from ID gets beyond the limit!!!\n');
%                 end
%                 offset = zeros(size(offset,1),size(offset,2)); %%TEMP
%                 offset = [0.004;0.002;0.004;0.002;0.004;0.002;0.004;0.002;]*0.1;
                lengthCommand = obj.model.cableLengths
%               
%                 lengthCommand = model_temp.cableLengths ./(1+obj.elongation_per_Newton.*model_temp.cableForces);
%                 lengthCommand = model_temp.cableLengths ./(1+obj.elongation_per_Newton.*model_temp.cableForces) - offset;
%                 lengthModifiedElongation = lengthCommand - obj.initialLength;
%                 if(lengthModifiedElongation > 0)
%                     lengthTuned
%                 else
%                     
%                 end
                lengthTuned = 0;
                obj.hardwareInterface.lengthCommandSend(lengthCommand + lengthTuned);
                %fprintf(obj.server, '(%f,%f,%f,%f,%f,%f)\n', obj.fksolver.model.q);
                
%                 % Record the relevant states for problem-solving purpose
%                 obj.time_abs_traj(t) = rem(now,1);
%                 obj.l_cmd_traj(t, :) = model_temp.cableLengths'; %
%                 
%                 % For recording the length of feedback, first assume the
%                 % elasticity factor is appropriate. So for get the true
%                 % length, we calculate back the extended length.
%                 l_feedback = obj.hardwareInterface.lengthFeedbackRead;
%                 while(~any(l_feedback+1))
%                     l_feedback = obj.hardwareInterface.lengthFeedbackRead;
%                 end
%                 obj.l_feedback_traj(t, :) = ((1+obj.elongation_per_Newton.*model_temp.cableForces).*l_feedback)';
%                 % And then, we use the feedbacked length to get the true q.
%                 [q_solved, ~, ~] = obj.fksolver.compute(obj.l_feedback_traj(t, :)', model_temp.cableLengths, 1:obj.model.numCables, trajectory.q(:,t), zeros(size(trajectory.q(:,t))), 1);
%                 obj.q_feedback_traj(t,:) = q_solved'; 
                
                
                ratio = 1;
                if(t/ratio==floor(t/ratio) && obj.isPlot)
                    %                     axes(obj.ax_plot);
                    q0 = trajectory.q(:,t);
                    q1 = q_solved;
                    q_delta = q1-q0;
                    
                    if(any(abs(q_delta(4:end))>0.2))
                        x=1;
                    end
                    
                    axis([q0(1)-0.2 q0(1)+0.2 q0(2)-0.2 q0(2)+0.2 q0(3)-0.2 q0(3)+0.2]);
                    descr = {'{\delta{\itq}}:';...
                        sprintf('%.5f',q_delta(1));...
                        sprintf('%.5f',q_delta(2));...
                        sprintf('%.5f',q_delta(3));...
                        sprintf('%.5f',q_delta(4));...
                        sprintf('%.5f',q_delta(5));...
                        sprintf('%.5f',q_delta(6))};
                    obj.ee_ideal.animate(q0);
                    obj.ee_real.animate(q1);
                    set(obj.hText,'String',descr);
                    drawnow;
                end
                
                elapsed = toc;
                if(elapsed < obj.timestep)
                    %java.lang.Thread.sleep((obj.timestep - elapsed)*1000);
                    pause(obj.timestep - elapsed);
                else
                    toc
                    pause(0.001);
                end
            end
%             data = [obj.time_abs_traj obj.time_rel_traj obj.q_cmd_traj obj.q_feedback_traj obj.l_cmd_traj obj.l_feedback_traj];
%             FileOperation.recordData(data);
        end

        function [data] = runTrajectoryForCalibration(obj, trajectory, A, b)
            if nargin < 4
                b = zeros(8,1);
                A = zeros(8,34);
            end
            obj.l_cmd_traj = zeros(length(trajectory.timeVector),obj.numMotor);
            obj.l_feedback_traj = zeros(length(trajectory.timeVector),obj.numMotor);
            obj.time_abs_traj = zeros(length(trajectory.timeVector),1);
            obj.q_feedback_traj = zeros(length(trajectory.timeVector),obj.model.numDofs);
            obj.time_rel_traj = trajectory.timeVector';
            obj.q_cmd_traj = trajectory.q';
            data.lengthfeedback = cell(1,length(trajectory.timeVector));
            data.lengthTarget = cell(1,length(trajectory.timeVector));
            data.currentTarget = cell(1,length(trajectory.timeVector));
            data.currentfeedback = cell(1,length(trajectory.timeVector));
            if(isrow(obj.time_rel_traj))
                obj.time_rel_traj = trajectory.timeVector;
            end
            offsetLength = obj.hardwareInterface.lengthFeedbackRead;
            
            %running trajectory
            for t = 1:1:length(trajectory.timeVector)
                data.lengthfeedback{t} = obj.hardwareInterface.lengthFeedbackRead;% - offsetLength;
                data.lengthTarget{t} = obj.model.cableLengths;
                data.currentTarget{t} = obj.hardwareInterface.goalForceFeedbackRead();
                data.currentfeedback{t} = obj.hardwareInterface.forceFeedbackRead();
                x = [trajectory.q(:,t);trajectory.q_dot(:,t);trajectory.q_ddot(:,t); data.currentfeedback{t}; data.lengthTarget{t}];
                runloop = tic;
                obj.q_present = trajectory.q(:,t);
                trajectory.q(:,t) = obj.q_present + obj.q_offset_tuned;
                % update cable lengths for next command from trajectory
                obj.model.update(trajectory.q(:,t), trajectory.q_dot(:,t), trajectory.q_ddot(:,t),zeros(size(trajectory.q_dot(:,t))));
%                 [~, model_temp, ~, ~, ~] = obj.idsim.IDSolver.resolve(trajectory.q(:,t), trajectory.q_dot(:,t), trajectory.q_ddot(:,t), zeros(obj.idsim.model.numDofs,1));
%                 lengthCommand = model_temp.cableLengths ./(1+obj.elongation_per_Newton.*model_temp.cableForces);
                lengthOffset = A*x + b;
                obj.hardwareInterface.lengthCommandSend(obj.model.cableLengths + lengthOffset);
                
                %Getting required data
                elapsed = toc(runloop);
                if(elapsed < obj.timestep)
                    pause(obj.timestep - elapsed);
                else
                    CASPR_log.Info(sprintf('inaccurate frequency, over running %2dms\n',(elapsed - obj.timestep)*1000));
                end
            end
            data.q = trajectory.q;
            data.q_dot = trajectory.q_dot;
            data.q_ddot = trajectory.q_ddot;
        end
        
        function updateServer(obj, server)
            obj.server = server;
        end
        
        function application_termination(obj)
            obj.hardwareInterface.systemOffSend();
            % Close the hardware interface
            obj.closeHardwareInterface();
            disp('Application terminated normally!');
        end
    end
end

% pulled version