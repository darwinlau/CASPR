classdef PoCaBotExperiment < ExperimentBase
    %POCABOTEXPERIMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        modelConfig
        idsim              % Inverse dynamics simulator
        fksolver           % Forward kinematics solver
        forwardKin
        numMotor
        timestep
        
        % variables for plotting
        isPlot = true;
        ee_ideal
        ee_real
        hText
        ax_plot
    end
    
    properties
        q_present
        
        % The extention part of the cable with original length of 1 meter 
        % under a tension of 1 Newton. Here we assume that the extension of
        % the cable is linear to the tension.
        % l_original*factor_offset_per_Newton_Meter*force = l_delta
        % l_all = l_original+l_delta = (1+factor_offset_per_Newton_Meter*force)*l_original
        % Given commanded length l_cmd, a length of l_cmd/(1+factor_offset_per_Newton_Meter*force)
        % should be set when under tension force.
        factor_offset_per_Newton_Meter = 0.0006;
        
        l_feedback_traj    % Temporary variable to store things for now
        l_cmd_traj         % Temporary variable to store things for now
        time_abs_traj
        time_rel_traj
        q_feedback_traj         % Temporary variable to store things for now
        q_d_feedback_traj       % Temporary variable to store things for now
        q_cmd_traj
    end
    
    methods
        function exp = PoCaBotExperiment(numMotor,strCableID,timestep)
            % Create the config
            model_config = ModelConfig('PoCaBot spatial');
            % Load the SystemKinematics object from the XML
            modelObj = model_config.getModel(strCableID);
            % Create the hardware interface
            %cableLengths_full = ones(numMotor,1)*4.05;
            cableLengths_full = [6.618; 4.800; 6.632;4.800;6.632;5.545;6.618;5.545];
            
            hw_interface = PoCaBotCASPRInterface('COM11', numMotor, cableLengths_full,false);  %1
            exp@ExperimentBase(hw_interface, modelObj);
            exp.modelConfig = model_config;
            exp.numMotor = numMotor;
            
            %            eb.forwardKin = FKDifferential(modelObj);
            exp.q_present = NaN;
            
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
            gripper = Gripper('COM6');
            gripper.initialize();
            gripper.setHandAngle( eb.BEST_HAND_ANGLE +50);
            gripper.setArmAngle( eb.MIN_ARM_ANGLE + 90);
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
            
            %             input('Regulate the pose of the end effector, press any key to continue!');
            %
            str = input('Read the initial position from the file? [Y]:','s');
            if isempty(str)
                str = 'Y';
            else
                str = 'N';
            end
            if(str == 'Y')
                fileID = fopen('C:\Users\Tristan\Desktop\initstate.ini','r');
                formatSpec = '%f';
                position_cmd = fscanf(fileID,formatSpec);
                fclose(fileID);
                
                obj.hardwareInterface.switchOperatingMode2POSITION_LIMITEDCURRENT();
                % Start the system to get feedback
                obj.hardwareInterface.systemOnSend();
                current = ones(obj.numMotor,1)*300;
                obj.hardwareInterface.currentCommandSend(current);
                
                profileAcc = ones(obj.numMotor,1)*100;
                obj.hardwareInterface.setProfileAcceleration(profileAcc);
                profileVel = ones(obj.numMotor,1)*100;
                obj.hardwareInterface.setProfileVelocity(profileVel);
                
                obj.hardwareInterface.motorPositionCommandSend(position_cmd);
                present_position = obj.hardwareInterface.motorPositionFeedbackRead();
                error_position = sum(abs(present_position - position_cmd));
                while (error_position>30)
                    pause(1);
                    present_position = obj.hardwareInterface.motorPositionFeedbackRead();
                    error_position = sum(abs(present_position - position_cmd));
                end
            else
                fileID = fopen('C:\Users\Tristan\Desktop\initstate.ini','w');
                present_position = obj.hardwareInterface.motorPositionFeedbackRead();
                fprintf(fileID,'%d \n',present_position);
                fclose(fileID);
            end
            
            % Update the model with the initial point so that the obj.model.cableLength has the initial lengths
            obj.model.update(trajectory.q{1}, trajectory.q_dot{1}, trajectory.q_ddot{1},zeros(size(trajectory.q_dot{1})));
            % Send the initial lengths to the hardware
            obj.hardwareInterface.lengthInitialSend(obj.model.cableLengths); %(1)
            
            obj.hardwareInterface.switchOperatingMode2POSITION_LIMITEDCURRENT();
            
            % Start the system to get feedback
            obj.hardwareInterface.systemOnSend();
            current = ones(obj.numMotor,1)*350;
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
                timing = mod(time,27);
                switch timing
                    case 0
                        gripper.setHandAngle( obj.MAX_HAND_ANGLE);
                    case 13.0
                        gripper.setHandAngle( obj.BEST_HAND_ANGLE);
                    otherwise
                end
                switch time
                    case 134.0
                        gripper.setArmAngle(0);
                    case 139.0
                        gripper.setArmAngle(90);
                    case 161.0
                        gripper.setArmAngle(0);
                    case 166.0
                        gripper.setArmAngle(90);
                    otherwise
                end
                
                % Print time for debugging
                tic;
                %send command length to Arduino Mega
                if (t < length(trajectory.timeVector - 2))
                    %tighten cables while running trajectory
                    %obj.hardwareInterface.lengthCommandSend(obj.model.cableLengths - [0.005;0.002;0.005;0.002;0.005;0.002;0.005;0.002]);   %(1)
                    obj.hardwareInterface.lengthCommandSend(obj.model.cableLengths *(1-0.004));   %(1)
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
            gripper.disconnect();
            disp('Application terminated normally!');
        end
        
        function [q_initial] = initialLenQCalibration(obj, l0_guess, sample_duration)
            % Open the hardware interface
            obj.openHardwareInterface();
            % Just detect the device to see if it is correct (should change
            % it later to exit cleanly and throw an error in the future
            obj.hardwareInterface.detectDevice();
            % this procedure is to regulate the pose of the endeffector and
            % make sure that the cable is under the tension.
            obj.hardwareInterface.switchOperatingMode2CURRENT();
            obj.hardwareInterface.systemOnSend();
            current = ones(obj.numMotor,1)*20;
            obj.hardwareInterface.currentCommandSend(current);
            
            input('Ready to sample the relative length? Press enter to get started.');
            obj.hardwareInterface.lengthInitialSend(l0_guess);
            samples_r = [];
            tic;
            sample_time = 0; % seconds
            fprintf('Sampling begins! This will last for %.1f seconds\n',sample_duration);
            length_r_pre = zeros(obj.numMotor,1);
            while(1)
                timestamp = toc;
                [length] = obj.hardwareInterface.lengthFeedbackRead();
                length_r = length - l0_guess;
                
                rotating_direction = (length_r - length_r_pre)>0;
                obj.hardwareInterface.switchEnable(~rotating_direction);
                obj.hardwareInterface.currentCommandSend((~rotating_direction).*current);
                length_r_pre = length_r;
                
                sample_present_r = [timestamp length_r'];
                samples_r = [samples_r;sample_present_r];
                % output the time
                if(floor(timestamp)>sample_time)
                    sample_time = floor(timestamp);
                    fprintf('%ds ',sample_time);
                end
                if(timestamp>=sample_duration)
                    break;
                end
            end
            obj.hardwareInterface.systemOnSend();
            obj.hardwareInterface.currentCommandSend(current);
            fprintf('\nSampling ends!\n');
            fprintf('%d data have been collected within %0.1f seconds!\n',size(samples_r,1),sample_duration);
            
            % Read time data (first column)
            time = samples_r(:,1);
            % Read the lengths (all other columns)
            lengths_r = num2cell(samples_r(:, 2:size(samples_r,2))', 1);
            
            % Sample num reduce the number of data points (select every X data point)
            sample_num = 10;
            % Recompute the time and length data by sampling
            time_sampled = time(1:sample_num:numel(time));
            lengths_r_sampled = lengths_r(1:sample_num:numel(lengths_r));
            
            % Some random joint space trajectory guess
            q_guess = cell(1,numel(time_sampled));
            q_guess(:) = {ones(obj.model.numDofs,1)*0.3};
            
            % Now we are ready to solve for the initial lengths
            disp('Start Running Solver for Initial Lengths:');
            start_tic = tic;
            % IMPORTANT LINE HERE
            [l0_solved, q_solved] = FKLeastSquares.ComputeInitialLengths(obj.model, lengths_r_sampled, l0_guess, 1:obj.model.numCables, q_guess);
            time_elapsed = toc(start_tic);
            fprintf('Finished. It took %.1fs for the computation.\n',time_elapsed);
            q_initial = q_solved(:,1);
            for i=1:numel(lengths_r)
                lengths_solved{i} = l0_solved + lengths_r{i};
            end
            
            % Get the present length and set the hardware accordingly.
            [length_present] = obj.hardwareInterface.lengthFeedbackRead();
            length_present_r = length_present - l0_guess;
            length_present_solved = l0_solved + length_present_r;
            
            % Initialize the hardware and the initial state
            obj.hardwareInterface.lengthInitialSend(length_present_solved);
            obj.hardwareInterface.switchOperatingMode2POSITION_LIMITEDCURRENT();
            % Start the system to get feedback
            obj.hardwareInterface.systemOnSend();
            current = ones(obj.numMotor,1)*400;%400
            obj.hardwareInterface.currentCommandSend(current);
            obj.hardwareInterface.lengthCommandSend(length_present_solved);
            
            profileAcc = ones(obj.numMotor,1)*150;
            profileAcc = profileAcc/(obj.timestep/0.05);
            obj.hardwareInterface.setProfileAcceleration(profileAcc);
            profileVel = ones(obj.numMotor,1)*360;
            obj.hardwareInterface.setProfileVelocity(profileVel);
            
            
            disp('Start Running FK Solver for Present Q:');
            % Get the present q_present by Foward Kinematics
            %             % Initialise the three inverse/forward kinematics solvers
            %             iksim_actual = InverseKinematicsSimulator(obj.model);
            %             fksim_guess = ForwardKinematicsSimulator(obj.model, fksolver);
            %             fksim_corrected = ForwardKinematicsSimulator(obj.model, fksolver);
            q_guess = ones(obj.model.numDofs,1)*0.2;
            [q_present_solved, q_dot_present, compTime] = obj.fksolver.compute(length_present_solved, length_present_solved, 1:obj.model.numCables, q_guess, zeros(size(q_guess)), 1);
            fprintf('FK has been done which cost %.3f seconds.\n',compTime);
            fprintf('The present q is solved which is [');
            fprintf('%.3f  ',q_present_solved);
            fprintf(']\n');
            fprintf('The according cable length is [');
            fprintf('%.3f  ',length_present_solved);
            fprintf(']\n');
            
            obj.q_present = q_present_solved;
            
            
            % disp('Start Running Forward Kinematics Simulation for Solved l0 Lengths');
            % start_tic = tic;
            % fksim_corrected.run(lengths_solved, iksim_actual.cableLengthsDot, iksim_actual.timeVector, q0_guess, iksim_actual.trajectory.q_dot{1});
            % time_elapsed = toc(start_tic);
            % fprintf('End Running Forward Kinematics Simulation for Solved l0 Lengths : %f seconds\n', time_elapsed);
            
            %             q0_capture = [20.264; 0.350; 17.593; 0.0]*pi/180;
            %             obj.model.update(q0_capture, [0; 0; 0; 0], [0; 0; 0; 0], [0; 0; 0; 0]);
            %
            %             l0_solved
            %             obj.model.cableLengths
            %             l0_solved(1:4) - obj.model.cableLengths(1:4)
            %             norm(l0_solved(1:4) - obj.model.cableLengths(1:4))
            %             q0_capture
            %             q_solved(:,1)
        end
        
        %% BELOW METHODS ARE FOR THE LONG TIME CONSTRUCTING TASK
        % init_pos is a vector with a size of 8 by 1 which is the initial
        % position of the motors. q0 is also a vetor with the same size but
        % it is the initial state of the end effector.
        function application_preparation(obj, fo, q0)
            % Open the hardware interface
            obj.openHardwareInterface();
            
            % Just detect the device to see if it is correct (should change
            % it later to exit cleanly and throw an error in the future
            obj.hardwareInterface.detectDevice();
            
            % this procedure is to regulate the pose of the endeffector and
            % make sure that the cable is under the tension.
            obj.hardwareInterface.switchOperatingMode2CURRENT();
            obj.hardwareInterface.systemOnSend();
            current = ones(obj.numMotor,1)*20;
            obj.hardwareInterface.currentCommandSend(current);
            
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
                
                profileAcc = ones(obj.numMotor,1)*50;
                obj.hardwareInterface.setProfileAcceleration(profileAcc);
                profileVel = ones(obj.numMotor,1)*50;
                obj.hardwareInterface.setProfileVelocity(profileVel);
                
                obj.hardwareInterface.motorPositionCommandSend(init_pos);
                current = ones(obj.numMotor,1)*200;%200
                
                obj.hardwareInterface.currentCommandSend(current);
                
                error_position = 100;
                while (error_position>30)
                    pause(0.5);
                    present_position = obj.hardwareInterface.motorPositionFeedbackRead();
                    error_position = sum(abs(present_position - init_pos));
                end
            else
                present_position = obj.hardwareInterface.motorPositionFeedbackRead();
                fo.writeInitPos_Motors(present_position);
            end
            
            % Update the model with the initial point so that the obj.model.cableLength has the initial lengths
            obj.model.update(q0, zeros(size(q0)), zeros(size(q0)),zeros(size(q0)));
            % Send the initial lengths to the hardware
            obj.hardwareInterface.lengthInitialSend(obj.model.cableLengths);
            
            obj.hardwareInterface.switchOperatingMode2POSITION_LIMITEDCURRENT();
            
            % Start the system to get feedback
            obj.hardwareInterface.systemOnSend();
            current = ones(obj.numMotor,1)*400;%400
            obj.hardwareInterface.currentCommandSend(current);
            
            profileAcc = ones(obj.numMotor,1)*150;
            profileAcc = profileAcc/(obj.timestep/0.05);
            obj.hardwareInterface.setProfileAcceleration(profileAcc);
            profileVel = ones(obj.numMotor,1)*360;
            obj.hardwareInterface.setProfileVelocity(profileVel);
            
            % PID Check;
            %fprintf('Check the PID parameters!\n');
            [KpD] = obj.hardwareInterface.getKpD();
            [KpI] = obj.hardwareInterface.getKpI();
            [KpP] = obj.hardwareInterface.getKpP();
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
                obj.q_present = trajectory.q(:,t);
                tic;
                % update cable lengths for next command from trajectory
                % obj.model.update(trajectory.q(:,t), trajectory.q_dot(:,t), trajectory.q_ddot(:,t),zeros(size(trajectory.q_dot(:,t))));
                
                [~, model_temp, ~, ~, ~] = obj.idsim.IDSolver.resolve(trajectory.q(:,t), trajectory.q_dot(:,t), trajectory.q_ddot(:,t), zeros(obj.idsim.model.numDofs,1));
                [offset] = obj.hardwareInterface.getCableOffsetByTensionByMotorAngleError(model_temp.cableForces);
                obj.hardwareInterface.lengthCommandSend(model_temp.cableLengths ./(1+obj.factor_offset_per_Newton_Meter*model_temp.cableForces) + offset);
                
                % Record the relevant states for problem-solving purpose
                obj.time_abs_traj(t) = rem(now,1);
                obj.l_cmd_traj(t, :) = model_temp.cableLengths'; %(1)
                % For recording the length of feedback, first assume the
                % elasticity factor is appropriate. So for get the true
                % length, we calculate back the extended length.
                l_feedback = obj.hardwareInterface.lengthFeedbackRead;
                while(~any(l_feedback+1))
                    l_feedback = obj.hardwareInterface.lengthFeedbackRead;
                end
                obj.l_feedback_traj(t, :) = ((1+obj.factor_offset_per_Newton_Meter*model_temp.cableForces).*l_feedback)';
                % And then, we use the feedbacked length to get the true q.
                [q_solved, ~, ~] = obj.fksolver.compute(obj.l_feedback_traj(t, :)', model_temp.cableLengths, 1:obj.model.numCables, trajectory.q(:,t), zeros(size(trajectory.q(:,t))), 1);
                obj.q_feedback_traj(t,:) = q_solved'; 
                
                ratio = 1;
                if(t/ratio==floor(t/ratio) && obj.isPlot)
                    %                     axes(obj.ax_plot);
                    q0 = trajectory.q(:,t);
                    q1 = q_solved;
                    q_delta = q1-q0;
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
                    java.lang.Thread.sleep((obj.timestep - elapsed)*1000);
                else
                    toc
                end
            end
            data = [obj.time_abs_traj obj.time_rel_traj obj.q_cmd_traj obj.q_feedback_traj obj.l_cmd_traj obj.l_feedback_traj];
            FileOperation.recordData(data);
        end
        
        function application_termination(obj)
            obj.hardwareInterface.systemOffSend();
            % Close the hardware interface
            obj.closeHardwareInterface();
            disp('Application terminated normally!');
        end
    end
    
    methods (Static)
        % The arguments time_blend_s and time_blend_e can be only used to
        % decide the acceleration of the triangular/trapezoidal profile.
        function [trajectory] = generateTrajectoryParabolicBlend(q_s, q_e, time_step, time_blend_s, time_blend_e, v_max)
            CASPR_log.Assert(length(q_s) == length(q_e), 'Length of input states are inconsistent!');
            if(q_s == q_e)
                trajectory.q = q_s;
                trajectory.q_dot = q_s*0;
                trajectory.q_ddot = q_s*0;
                trajectory.timeVector = 0;
                return;
            end
            n_dof = length(q_s);
            vmax = abs(v_max);
            delta_q = q_e-q_s;
            distance = norm(delta_q);
            acc_s = vmax/time_blend_s;
            acc_e = vmax/time_blend_e;
            if(1/2*vmax*(time_blend_s+time_blend_e)>=distance)
                % Triangular Profile
                time_acc_s = ceil(sqrt(2*acc_e*distance/acc_s/(acc_s+acc_e))/time_step)*time_step;
                time_acc_e = ceil(sqrt(2*acc_s*distance/acc_e/(acc_s+acc_e))/time_step)*time_step;
                time_const_speed = 0;
            else
                % Trapezoidal Profile
                time_acc_s = ceil(time_blend_s/time_step)*time_step;
                time_acc_e = ceil(time_blend_e/time_step)*time_step;
                distance_const_speed = distance - (vmax*(time_acc_s+time_acc_e)/2);
                if(distance_const_speed<=0)
                    time_const_speed = 0;
                else
                    time_const_speed = ceil(distance_const_speed/vmax/time_step)*time_step;
                end
            end
            time_vector = 0 : time_step : time_acc_s+time_acc_e+time_const_speed;
            
            q = zeros(n_dof, length(time_vector));
            q_dot = zeros(n_dof, length(time_vector));
            q_ddot = zeros(n_dof, length(time_vector));
            
            v_max_true = delta_q/(1/2*(time_acc_s+time_acc_e)+time_const_speed);
            acc_true_s = v_max_true/time_acc_s;
            acc_true_e = v_max_true/time_acc_e;
            for t_ind = 1:length(time_vector)
                t = time_vector(t_ind);
                if (t <= time_acc_s)
                    q(:,t_ind) = q_s + acc_true_s*t^2/2;
                    q_dot(:,t_ind) = acc_true_s*t;
                    q_ddot(:,t_ind) = acc_true_s;
                elseif (t <= time_acc_s + time_const_speed)
                    q(:,t_ind) = q_s + v_max_true*time_acc_s/2 + v_max_true * (t-time_acc_s);
                    q_dot(:,t_ind) = v_max_true;
                    q_ddot(:,t_ind) = 0;
                else
                    q(:,t_ind) = -acc_true_e*(t-time_acc_s-time_const_speed-time_acc_e)^2/2 + q_e;
                    q_dot(:,t_ind) = -acc_true_e*(t-time_acc_s-time_const_speed-time_acc_e);
                    q_ddot(:,t_ind) = -acc_true_e;
                end
            end
            trajectory.q = q;
            trajectory.q_dot = q_dot;
            trajectory.q_ddot = q_ddot;
            trajectory.timeVector = time_vector;
        end
        
        function plotTrajectory(trajectory)
            figure;
            hold on;
%             plot(trajectory.timeVector',trajectory.q');
            plot(trajectory.timeVector',trajectory.q_dot','*');
            plot(trajectory.timeVector',trajectory.q_ddot','-');
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

