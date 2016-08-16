% Will make abstract later
classdef ExperimentBase < handle
    properties (SetAccess = private)
        hardwareInterface;
        model
        closeFigureHandle;
    end
    
    properties (Dependent)
        isRunning;
    end
    
    methods
        function eb = ExperimentBase(hw_interface, model)
            eb.hardwareInterface = hw_interface;
            eb.model = model;
        end
        
        function runDataCollectionHundredSamples(obj)
            obj.hardwareInterface.systemOnSend();
            for i = 1:100
                i
                obj.hardwareInterface.cmdRead();
            end
            obj.hardwareInterface.systemOffSend();
        end
        
        function runOneLocation(obj)
            obj.hardwareInterface.systemOnSend();
            X = [0.23;0.23;0.23;0.23;0.23;0.23;0.23;0.23];
            obj.hardwareInterface.lengthCommandSend(X);
            obj.hardwareInterface.cmdRead();
            obj.hardwareInterface.systemOffSend();
        end
        function runTrajectory(obj)
            clc;
            obj.hardwareInterface.systemOnSend();
            load('C:\Users\AdminDing\Desktop\CASPR_private\ref information\cablelength traj1.mat','lengths');
            %lengths
            for i=1:size(lengths,2)
                X = lengths(:,i);
                i
                X = round(X, 3)
            obj.hardwareInterface.lengthCommandSend(X);
                        pause(0.1);
            end
            obj.hardwareInterface.systemOffSend();
        end
        
        function runKinematicTrajectory(obj, trajectory)

            obj.hardwareInterface.systemOnSend();
            X = 200*ones(8,1);
            for t = 1:length(trajectory.timeVector)
                tout = tic;
                obj.hardwareInterface.cmdRead();
                obj.model.update(trajectory.q{t}, trajectory.q_dot{t}, trajectory.q_ddot{t},zeros(size(trajectory.q_dot{t})));
                X = round(obj.model.cableLengths, 3);
                obj.hardwareInterface.lengthCommandSend(X);
                elapsed_time = toc(tout);
                elapsed_time
            end
            obj.hardwareInterface.systemOffSend();
        end
        
        function runWhileCode(obj)
            obj.createCloseFigureHandle();
            obj.hardwareInterface.systemOnSend();
            %obj.hardwareInterface.lengthInitialSend([1.0; 1.0; 1.0; 1.0; 1.0; 1.0; 1.0; 1.0]);
            
            while(obj.isRunning)
                % The code will wait on the serial comm to read until it
                % receives something meaningful from the Arduino
                obj.hardwareInterface.cmdRead();
                
                % After a meaningful command the experiment should process
                % it
                drawnow;
            end
            
            obj.hardwareInterface.systemOffSend();
        end
        
        function createCloseFigureHandle(obj)
            if (~obj.isRunning)
                obj.closeFigureHandle = figure('Position', [100, 100, 200, 50]);
                set(obj.closeFigureHandle, 'MenuBar', 'none');
                set(obj.closeFigureHandle, 'ToolBar', 'none');
                uicontrol('Style', 'PushButton', ...
                        'String', 'Close run() function', ...
                        'Position', [25 15 150 20], ...
                        'Callback', 'delete(gcbf);');
                drawnow;
            end
        end
        
        function closeHardwareInterface(obj)
            obj.hardwareInterface.close();
        end
        
        function value = get.isRunning(obj)
            if (~isempty(obj.closeFigureHandle) && ishandle(obj.closeFigureHandle))
                value = true;
            else
                value = false;
            end
        end
    end
end

