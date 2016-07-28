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
        
        function runKinematicTrajectory(obj, trajectory)
            obj.hardwareInterface.systemOnSend();
            for t = 1:length(trajectory.timeVector)
                obj.hardwareInterface.cmdRead();
                obj.model.update(obj.trajectory.q{t}, obj.trajectory.q_dot{t}, obj.trajectory.q_ddot{t},zeros(size(obj.trajectory.q_dot{t})));
                obj.hardwareInterface.lengthCommandSend(obj.model.cableLengths);
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

