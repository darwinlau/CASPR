% Will make abstract later
classdef ExperimentBase < handle
    properties (SetAccess = private)
        hardwareInterface;
        closeFigureHandle;
    end
    
    properties (Dependent)
        isRunning;
    end
    
    methods
        function eb = ExperimentBase(hw_interface)
            eb.hardwareInterface = hw_interface;
        end
        
        function run(obj)
            obj.createCloseFigureHandle();
            obj.hardwareInterface.systemOnSend();
            %obj.hardwareInterface.lengthInitialSend([1.0; 1.0; 1.0; 1.0; 1.0; 1.0; 1.0; 1.0]);
            
            while(obj.isRunning)
                % The code will wait on the serial comm to read until it
                % receives something meaningful from the Arduino
                obj.hardwareInterface.feedbackRead();
                
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

