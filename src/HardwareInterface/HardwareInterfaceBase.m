classdef (Abstract) HardwareInterfaceBase < handle            
    methods
        function interface = HardwareInterfaceBase()
        end
    end
    
    methods (Abstract)
        open(obj)                
        close(obj)
        [success] = detectDevice(obj)
        lengthCommandSend(obj, l_cmd)
        lengthInitialSend(obj, l0)        
        cmdRead(obj)
        feedbackRead(obj, cmd_str)
        systemOnSend(obj)        
        systemOffSend(obj)
    end
end

