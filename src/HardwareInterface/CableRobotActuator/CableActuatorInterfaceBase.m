% Base class for hardware interfaces to inherit and implement
%
% Author        : Darwin LAU
% Created       : 2017
% Description   : The base class has a set of methods that must be 
% implemented by different interfaces on how to execute the functionality 
classdef (Abstract) CableActuatorInterfaceBase < handle            
    methods
        function interface = CableActuatorInterfaceBase()
        end
    end
    
    methods (Abstract)
        % Method to open the connection or files for the interface
        open(obj)
        % Method to close the connection or files when finished
        close(obj)
        % Method to detect (optional) if the device is alive (returns 1 is
        % device is detected and 0 if not)
        [success] = detectDevice(obj)
        % Method to send the vector of cable lengths (l_cmd) to hardware
        lengthCommandSend(obj, l_cmd)
        % Method to send the initial cable length information to the
        % hardware interface and hardware (if required)
        lengthInitialSend(obj, l0)
        % Method to read the cable lengths from the hardware (if available)
        lengthFeedbackRead(obj)
        % Method to send some "on" state to the hardware (optional)
        systemOnSend(obj)        
        % Method to send some "off" state to the hardware (optional)
        systemOffSend(obj)
        % Method to initialise both the hardware and the software
        initialise(obj)
        % Method to switch the operating mode of all actuators
        % The mode should be specified by the enumeration class
        % 'ActuatorOperatingModeType'
        switchOperatingMode(obj,mode)
        % Method to send the vector of actuator force (f_cmd) to hardware
        forceCommandSend(obj, f_cmd)
        % Method to read the forces from the hardware (if available)
        forceFeedbackRead(obj)
    end
end

