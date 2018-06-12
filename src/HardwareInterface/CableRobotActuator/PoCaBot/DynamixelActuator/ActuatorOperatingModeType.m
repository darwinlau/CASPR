classdef ActuatorOperatingModeType
    enumeration 
        CURRENT_MODE
        VELOCITY_MODE
        POSITION_MODE %Single circle
        EXTENDED_POSITION_MODE %Multiple circles
        CURRENT_BASED_POSITION_MODE %Multiple circles with constrained current
        PWM_MODE %This mode directly control the PWM output
    end
end