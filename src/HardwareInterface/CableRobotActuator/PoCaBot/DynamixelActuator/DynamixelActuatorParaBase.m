classdef DynamixelActuatorParaBase < handle
    properties(Abstract, Constant = true)
        % These properties comply with Dynamixel Protocol 2.0
        % This class is designed for using a Dynamixel XH430-W210, and an USB2DYNAMIXEL.
        % To use another Dynamixel model, such as PRO series, see their details in E-Manual(support.robotis.com) and edit below variables yourself.
        % Control table address
        ADDR_TORQUE_ENABLE                % Control table address is different in Dynamixel model
        LEN_TORQUE_ENABLE
        % info. related to CURRENT
        % -648 ~ 648 * 2.69[mA]
        ADDR_GOAL_CURRENT
        ADDR_PRESENT_CURRENT
        LEN_GOAL_CURRENT
        LEN_PRESENT_CURRENT
        
        % info. related to POSITION
        % - 1,048,575 ~ 1,048,575 * 0.088 deg = -256[rev] ~ 256[rev]
        % 1[rev] : 0 ~ 4,095
        ADDR_GOAL_POSITION
        ADDR_PRESENT_POSITION
        LEN_GOAL_POSITION
        LEN_PRESENT_POSITION
        
        % The profile acceleration of the dynamixel;
        % This value could be set to 100;
        ADDR_PROFILE_ACCELERATION
        LEN_PROFILE_ACCELERATION
        
        % The profile velocity of the dynamixel
        % This value could be set to 112;
        ADDR_PROFILE_VELOCITY 
        LEN_PROFILE_VELOCITY 
        
        % Drive mode
        ADDR_DRIVE_MODE
        LEN_DRIVE_MODE
        ROTATION_DIRECTION_BIT_POSITION
        
        % Beware of that when trying to modify the operatiing mode, the
        % motors must be turned off.
        ADDR_OPERATING_MODE 
        LEN_OPERATING_MODE
        
        OPERATING_MODE_CURRENT
        OPERATING_MODE_VELOCITY
        OPERATING_MODE_POSITION
        OPERATING_MODE_EXTENDED_POSITION
        OPERATING_MODE_CURRENT_BASED_POSITION
        OPERATING_MODE_PWM
        
        % Hardware Error Status
        % This value indicates hardware error status. For more details, please refer to the Shutdown.
        ADDR_HARDWARE_ERROR_STATUS
        LEN_HARDWARE_ERROR_STATUS
        
        % The PID parameters of Position Control Loop
        % Range: 0~16383 for all three paras
        % 
        ADDR_KpD % KPD = KPD(TBL) / 16
        ADDR_KpI % KPI = KPI(TBL) / 65536
        ADDR_KpP % KPP = KPP(TBL) / 128
        
        LEN_KpD 
        LEN_KpI 
        LEN_KpP 
        
        DXL_MINIMUM_CURRENT_VALUE 
        DXL_MAXIMUM_CURRENT_VALUE 
        DXL_MOVING_STATUS_THRESHOLD            % Dynamixel moving status threshold
        
        ENCODER_COUNT_PER_TURN
        MAX_CURRENT % nominal value.
        MAX_TORQUE %N.m
        KpP_SCALE_FACTOR
        KpI_SCALE_FACTOR
        KpD_SCALE_FACTOR
        
        % Below paras are based on experiences.
        PROFILE_ACC
        PROFILE_VEL
        MAX_WORK_CURRENT
        KpP
        KpI
        KpD
    end
    
    methods
        function actuatorPara = DynamixelActuatorParaBase()
        end
    end
end