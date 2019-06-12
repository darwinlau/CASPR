classdef XM540_W150 < DynamixelActuatorParaBase
    properties(Constant = true)
        % These properties comply with Dynamixel Protocol 2.0
        % This class is designed for using a Dynamixel XM540_W150.
        % To use another Dynamixel model, such as PRO series, see their details in E-Manual(support.robotis.com) and edit below variables accordingly.
        % Control table address
        ADDR_TORQUE_ENABLE          = 64;                 % Control table address is varing with Dynamixel model
        LEN_TORQUE_ENABLE           = 1;
        % info. related to CURRENT
        % -648 ~ 648 * 2.69[mA]
        ADDR_GOAL_CURRENT           = 102;
        ADDR_PRESENT_CURRENT       = 126;
        LEN_GOAL_CURRENT       = 2;
        LEN_PRESENT_CURRENT    = 2;
        
        % info. related to POSITION
        % - 1,048,575 ~ 1,048,575 * 0.088 deg = -256[rev] ~ 256[rev]
        % 1[rev] : 0 ~ 4,095
        ADDR_GOAL_POSITION           = 116;
        ADDR_PRESENT_POSITION       = 132;
        LEN_GOAL_POSITION       = 4;
        LEN_PRESENT_POSITION    = 4;
        
        % The profile acceleration of the dynamixel;
        % This value could be set to 100;
        ADDR_PROFILE_ACCELERATION = 108;
        LEN_PROFILE_ACCELERATION = 4;
        
        % The profile velocity of the dynamixel
        % This value could be set to 112;
        ADDR_PROFILE_VELOCITY = 112;
        LEN_PROFILE_VELOCITY = 4;
        
        % Drive mode
        ADDR_DRIVE_MODE = 10;
        LEN_DRIVE_MODE = 1;
        ROTATION_DIRECTION_BIT_POSITION = 0;
        
        % Beware of that when trying to modify the operatiing mode, the
        % motors must be turned off.
        ADDR_OPERATING_MODE         = 11;
        LEN_OPERATING_MODE          = 1;
        
        OPERATING_MODE_CURRENT = 0;
        OPERATING_MODE_VELOCITY = 1;
        OPERATING_MODE_POSITION = 3;
        OPERATING_MODE_EXTENDED_POSITION = 4;
        OPERATING_MODE_CURRENT_BASED_POSITION = 5;
        OPERATING_MODE_PWM = 16;
        
        % Hardware Error Status
        % This value indicates hardware error status. For more details, please refer to the Shutdown.
        ADDR_HARDWARE_ERROR_STATUS = 70;
        LEN_HARDWARE_ERROR_STATUS = 1;
        
        % The PID parameters of Position Control Loop
        % Range: 0~16383 for all three paras
        % 
        ADDR_KpD = 80;% KPD = KPD(TBL) / 16
        ADDR_KpI = 82;% KPI = KPI(TBL) / 65536
        ADDR_KpP = 84;% KPP = KPP(TBL) / 128 %84
        
        LEN_KpD = 2;
        LEN_KpI = 2;
        LEN_KpP = 2;

        DXL_MINIMUM_CURRENT_VALUE  = 0;
        DXL_MAXIMUM_CURRENT_VALUE  = 2047;
        DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold
        
        ENCODER_COUNT_PER_TURN = 4096;
        MAX_CURRENT = 2047;% nominal value.
        MAX_TORQUE = 4.7;%N.m 12kg*9.8*0.06
        KpP_SCALE_FACTOR = 128;
        KpI_SCALE_FACTOR = 65536;
        KpD_SCALE_FACTOR = 16;
        
        % Below paras are based on experiences.
        PROFILE_ACC = 60;%40
        PROFILE_VEL = 600;%300
        MAX_WORK_CURRENT = 1000;
        KpP = 1000;%600%2800%1200
        KpI = 50;
        KpD = 1200;%6500
    end
end