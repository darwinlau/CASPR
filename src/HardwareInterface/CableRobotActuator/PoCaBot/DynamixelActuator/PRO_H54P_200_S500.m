classdef PRO_H54P_200_S500 < DynamixelActuatorParaBase
    properties(Constant = true)
        % These properties comply with Dynamixel Protocol 2.0
        % This class is designed for using a Dynamixel model of PRO_H54P_200_S500, and an U2D2.
        % Control table address
        ADDR_TORQUE_ENABLE          = 512;                 % Control table address is varing with Dynamixel model
        LEN_TORQUE_ENABLE           = 1;
        % info. related to CURRENT
        % -648 ~ 648 * 2.69[mA]
        ADDR_GOAL_CURRENT           = 550;
        ADDR_PRESENT_CURRENT       = 574;
        LEN_GOAL_CURRENT       = 2;
        LEN_PRESENT_CURRENT    = 2;
        
        % info. related to POSITION
        % 251,417 ~ 360 degrees
        ADDR_GOAL_POSITION           = 564;
        ADDR_PRESENT_POSITION       = 580;
        LEN_GOAL_POSITION       = 4;
        LEN_PRESENT_POSITION    = 4;
        
        % The profile acceleration of the dynamixel;
        % This value could be set to 100;
        ADDR_PROFILE_ACCELERATION = 556;
        LEN_PROFILE_ACCELERATION = 4;
        
        % The profile velocity of the dynamixel
        ADDR_PROFILE_VELOCITY = 560;
        LEN_PROFILE_VELOCITY = 4;
        
        % Drive mode
        ADDR_DRIVE_MODE = -1;
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
        OPERATING_MODE_CURRENT_BASED_POSITION = 4;
        OPERATING_MODE_PWM = -1;
        
        % Hardware Error Status
        % This value indicates hardware error status. For more details, please refer to the Shutdown.
        ADDR_HARDWARE_ERROR_STATUS = 518;
        LEN_HARDWARE_ERROR_STATUS = 1;
        
        % The PID parameters of Position Control Loop
        % 
        ADDR_KpD = 528;% KPD = KPD(TBL) / 16
        ADDR_KpI = 530;% KPI = KPI(TBL) / 65536
        ADDR_KpP = 532;% KPP = KPP(TBL) / 128
        
        LEN_KpD = 2;
        LEN_KpI = 2;
        LEN_KpP = 2;

        DXL_MINIMUM_CURRENT_VALUE  = 0;
        % A = V x 33,000 /  2,048 
        % A : Current[mA]
        % V : Present Current or Goal Torque
        % Maximum current = 3.0 A
        % Torque_goal_max = A*2048/33000 = 3000*2048/33000 = 186
        DXL_MAXIMUM_CURRENT_VALUE  = 186;
        DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold
        
        ENCODER_COUNT_PER_TURN = 1003846;
        MAX_CURRENT = 186;% nominal value.
        MAX_TORQUE = 10.1;%N.m
        KpP_SCALE_FACTOR = 1;
        KpI_SCALE_FACTOR = 65536;
        KpD_SCALE_FACTOR = 16;
        
        % Below paras are based on experiences.
        PROFILE_ACC = 600; %150
        PROFILE_VEL = 8000; %300
        MAX_WORK_CURRENT = 1000;
        KpP = 500; %900
        KpI = 0;
        KpD = 100;
    end
end