classdef COMM_PACKET_ID < Simulink.IntEnumType
    % MATLAB enumeration class definition generated from template
    
    enumeration
        COMM_FW_VERSION(0),
		COMM_JUMP_TO_BOOTLOADER(1),
		COMM_ERASE_NEW_APP(2),
		COMM_WRITE_NEW_APP_DATA(3),
		COMM_GET_VALUES(4),
		COMM_SET_DUTY(5),
		COMM_SET_CURRENT(6),
		COMM_SET_CURRENT_BRAKE(7),
		COMM_SET_RPM(8),
		COMM_SET_POS(9),
		COMM_SET_HANDBRAKE(10),
		COMM_SET_DETECT(11),
		COMM_SET_SERVO_POS(12),
		COMM_SET_MCCONF(13),
		COMM_GET_MCCONF(14),
		COMM_GET_MCCONF_DEFAULT(15),
		COMM_SET_APPCONF(16),
		COMM_GET_APPCONF(17),
		COMM_GET_APPCONF_DEFAULT(18),
		COMM_SAMPLE_PRINT(19),
		COMM_TERMINAL_CMD(20),
		COMM_PRINT(21),
		COMM_ROTOR_POSITION(22),
		COMM_EXPERIMENT_SAMPLE(23),
		COMM_DETECT_MOTOR_PARAM(24),
		COMM_DETECT_MOTOR_R_L(25),
		COMM_DETECT_MOTOR_FLUX_LINKAGE(26),
		COMM_DETECT_ENCODER(27),
		COMM_DETECT_HALL_FOC(28),
		COMM_REBOOT(29),
		COMM_ALIVE(30),
		COMM_GET_DECODED_PPM(31),
		COMM_GET_DECODED_ADC(32),
		COMM_GET_DECODED_CHUK(33),
		COMM_FORWARD_CAN(34),
		COMM_SET_CHUCK_DATA(35),
		COMM_CUSTOM_APP_DATA(36),
		COMM_NRF_START_PAIRING(37),
		COMM_GPD_SET_FSW(38),
		COMM_GPD_BUFFER_NOTIFY(39),
		COMM_GPD_BUFFER_SIZE_LEFT(40),
		COMM_GPD_FILL_BUFFER(41),
		COMM_GPD_OUTPUT_SAMPLE(42),
		COMM_GPD_SET_MODE(43),
		COMM_GPD_FILL_BUFFER_INT8(44),
		COMM_GPD_FILL_BUFFER_INT16(45),
		COMM_GPD_SET_BUFFER_INT_SCALE(46),
		COMM_GET_VALUES_SETUP(47),
		COMM_SET_MCCONF_TEMP(48),
		COMM_SET_MCCONF_TEMP_SETUP(49),
		COMM_GET_VALUES_SELECTIVE(50),
		COMM_GET_VALUES_SETUP_SELECTIVE(51),
		COMM_EXT_NRF_PRESENT(52),
		COMM_EXT_NRF_ESB_SET_CH_ADDR(53),
		COMM_EXT_NRF_ESB_SEND_DATA(54),
		COMM_EXT_NRF_ESB_RX_DATA(55),
		COMM_EXT_NRF_SET_ENABLED(56),
		COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP(57),
		COMM_DETECT_APPLY_ALL_FOC(58),
		COMM_JUMP_TO_BOOTLOADER_ALL_CAN(59),
		COMM_ERASE_NEW_APP_ALL_CAN(60),
		COMM_WRITE_NEW_APP_DATA_ALL_CAN(61),
		COMM_PING_CAN(62),
		COMM_APP_DISABLE_OUTPUT(63),
		COMM_TERMINAL_CMD_SYNC(64),
		COMM_GET_IMU_DATA(65)
    end

    methods (Static)
        
        function defaultValue = getDefaultValue()
            % GETDEFAULTVALUE  Returns the default enumerated value.
            %   If this method is not defined, the first enumeration is used.
            defaultValue = COMM_PACKET_ID.COMM_FW_VERSION;
        end

        function dScope = getDataScope()
            % GETDATASCOPE  Specifies whether the data type definition should be imported from,
            %               or exported to, a header file during code generation.
            dScope = 'Imported';
        end

        function desc = getDescription()
            % GETDESCRIPTION  Returns a description of the enumeration.
            desc = '';
        end
        
        function headerFile = getHeaderFile()
            % GETHEADERFILE  Specifies the name of a header file. 
            headerFile = 'datatypes.h';
        end
        
        function flag = addClassNameToEnumNames()
            % ADDCLASSNAMETOENUMNAMES  Indicate whether code generator applies the class name as a prefix
            %                          to the enumeration.
            flag = false;
        end

    end

end
