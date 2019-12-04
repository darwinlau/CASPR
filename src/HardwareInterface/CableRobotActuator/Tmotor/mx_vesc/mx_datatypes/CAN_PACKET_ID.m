classdef CAN_PACKET_ID < Simulink.IntEnumType
    % MATLAB enumeration class definition generated from template
    
    enumeration
        CAN_PACKET_SET_DUTY(0),
		CAN_PACKET_SET_CURRENT(1),
		CAN_PACKET_SET_CURRENT_BRAKE(2),
		CAN_PACKET_SET_RPM(3),
		CAN_PACKET_SET_POS(4),
		CAN_PACKET_FILL_RX_BUFFER(5),
		CAN_PACKET_FILL_RX_BUFFER_LONG(6),
		CAN_PACKET_PROCESS_RX_BUFFER(7),
		CAN_PACKET_PROCESS_SHORT_BUFFER(8),
		CAN_PACKET_STATUS(9),
		CAN_PACKET_SET_CURRENT_REL(10),
		CAN_PACKET_SET_CURRENT_BRAKE_REL(11),
		CAN_PACKET_SET_CURRENT_HANDBRAKE(12),
		CAN_PACKET_SET_CURRENT_HANDBRAKE_REL(13),
		CAN_PACKET_STATUS_2(14),
		CAN_PACKET_STATUS_3(15),
		CAN_PACKET_STATUS_4(16),
		CAN_PACKET_PING(17),
		CAN_PACKET_PONG(18),
		CAN_PACKET_DETECT_APPLY_ALL_FOC(19),
		CAN_PACKET_DETECT_APPLY_ALL_FOC_RES(20),
		CAN_PACKET_CONF_CURRENT_LIMITS(21),
		CAN_PACKET_CONF_STORE_CURRENT_LIMITS(22),
		CAN_PACKET_CONF_CURRENT_LIMITS_IN(23),
		CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN(24),
		CAN_PACKET_CONF_FOC_ERPMS(25),
		CAN_PACKET_CONF_STORE_FOC_ERPMS(26)
    end

    methods (Static)
        
        function defaultValue = getDefaultValue()
            % GETDEFAULTVALUE  Returns the default enumerated value.
            %   If this method is not defined, the first enumeration is used.
            defaultValue = CAN_PACKET_ID.CAN_PACKET_SET_DUTY;
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
