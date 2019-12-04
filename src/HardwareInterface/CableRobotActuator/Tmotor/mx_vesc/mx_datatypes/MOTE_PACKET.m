classdef MOTE_PACKET < Simulink.IntEnumType
    % MATLAB enumeration class definition generated from template
    
    enumeration
        MOTE_PACKET_BATT_LEVEL(0),
		MOTE_PACKET_BUTTONS(1),
		MOTE_PACKET_ALIVE(2),
		MOTE_PACKET_FILL_RX_BUFFER(3),
		MOTE_PACKET_FILL_RX_BUFFER_LONG(4),
		MOTE_PACKET_PROCESS_RX_BUFFER(5),
		MOTE_PACKET_PROCESS_SHORT_BUFFER(6),
		MOTE_PACKET_PAIRING_INFO(7)
    end

    methods (Static)
        
        function defaultValue = getDefaultValue()
            % GETDEFAULTVALUE  Returns the default enumerated value.
            %   If this method is not defined, the first enumeration is used.
            defaultValue = MOTE_PACKET.MOTE_PACKET_BATT_LEVEL;
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
