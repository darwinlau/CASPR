classdef mc_fault_code < Simulink.IntEnumType
    % MATLAB enumeration class definition generated from template
    
    enumeration
        FAULT_CODE_NONE(0),
		FAULT_CODE_OVER_VOLTAGE(1),
		FAULT_CODE_UNDER_VOLTAGE(2),
		FAULT_CODE_DRV(3),
		FAULT_CODE_ABS_OVER_CURRENT(4),
		FAULT_CODE_OVER_TEMP_FET(5),
		FAULT_CODE_OVER_TEMP_MOTOR(6),
		FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE(7),
		FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE(8),
		FAULT_CODE_MCU_UNDER_VOLTAGE(9),
		FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET(10),
		FAULT_CODE_ENCODER(11)
    end

    methods (Static)
        
        function defaultValue = getDefaultValue()
            % GETDEFAULTVALUE  Returns the default enumerated value.
            %   If this method is not defined, the first enumeration is used.
            defaultValue = mc_fault_code.FAULT_CODE_NONE;
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
