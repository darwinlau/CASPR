classdef adc_control_type < Simulink.IntEnumType
    % MATLAB enumeration class definition generated from template
    
    enumeration
        ADC_CTRL_TYPE_NONE(0),
		ADC_CTRL_TYPE_CURRENT(1),
		ADC_CTRL_TYPE_CURRENT_REV_CENTER(2),
		ADC_CTRL_TYPE_CURRENT_REV_BUTTON(3),
		ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC(4),
		ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER(5),
		ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON(6),
		ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC(7),
		ADC_CTRL_TYPE_DUTY(8),
		ADC_CTRL_TYPE_DUTY_REV_CENTER(9),
		ADC_CTRL_TYPE_DUTY_REV_BUTTON(10),
		ADC_CTRL_TYPE_PID(11),
		ADC_CTRL_TYPE_PID_REV_CENTER(12),
		ADC_CTRL_TYPE_PID_REV_BUTTON(13)
    end

    methods (Static)
        
        function defaultValue = getDefaultValue()
            % GETDEFAULTVALUE  Returns the default enumerated value.
            %   If this method is not defined, the first enumeration is used.
            defaultValue = adc_control_type.ADC_CTRL_TYPE_NONE;
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
