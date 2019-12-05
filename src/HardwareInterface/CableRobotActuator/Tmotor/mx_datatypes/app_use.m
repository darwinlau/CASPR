classdef app_use < Simulink.IntEnumType
    % MATLAB enumeration class definition generated from template
    
    enumeration
        APP_NONE(0),
		APP_PPM(1),
		APP_ADC(2),
		APP_UART(3),
		APP_PPM_UART(4),
		APP_ADC_UART(5),
		APP_NUNCHUK(6),
		APP_NRF(7),
		APP_CUSTOM(8)
    end

    methods (Static)
        
        function defaultValue = getDefaultValue()
            % GETDEFAULTVALUE  Returns the default enumerated value.
            %   If this method is not defined, the first enumeration is used.
            defaultValue = app_use.APP_NONE;
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
