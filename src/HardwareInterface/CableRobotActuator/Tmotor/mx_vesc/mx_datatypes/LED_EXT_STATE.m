classdef LED_EXT_STATE < Simulink.IntEnumType
    % MATLAB enumeration class definition generated from template
    
    enumeration
        LED_EXT_OFF(0),
		LED_EXT_NORMAL(1),
		LED_EXT_BRAKE(2),
		LED_EXT_TURN_LEFT(3),
		LED_EXT_TURN_RIGHT(4),
		LED_EXT_BRAKE_TURN_LEFT(5),
		LED_EXT_BRAKE_TURN_RIGHT(6),
		LED_EXT_BATT(7)
    end

    methods (Static)
        
        function defaultValue = getDefaultValue()
            % GETDEFAULTVALUE  Returns the default enumerated value.
            %   If this method is not defined, the first enumeration is used.
            defaultValue = LED_EXT_STATE.LED_EXT_OFF;
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
