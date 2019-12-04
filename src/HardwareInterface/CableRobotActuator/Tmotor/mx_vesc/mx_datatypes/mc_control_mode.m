classdef mc_control_mode < Simulink.IntEnumType
    % MATLAB enumeration class definition generated from template
    
    enumeration
        CONTROL_MODE_DUTY(0),
		CONTROL_MODE_SPEED(1),
		CONTROL_MODE_CURRENT(2),
		CONTROL_MODE_CURRENT_BRAKE(3),
		CONTROL_MODE_POS(4),
		CONTROL_MODE_HANDBRAKE(5),
		CONTROL_MODE_OPENLOOP(6),
		CONTROL_MODE_OPENLOOP_PHASE(7),
		CONTROL_MODE_NONE(8)
    end

    methods (Static)
        
        function defaultValue = getDefaultValue()
            % GETDEFAULTVALUE  Returns the default enumerated value.
            %   If this method is not defined, the first enumeration is used.
            defaultValue = mc_control_mode.CONTROL_MODE_DUTY;
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
