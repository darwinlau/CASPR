classdef ppm_control_type < Simulink.IntEnumType
    % MATLAB enumeration class definition generated from template
    
    enumeration
        PPM_CTRL_TYPE_NONE(0),
		PPM_CTRL_TYPE_CURRENT(1),
		PPM_CTRL_TYPE_CURRENT_NOREV(2),
		PPM_CTRL_TYPE_CURRENT_NOREV_BRAKE(3),
		PPM_CTRL_TYPE_DUTY(4),
		PPM_CTRL_TYPE_DUTY_NOREV(5),
		PPM_CTRL_TYPE_PID(6),
		PPM_CTRL_TYPE_PID_NOREV(7)
    end

    methods (Static)
        
        function defaultValue = getDefaultValue()
            % GETDEFAULTVALUE  Returns the default enumerated value.
            %   If this method is not defined, the first enumeration is used.
            defaultValue = ppm_control_type.PPM_CTRL_TYPE_NONE;
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
