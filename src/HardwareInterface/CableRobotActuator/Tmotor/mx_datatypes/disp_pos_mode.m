classdef disp_pos_mode < Simulink.IntEnumType
    % MATLAB enumeration class definition generated from template
    
    enumeration
        DISP_POS_MODE_NONE(0),
		DISP_POS_MODE_INDUCTANCE(1),
		DISP_POS_MODE_OBSERVER(2),
		DISP_POS_MODE_ENCODER(3),
		DISP_POS_MODE_PID_POS(4),
		DISP_POS_MODE_PID_POS_ERROR(5),
		DISP_POS_MODE_ENCODER_OBSERVER_ERROR(6)
    end

    methods (Static)
        
        function defaultValue = getDefaultValue()
            % GETDEFAULTVALUE  Returns the default enumerated value.
            %   If this method is not defined, the first enumeration is used.
            defaultValue = disp_pos_mode.DISP_POS_MODE_NONE;
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
