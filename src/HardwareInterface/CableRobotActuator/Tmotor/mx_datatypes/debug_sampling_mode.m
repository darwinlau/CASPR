classdef debug_sampling_mode < Simulink.IntEnumType
    % MATLAB enumeration class definition generated from template
    
    enumeration
        DEBUG_SAMPLING_OFF(0),
		DEBUG_SAMPLING_NOW(1),
		DEBUG_SAMPLING_START(2),
		DEBUG_SAMPLING_TRIGGER_START(3),
		DEBUG_SAMPLING_TRIGGER_FAULT(4),
		DEBUG_SAMPLING_TRIGGER_START_NOSEND(5),
		DEBUG_SAMPLING_TRIGGER_FAULT_NOSEND(6),
		DEBUG_SAMPLING_SEND_LAST_SAMPLES(7)
    end

    methods (Static)
        
        function defaultValue = getDefaultValue()
            % GETDEFAULTVALUE  Returns the default enumerated value.
            %   If this method is not defined, the first enumeration is used.
            defaultValue = debug_sampling_mode.DEBUG_SAMPLING_OFF;
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
