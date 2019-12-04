classdef drv8301_oc_mode < Simulink.IntEnumType
    % MATLAB enumeration class definition generated from template
    
    enumeration
        DRV8301_OC_LIMIT(0),
		DRV8301_OC_LATCH_SHUTDOWN(1),
		DRV8301_OC_REPORT_ONLY(2),
		DRV8301_OC_DISABLED(3)
    end

    methods (Static)
        
        function defaultValue = getDefaultValue()
            % GETDEFAULTVALUE  Returns the default enumerated value.
            %   If this method is not defined, the first enumeration is used.
            defaultValue = drv8301_oc_mode.DRV8301_OC_LIMIT;
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
