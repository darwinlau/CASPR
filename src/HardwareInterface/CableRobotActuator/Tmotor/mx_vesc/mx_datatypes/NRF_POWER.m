classdef NRF_POWER < Simulink.IntEnumType
    % MATLAB enumeration class definition generated from template
    
    enumeration
        NRF_POWER_M18DBM(0),
		NRF_POWER_M12DBM(1),
		NRF_POWER_M6DBM(2),
		NRF_POWER_0DBM(3),
		NRF_POWER_OFF(4)
    end

    methods (Static)
        
        function defaultValue = getDefaultValue()
            % GETDEFAULTVALUE  Returns the default enumerated value.
            %   If this method is not defined, the first enumeration is used.
            defaultValue = NRF_POWER.NRF_POWER_M18DBM;
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
