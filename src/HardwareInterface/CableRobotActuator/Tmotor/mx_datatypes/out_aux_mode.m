classdef out_aux_mode < Simulink.IntEnumType
    % MATLAB enumeration class definition generated from template
    
    enumeration
        OUT_AUX_MODE_OFF(0),
		OUT_AUX_MODE_ON_AFTER_2S(1),
		OUT_AUX_MODE_ON_AFTER_5S(2),
		OUT_AUX_MODE_ON_AFTER_10S(3)
    end

    methods (Static)
        
        function defaultValue = getDefaultValue()
            % GETDEFAULTVALUE  Returns the default enumerated value.
            %   If this method is not defined, the first enumeration is used.
            defaultValue = out_aux_mode.OUT_AUX_MODE_OFF;
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
