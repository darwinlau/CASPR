classdef NRF_RETR_DELAY < Simulink.IntEnumType
    % MATLAB enumeration class definition generated from template
    
    enumeration
        NRF_RETR_DELAY_250US(0),
		NRF_RETR_DELAY_500US(1),
		NRF_RETR_DELAY_750US(2),
		NRF_RETR_DELAY_1000US(3),
		NRF_RETR_DELAY_1250US(4),
		NRF_RETR_DELAY_1500US(5),
		NRF_RETR_DELAY_1750US(6),
		NRF_RETR_DELAY_2000US(7),
		NRF_RETR_DELAY_2250US(8),
		NRF_RETR_DELAY_2500US(9),
		NRF_RETR_DELAY_2750US(10),
		NRF_RETR_DELAY_3000US(11),
		NRF_RETR_DELAY_3250US(12),
		NRF_RETR_DELAY_3500US(13),
		NRF_RETR_DELAY_3750US(14),
		NRF_RETR_DELAY_4000US(15)
    end

    methods (Static)
        
        function defaultValue = getDefaultValue()
            % GETDEFAULTVALUE  Returns the default enumerated value.
            %   If this method is not defined, the first enumeration is used.
            defaultValue = NRF_RETR_DELAY.NRF_RETR_DELAY_250US;
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
