% Library of utilities for unit tests
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
classdef UnitTestOperations
    methods (Static)
        % This function is used to create structures for class setup
        % parameters. It can use a given enumeration class and provides the
        % necessary structure format.
        function output_struct = createTestClassSetupParameter(enumeration_string)
            [type_list,type_string] = enumeration(enumeration_string);
            output_struct = struct(type_string{1},type_list(1));
            for i = 2:length(type_list)
                output_struct = setfield(output_struct,type_string{i},type_list(i));
            end
        end
    end
end

