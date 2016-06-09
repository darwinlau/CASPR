% Library of cell operation utilities
%
% Author        : Darwin LAU
% Created       : 2014
% Description    :
classdef CellOperations
    
    methods (Static)
        % Creates a cell array
        function c = CreateCellArray(constructorFn, dim)
            c = cell(dim);
            for idx = 1:numel(c)
                c(idx) = {constructorFn()};
            end
        end
    end
    
end

