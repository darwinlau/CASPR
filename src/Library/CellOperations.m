classdef CellOperations
    %CELLOPERATIONS Summary of this class goes here
    %   Detailed explanation goes here
    
    methods (Static)
        function c = CreateCellArray(constructorFn, dim)
            c = cell(dim);
            for idx = 1:numel(c)
                c(idx) = {constructorFn()};
            end
            %c = cellfun(constructorFn, empty_c);
            %c(:) = {constructorFn()};
        end
    end
    
end

