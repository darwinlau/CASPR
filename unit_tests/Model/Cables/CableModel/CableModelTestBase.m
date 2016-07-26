% Testing of cable model base
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
classdef (Abstract) CableModelTestBase < matlab.unittest.TestCase
    % All cable models tests should include a test of the update and
    % dependent variable functions
    methods (Abstract, Test)        
        testUpdate(testCase)
        testLength(testCase)
        testK(testCase)
    end
    
    methods 
        function assertPositiveCableLengths(testCase,l)
            assert(l>=0,'cable length must be positive')
        end
    end
end