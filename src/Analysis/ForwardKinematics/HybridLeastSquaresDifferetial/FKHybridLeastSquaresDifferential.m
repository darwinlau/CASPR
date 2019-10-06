% Solves the forward kinematics using a hybrid between differential and
% least squares. This is motivated by the acculumating error of
% differential method and the higher computational time of the least
% squares method.
%
% Author        : Darwin LAU
% Created       : 2015
% Description    : 
%   The method alternates between the differential and least squares
%   method. The interval denotes that the least squares should be run once
%   out of X times (for example, interval = 5 means that least squares
%   would be run once every 5 times)
classdef FKHybridLeastSquaresDifferential < FKAnalysisBase
    
    properties (Access = private)
        fkDifferential
        fkLeastSquares
        interval
        counter = 1;
    end
    
    methods
        function fkhybrid = FKHybridLeastSquaresDifferential(kin_model, approxType, qDotType, interval)
            CASPR_log.Assert(interval > 0 && rem(interval,1)==0, 'Frequency must be an integer greater than 0');
            
            fkhybrid@FKAnalysisBase(kin_model);
            fkhybrid.fkLeastSquares = FKLeastSquares(kin_model, approxType, qDotType);
            fkhybrid.fkDifferential = FKDifferential(kin_model);
            fkhybrid.interval = interval;
        end
        
        function [q, q_dot] = computeFunction(obj, len, len_prev_2, q_prev, q_d_prev, delta_t, cable_indices)            
            if (obj.counter < obj.interval)
                [q, q_dot] = obj.fkDifferential.compute(len, len_prev_2, q_prev, q_d_prev, delta_t, cable_indices);
                obj.counter = obj.counter + 1;
            else
                [q, q_dot] = obj.fkLeastSquares.compute(len, len_prev_2, q_prev, q_d_prev, delta_t, cable_indices);
                obj.counter = 1;
            end
        end     
    end
end

