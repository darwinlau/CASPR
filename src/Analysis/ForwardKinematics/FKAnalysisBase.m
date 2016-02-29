classdef FKAnalysisBase < handle
    %IDFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        model;
    end
    
    properties (SetAccess = protected, GetAccess = protected)
        q_previous = []
        l_previous = []
    end
    
    methods
        function fk = FKAnalysisBase(kin_model)
            fk.model = kin_model;
        end
        
        function [q, q_dot, comp_time] = compute(obj, len, len_prev_2, q_prev, q_d_prev, delta_t)
            start_tic = tic;
            [q, q_dot] = obj.computeFunction(len, len_prev_2, q_prev, q_d_prev, delta_t);
            comp_time = toc(start_tic);
        end
    end
    
    methods (Abstract)
        [q,q_dot] = computeFunction(obj, len, len_prev_2, q_prev, q_d_prev, delta_t);
    end
        
    methods (Static)
        function [errorVector, jacobian] = ComputeLengthErrorVector(q, l, model)
            model.update(q, zeros(model.numDofs,1), zeros(model.numDofs,1));
            errorVector = l - model.cableLengths;
            jacobian = - model.L;
        end
    end
end