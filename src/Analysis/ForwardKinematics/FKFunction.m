classdef FKFunction < handle
    %IDFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected, GetAccess = protected)
        q_previous = []
        l_previous = []
    end
    
    methods (Abstract)
        f = compute(obj, len, len_prev_2, q_prev, q_d_prev, delta_t, kin_model);
    end
        
    methods (Static)
        function [errorVector, jacobian] = ComputeLengthErrorVector(q, l, model)
            model.update(q, zeros(model.numDofs,1), zeros(model.numDofs,1));
            errorVector = l - model.cableLengths;
            jacobian = - model.L;
        end
    end
end