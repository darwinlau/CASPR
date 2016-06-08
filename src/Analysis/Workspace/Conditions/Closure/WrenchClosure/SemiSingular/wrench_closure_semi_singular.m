% Semisingular implementation of wrench closure condition
% Author         : Jonathan EDEN
% Created        : 2016
% Description    : Implementation of wrench closure condition using the
% semisingular metric
function inWorkspace = wrench_closure_semi_singular(dynamics)
    m = SemiSingularMetric();
    inWorkspace = m.evaluate(dynamics,[],[],[])>0;
end