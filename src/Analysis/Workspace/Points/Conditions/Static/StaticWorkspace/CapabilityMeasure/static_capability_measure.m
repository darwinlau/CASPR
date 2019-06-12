% Static equilibrium acceleration capability measure implementation for
% evaluating static workspace.
% Author         : Jonathan EDEN
% Created        : 2016
% Description    : Implementation of static workspace analysis using the
% SEACM metric
function inWorkspace = static_capability_measure(dynamics)
    if(static_quadprog(dynamics) == 1)
        m = SEACMetric();
        value = m.evaluate(dynamics);
        inWorkspace = value>=0;
    else
        inWorkspace = false;
    end
end