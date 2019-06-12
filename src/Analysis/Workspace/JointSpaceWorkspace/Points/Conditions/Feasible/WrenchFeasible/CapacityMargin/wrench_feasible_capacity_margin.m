% Capacity margin implementation for evaluating WFW.
%
% Please cite the following paper when using this algorithm:
% A. L. Cruz-Ruizy, S. Caro, P. Cardou and F.Guay "ARACHNIS: Analysis
% of robots actuated by cables with handy and neat interface software", 
% in Proceedings of the Cable-Driven Paralle Robots, 2015.
% Author         : Jonathan EDEN
% Created        : 2016
% Description    : Implementation of wrench feasibility evaluation using the
% capacity margin
function inWorkspace = wrench_feasible_capacity_margin(desired_wrench_set,dynamics)
    m = CapacityMarginMetric(desired_wrench_set);
    val = m.evaluate(dynamics);
    inWorkspace = (val>=0);
end