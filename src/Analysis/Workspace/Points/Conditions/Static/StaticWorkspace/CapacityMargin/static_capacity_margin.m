% Capacity margin implementation for evaluating static workspace.
%
% Please cite the following paper when using this algorithm:
% A. L. Cruz-Ruizy, S. Caro, P. Cardou and F.Guay "ARACHNIS: Analysis
% of robots actuated by cables with handy and neat interface software", 
% in Proceedings of the Cable-Driven Paralle Robots, 2015.
% Author         : Jonathan EDEN
% Created        : 2016
% Description    : Implementation of static workspace analysis using the
% capacity margin
function inWorkspace = static_capacity_margin(dynamics)
    m = CapacityMarginMetric(dynamics.G);
    temp_value = m.evaluate(dynamics);
    inWorkspace = temp_value>=0;
end