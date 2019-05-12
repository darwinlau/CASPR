% Tension factor implementation for evaluating WCW.
%
% Please cite the following paper when using this algorithm:
% C.B Pham, S.H Yeo, G. Yang and I. Chen, "Workspace analysis of fully
% restrained cable-driven manipulators", Robotics and Autonomous Systems, 
% vol. 57, no. 9, pp. 901-912, 2009.
% Author         : Jonathan EDEN
% Created        : 2016
% Description    : Implementation of WCW analysis using the tension factor
function inWorkspace = wrench_closure_tension_factor(dynamics)
    m = TensionFactorMetric();
    inWorkspace = m.evaluate(dynamics)>0;
end