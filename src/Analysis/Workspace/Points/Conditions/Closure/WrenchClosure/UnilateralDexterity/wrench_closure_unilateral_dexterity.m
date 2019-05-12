% Unilateral dexterity implementation for evaluating WCW.
%
% Please cite the following paper when using this algorithm:
% R. Kurtz and V. Hayward. "Dexterity measures with unilateral actuation 
% constraints: the n+ 1 case." Advanced robotics 9.5 (1994): 561-577.
% Author         : Jonathan EDEN
% Created        : 2016
% Description    : Implementation of WCW analysis using the unilateral
% dexterity
% Note           : This metric has been extended to consider the case of
% greater than n+1 elements.
function inWorkspace = wrench_closure_unilateral_dexterity(dynamics)
    m = UnilateralDexterityMetric();
    inWorkspace = m.evaluate(dynamics)>0;
end