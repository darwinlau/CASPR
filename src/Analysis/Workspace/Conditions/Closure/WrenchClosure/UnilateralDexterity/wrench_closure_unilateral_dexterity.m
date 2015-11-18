function inWorkspace = wrench_closure_unilateral_dexterity(dynamics,options)
    m = UnilateralDexterityMetric();
    inWorkspace = m.evaluate(dynamics,options);
end