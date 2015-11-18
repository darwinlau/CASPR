function inWorkspace = wrench_closure_tension_factor(dynamics,options)
    m = TensionFactorMetric();
    inWorkspace = m.evaluate(dynamics,options);
end