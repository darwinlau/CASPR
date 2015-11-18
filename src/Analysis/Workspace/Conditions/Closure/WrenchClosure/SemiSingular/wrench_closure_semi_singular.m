function inWorkspace = wrench_closure_semi_singular(dynamics)
    m = SemiSingularMetric();
    inWorkspace = m.evaluate(dynamics);
end