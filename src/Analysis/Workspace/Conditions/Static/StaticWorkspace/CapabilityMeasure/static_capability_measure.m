function inWorkspace = static_capability_measure(dynamics)
    m = SEACM();
    temp_value = m.evaluate(dynamics);
    inWorkspace = temp_value*(temp_value>0);
end