function inWorkspace = static_capacity_margin(dynamics,options)
    m = CapacityMarginMetric(dynamics.G);
    temp_value = m.evaluate(dynamics);
    inWorkspace = temp_value*(temp_value>=0);
end