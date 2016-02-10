function inWorkspace = wrench_feasible_capacity_margin(desired_wrench_set,dynamics)
    m = CapacityMarginMetric(desired_wrench_set);
    temp_value = m.evaluate(dynamics);
    inWorkspace = temp_value*(temp_value>=0);
end