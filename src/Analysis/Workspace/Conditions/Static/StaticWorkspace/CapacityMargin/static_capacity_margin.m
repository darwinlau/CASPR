function inWorkspace = static_capacity_margin(dynamics,options)
    if(static_quadprog(dynamics,options) == 1)
        m = CapacityMarginMetric();
        temp_value = m.evaluate(dynamics);
        inWorkspace = temp_value*(temp_value>=0);
    else
        inWorkspace = 0;
    end
end