function inWorkspace = static_capability_measure(dynamics,options)
    if(static_quadprog(dynamics,options) == 1)
        m = SEACM();
        temp_value = m.evaluate(dynamics,[],[],[]);
        inWorkspace = temp_value*(temp_value>=0);
    else
        inWorkspace = false;
    end
end