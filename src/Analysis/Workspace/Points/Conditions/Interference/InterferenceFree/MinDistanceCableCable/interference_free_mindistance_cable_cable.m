% Interference free workspace between cable and cable
% Use distance-based method without optimiazation
% Auther: Benji Zeqing Zhang
% Date:   7/2017
% Description: cal.the min distance between cable and cable, return 1 if
%              distance is not 0, otherwise return 1.
function inWorkspace = interference_free_mindistance_cable_cable(dynamics, epsilon_d)
    m = MinCableCableDistanceMetric();
    val = m.evaluate(dynamics);
    inWorkspace = (val > epsilon_d); % Minimum distance cables should be away from each other
end

