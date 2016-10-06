% Will make abstract later
classdef (Abstract) ExperimentBase < handle
    properties (SetAccess = protected)
        hardwareInterface;
        model;
    end
    
    methods
        function eb = ExperimentBase(hw_interface, model)
            eb.hardwareInterface = hw_interface;
            eb.model = model;
        end
        
        function openHardwareInterface(obj)
            obj.hardwareInterface.open();
        end
                
        function closeHardwareInterface(obj)
            obj.hardwareInterface.close();
        end
    end
end

