classdef (Abstract) SystemDynamicsCables < handle
    %CABLESYSTEMKINEMATICS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    properties (SetAccess = private)
    end
    
    properties (SetAccess = protected)
        cables = {};                 % cell array of CableDynamics object
        numCables = 0;
    end
    
    properties (Dependent)
        forces                      % vector of forces from cables
        forcesMin                   % vector of min forces from cables
        forcesMax                   % vector of max forces from cables
        forcesInvalid               % vector of invalid forces
    end
    
    methods
        function ck = SystemDynamicsCables(numCables)
            ck.numCables = numCables;
            for i = 1:numCables
                ck.cables{i} = CableDynamics(sprintf('Cable %d', i));
            end
        end
        
        function update(obj, cableKinematics, bodyKinematics, bodyDynamics)
        end
       
        function value = get.forces(obj)
            value = zeros(obj.numCables, 1);
            for i = 1:obj.numCables
                value(i) = obj.cables{i}.force;
            end
        end
        
        function set.forces(obj, f)
            assert(length(f) == obj.numCables, 'Forces dimension inconsistent with the number of cables');
            
            for i = 1:obj.numCables
                obj.cables{i}.force = f(i);
            end
        end
        
        function value = get.forcesMin(obj)
            value = zeros(obj.numCables, 1);
            for i = 1:obj.numCables
                value(i) = obj.cables{i}.forceMin;
            end
        end
        
        function value = get.forcesMax(obj)
            value = zeros(obj.numCables, 1);
            for i = 1:obj.numCables
                value(i) = obj.cables{i}.forceMax;
            end
        end
        
        function value = get.forcesInvalid(obj)
            value = zeros(obj.numCables, 1);
            for i = 1:obj.numCables
                value(i) = obj.cables{i}.forceInvalid;
            end
        end
    end
end

