classdef SystemDynamicsCables < handle
    %CABLESYSTEMKINEMATICS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    properties (SetAccess = private)
    end
    
    properties (SetAccess = protected)
        cables = {};                 % cell array of CableDynamics object
    end
    
    properties (Dependent)
        forces                      % vector of forces from cables
        forcesMin                   % vector of min forces from cables
        forcesMax                   % vector of max forces from cables
        forcesInvalid               % vector of invalid forces
        numCables;
    end
    
    methods
        function cd = SystemDynamicsCables(cables)
            cd.cables = cables;
        end
        
        function update(obj, cableKinematics, bodyKinematics)
            for k = 1:obj.numCables
                obj.cables{k}.update(cableKinematics, bodyKinematics);
            end
        end
        
        function sim_update(obj, cableKinematics, bodyKinematics)
            for k = 1:obj.numCables
                obj.cables{k}.update(cableKinematics, bodyKinematics);
            end
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
        
        function value = get.numCables(obj)
            value = length(obj.cables);
        end
    end
    
    methods (Static)
        function c = LoadXmlObj(cable_prop_xmlobj)
            rootNode = cable_prop_xmlobj.getDocumentElement;
            assert(strcmp(rootNode.getNodeName, 'cables'), 'Root elemnt should be <cables>');
            allCableItems = rootNode.getChildNodes;
                        
            num_cables = allCableItems.getLength;
            xml_cables = cell(1,num_cables);
            
            % Creates all of the links first
            for k = 1:num_cables
                % Java uses 0 indexing
                currentCableItem = allCableItems.item(k-1);
                
                type = char(currentCableItem.getNodeName);
                if (strcmp(type, 'cable_ideal'))
                    xml_cables{k} = CableDynamicsIdeal.LoadXmlObj(currentCableItem);
                else
                    error('Unknown cables type: %s', type);
                end
            end
            
            % Create the actual object to return
            c = SystemDynamicsCables(xml_cables);
        end
    end
end

