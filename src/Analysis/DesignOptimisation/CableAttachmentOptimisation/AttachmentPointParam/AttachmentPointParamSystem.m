% Representation for the attachments for the entire system. The system is
% comprised of the attachments for a cable, which contains the individual
% attachment points.
% 
% Author        : Darwin LAU
% Created       : 2016
% Description	:

classdef AttachmentPointParamSystem < handle    
    
    properties (SetAccess = private)
        cables      % A cell array of AttachmentPointParamCable objects
    end
    
    properties (Dependent)
        numVars     % Total number of variables to represent the systems' attachments
        numCables   % Total number of cables for the system
        x           % Total state variable of the system
        x_min       % Vector for minimum values of each state
        x_max       % Vector for maximum values of each state
    end
    
    methods
        function ap = AttachmentPointParamSystem(cables)
            ap.cables = cables;
        end
        
        % Updates the state for each cable
        function updateCableAttachments(obj, x) %, cablesKin, bodiesKin) % Not sure if it would be used in the future
            counter = 0;
            for i = 1:obj.numCables
                xi = x(counter+1:counter+obj.cables{i}.numVars);
                obj.cables{i}.updateCableAttachments(xi);
                counter = counter + obj.cables{i}.numVars;
            end
        end
        
        function value = get.numVars(obj)
            value = 0;
            for i = 1:length(obj.cables)
                value = value + obj.cables{i}.numVars;
            end
        end
        
        function value = get.numCables(obj)
            value = length(obj.cables);
        end
        
        function value = get.x(obj)
            counter = 0;
            value = zeros(obj.numVars, 1);
            for i = 1:length(obj.cables)
                value(counter+1:counter+obj.cables{i}.numVars) = obj.cables{i}.x;
                counter = counter + obj.cables{i}.numVars;
            end
        end
        
        function value = get.x_min(obj)
            counter = 0;
            value = zeros(obj.numVars, 1);
            for i = 1:length(obj.cables)
                value(counter+1:counter+obj.cables{i}.numVars) = obj.cables{i}.x_min;
                counter = counter + obj.cables{i}.numVars;
            end
        end
        
        function value = get.x_max(obj)
            counter = 0;
            value = zeros(obj.numVars, 1);
            for i = 1:length(obj.cables)
                value(counter+1:counter+obj.cables{i}.numVars) = obj.cables{i}.x_max;
                counter = counter + obj.cables{i}.numVars;
            end
        end
    end
end

