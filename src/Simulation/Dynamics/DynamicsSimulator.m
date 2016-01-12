classdef DynamicsSimulator < MotionSimulator
    %DYNAMICSSIMULATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        cableForces         % cell array of cable force vector
        interactionWrench   % cell array of the interaction wrench vector
    end
    
    methods        
        function ds = DynamicsSimulator(model)
            ds@MotionSimulator(model);
        end
        
        function plotCableForces(obj, cables_to_plot,figure_handles)
            assert(~isempty(obj.cableForces), 'Cannot plot since cableForces is empty');
            
            if nargin == 1 || isempty(cables_to_plot)
                cables_to_plot = 1:obj.model.numCables;
            end
            
            forces = cell2mat(obj.cableForces);
            
            if(nargin == 3)
                figure(figure_handles); cla;
            else
                figure;
            end
            plot(obj.timeVector, forces(cables_to_plot, :), 'LineWidth', 1.5, 'Color', 'k'); 
            title('Cable forces'); 
            
%             line_color_order = [1 0 0; 0 1 0; 0 0 1; 0 1 1; 1 0 1; 1 1 0; 0 0 0; 1 165/255 0; 139/255 69/255 19/255];
%             line_style_order = '-|--|:|-.';
%             gcf = figure;
%             set(gcf, 'DefaultAxesColorOrder', line_color_order, 'DefaultAxesLineStyleOrder', line_style_order);
%             plot(obj.timeVector, forces(cables_to_plot, :), 'LineWidth', 1.5); title('Cable forces');
        end
        
        function plotInteractionForceMagnitudes(obj, links_to_plot,figure_handles)
            assert(~isempty(obj.interactionWrench), 'Cannot plot since interactionWrench is empty');
            
            if nargin == 1 || isempty(links_to_plot)
                links_to_plot = 1:obj.model.numLinks;
            end
            
            wrenches = cell2mat(obj.interactionWrench);
            forcesMag = zeros(length(links_to_plot), length(obj.timeVector));
            
            % Forces are the first 3 components of every link
            for t = 1:length(obj.timeVector)
                vector = wrenches(:, t);
                for k = 1:length(links_to_plot)
                    link = links_to_plot(k);
                    forcesMag(link, t) = norm(vector(6*link-5:6*link-3),2);
                end
            end
            
            if(nargin == 3)
                figure(figure_handles); cla;
            else
                figure;
            end
            plot(obj.timeVector, forcesMag, 'LineWidth', 1.5, 'Color', 'k');
            title('Magnitude of interaction forces'); 
            
%             line_color_order = [1 0 0; 0 1 0; 0 0 1; 0 1 1; 1 0 1; 1 1 0; 0 0 0; 1 165/255 0; 139/255 69/255 19/255];
%             line_style_order = '-|--|:|-.';
%             gcf = figure;
%             set(gcf, 'DefaultAxesColorOrder', line_color_order, 'DefaultAxesLineStyleOrder', line_style_order);
%             plot(obj.timeVector, forces(links_to_plot, :), 'LineWidth', 1.5); title('Cable forces');
        end        
        
        % Plots the interaction angle into the Z axis
        function plotInteractionForceAngles(obj, links_to_plot)
            assert(~isempty(obj.interactionWrench), 'Cannot plot since interactionWrench is empty');
            
            if nargin == 1 || isempty(links_to_plot)
                links_to_plot = 1:obj.model.numLinks;
            end
            
            wrenches = cell2mat(obj.interactionWrench);
            angles = zeros(length(links_to_plot), length(obj.timeVector));
            
            % Forces are the first 3 components of every link
            for t = 1:length(obj.timeVector)
                vector = wrenches(:, t);
                for k = 1:length(links_to_plot)
                    link = links_to_plot(k);
                    intForce = vector(6*link-5:6*link-3);
                    angles(k, t) = atan(norm(intForce(1:2),2)/abs(intForce(3)))*180/pi;
                end
            end
            
            figure;
            plot(obj.timeVector, angles, 'LineWidth', 1.5, 'Color', 'k');
            title('Interaction angles');           
        end
        
        function plotInteractionMomentMagnitudes(obj, links_to_plot)
            assert(~isempty(obj.interactionWrench), 'Cannot plot since interactionWrench is empty');
            
            if nargin == 1 || isempty(links_to_plot)
                links_to_plot = 1:obj.model.numLinks;
            end
            
            wrenches = cell2mat(obj.interactionWrench);
            momentsMag = zeros(length(links_to_plot), length(obj.timeVector));
            
            % Forces are the first 3 components of every link
            for t = 1:length(obj.timeVector)
                vector = wrenches(:, t);
                for k = 1:length(links_to_plot)
                    link = links_to_plot(k);
                    momentsMag(link, t) = norm(vector(6*link-2:6*link),2);
                end
            end
            
            figure;
            plot(obj.timeVector, momentsMag, 'LineWidth', 1.5, 'Color', 'k');
            title('Magnitude of interaction moments'); 
        end
        
        function plotInteractionForceZ(obj, links_to_plot)
            assert(~isempty(obj.interactionWrench), 'Cannot plot since interactionWrench is empty');
            
            if nargin == 1 || isempty(links_to_plot)
                links_to_plot = 1:obj.model.numLinks;
            end
            
            wrenches = cell2mat(obj.interactionWrench);
            forcesZ = zeros(length(links_to_plot), length(obj.timeVector));
            
            % Forces are the first 3 components of every link
            for t = 1:length(obj.timeVector)
                vector = wrenches(:, t);
                for k = 1:length(links_to_plot)
                    link = links_to_plot(k);
                    forcesZ(link, t) = vector(6*link-3);
                end
            end
            
            figure;
            plot(obj.timeVector, forcesZ, 'LineWidth', 1.5, 'Color', 'k');
            title('Interaction forces (Z component)'); 
        end
    end
    
end

