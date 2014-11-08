classdef DynamicsSimulator < MotionSimulator
    %DYNAMICSSIMULATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods        
        function plotCableForces(obj, cables_to_plot)
            assert(~isempty(obj.states), 'State dynamics cell array is empty and nothing to plot.');
            if nargin == 1 || isempty(cables_to_plot)
                cables_to_plot = 1:obj.states{1}.numCables;
            end
            
            forces = zeros(obj.states{1}.numCables, length(obj.timeVector));
            
            for t = 1:length(obj.timeVector)
                forces(:, t) = obj.states{t}.cableDynamics.forces;
            end
            
            figure; plot(obj.timeVector, forces(cables_to_plot, :), 'LineWidth', 1.5, 'Color', 'k'); title('Cable forces'); 
            
%             line_color_order = [1 0 0; 0 1 0; 0 0 1; 0 1 1; 1 0 1; 1 1 0; 0 0 0; 1 165/255 0; 139/255 69/255 19/255];
%             line_style_order = '-|--|:|-.';
%             gcf = figure;
%             set(gcf, 'DefaultAxesColorOrder', line_color_order, 'DefaultAxesLineStyleOrder', line_style_order);
%             plot(obj.timeVector, forces(cables_to_plot, :), 'LineWidth', 1.5); title('Cable forces');
        end
        
        function plotInteractionForceMagnitudes(obj, links_to_plot)
            assert(~isempty(obj.states), 'State dynamics cell array is empty and nothing to plot.');
            if nargin == 1 || isempty(links_to_plot)
                links_to_plot = 1:obj.states{1}.numLinks;
            end
            
            forces = zeros(obj.states{1}.numLinks, length(obj.timeVector));
            
            for t = 1:length(obj.timeVector)
                forces(:, t) = obj.states{t}.jointForceMagnitudes;
            end
            
            figure; plot(obj.timeVector, forces(links_to_plot, :), 'LineWidth', 1.5, 'Color', 'k');  title('Interaction forces'); 
            %figure; plot(obj.timeVector, sum(forces.^2, 1), 'LineWidth', 1.5, 'Color', 'k');  title('Interaction forces'); 
            
%             line_color_order = [1 0 0; 0 1 0; 0 0 1; 0 1 1; 1 0 1; 1 1 0; 0 0 0; 1 165/255 0; 139/255 69/255 19/255];
%             line_style_order = '-|--|:|-.';
%             gcf = figure;
%             set(gcf, 'DefaultAxesColorOrder', line_color_order, 'DefaultAxesLineStyleOrder', line_style_order);
%             plot(obj.timeVector, forces(links_to_plot, :), 'LineWidth', 1.5); title('Cable forces');
        end        
        
        function plotInteractionForceAngles(obj, links_to_plot)
            assert(~isempty(obj.states), 'State dynamics cell array is empty and nothing to plot.');
            if nargin == 1 || isempty(links_to_plot)
                links_to_plot = 1:obj.states{1}.numLinks;
            end
            
            angles = zeros(obj.states{1}.numLinks, length(obj.timeVector));
            
            for t = 1:length(obj.timeVector)
                angles(:, t) = obj.states{t}.jointForceAngles;
            end
            
            figure; plot(obj.timeVector, angles(links_to_plot, :), 'LineWidth', 1.5, 'Color', 'k');  title('Interaction angles'); 
            
%             line_color_order = [1 0 0; 0 1 0; 0 0 1; 0 1 1; 1 0 1; 1 1 0; 0 0 0; 1 165/255 0; 139/255 69/255 19/255];
%             line_style_order = '-|--|:|-.';
%             gcf = figure;
%             set(gcf, 'DefaultAxesColorOrder', line_color_order, 'DefaultAxesLineStyleOrder', line_style_order);
%             plot(obj.timeVector, angles(links_to_plot, :), 'LineWidth', 1.5); title('Cable forces');
        end
        
        function plotInteractionMomentMagnitudes(obj, links_to_plot)
            assert(~isempty(obj.states), 'State dynamics cell array is empty and nothing to plot.');
            
            if nargin == 1 || isempty(links_to_plot)
                links_to_plot = 1:obj.states{1}.numLinks;
            end
            
            forces = zeros(obj.states{1}.numLinks, length(obj.timeVector));
            
            for t = 1:length(obj.timeVector)
                forces(:, t) = obj.states{t}.jointMomentMagnitudes;
            end
            
            figure; plot(obj.timeVector, forces(links_to_plot, :), 'LineWidth', 1.5, 'Color', 'k');  title('Interaction moments'); 
        end
        
        function plotInteractionZForce(obj, links_to_plot)
            assert(~isempty(obj.states), 'State dynamics cell array is empty and nothing to plot.');
            
            if nargin == 1 || isempty(links_to_plot)
                links_to_plot = 1:obj.states{1}.numLinks;
            end
            
            forces = zeros(obj.states{1}.numLinks, length(obj.timeVector));
            
            for t = 1:length(obj.timeVector)
                forces(:, t) = obj.states{t}.jointWrenches(6*links_to_plot-3);
            end
            
            figure; plot(obj.timeVector, forces(links_to_plot, :), 'LineWidth', 1.5, 'Color', 'k');  title('Interaction forces Z'); 
        end
    end
    
end

