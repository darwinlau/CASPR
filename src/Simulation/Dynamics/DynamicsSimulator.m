% Base class for different simulators that deal with the study of the
% dynamics of CDPRs. 
%
% Author        : Darwin LAU
% Created       : 2013
% Description    :
%   Dynamics simulator is used in the simulation of ID, FD and control. The
%   additional class provides more plotting functionality, such as:
%       - plotCableForces
%       - plotInteractionForceMagnitudes
%       - plotInteractionForceAngles
%       - plotInteractionMomentMagnitudes
%       - plotInteractionForceZ
classdef DynamicsSimulator < MotionSimulatorBase
    
    properties
        cableForcesActive   % cell array of cable force vector (active)
        cableIndicesActive  % cell array of cable indices for active cables
        cableForces         % cell array of cable force vector (all)
        interactionWrench   % cell array of the interaction wrench vector
    end
    
    methods   
        % Constructor fo the dynamics simulator
        function ds = DynamicsSimulator(model)
            ds@MotionSimulatorBase(model);
        end
        
        % Plots the cable forces of the CDPR over the trajectory. Users
        % need to specify the plot axis and also which cables (as an array
        % of numbers) to plot (it is possible to default to plot all cables
        % if the array is []).
        function plotCableForces(obj, cables_to_plot, plot_axis)
            CASPR_log.Assert(~isempty(obj.cableForces), 'Cannot plot since cableForces is empty');
            
            if nargin <= 1 || isempty(cables_to_plot)
                cables_to_plot = 1:obj.model.numCables;
            end
            
            forces = cell2mat(obj.cableForces);
            
            valid_forces_ind = find(forces(1,:) ~= CableModelBase.INVALID_FORCE);
            valid_forces_ind_inv = setdiff(1:length(obj.timeVector), valid_forces_ind);
                
            if nargin <= 2 || isempty(plot_axis)
                figure;
                hold on;
                plot(obj.timeVector(valid_forces_ind), forces(cables_to_plot, valid_forces_ind), '.', 'LineWidth', 1.5, 'Color', 'k'); 
                plot(obj.timeVector(valid_forces_ind_inv), forces(cables_to_plot, valid_forces_ind_inv), '.', 'LineWidth', 1.5, 'Color', 'r'); 
                hold off;
                title('Cable forces');                 
            else
                hold on;
                plot(plot_axis,obj.timeVector(valid_forces_ind), forces(cables_to_plot, valid_forces_ind), '.', 'LineWidth', 1.5, 'Color', 'k'); 
                plot(plot_axis,obj.timeVector(valid_forces_ind_inv), forces(cables_to_plot, valid_forces_ind_inv), '.', 'LineWidth', 1.5, 'Color', 'r'); 
                hold off;                
            end
            xlabel('Time (seconds)')
            ylabel('Force (N)');
        end
        
        % Plots the magnitude of interaction forces trajectory. Users
        % need to specify the plot axis and also which links (as an array
        % of numbers) to plot (it is possible to default to plot all links
        % if the array is []).
        function plotInteractionForceMagnitudes(obj, links_to_plot, plot_axis)
            CASPR_log.Assert(~isempty(obj.interactionWrench), 'Cannot plot since interactionWrench is empty');
            
            if nargin <= 1 || isempty(links_to_plot)
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
            
            if nargin <= 2 || isempty(plot_axis)
                figure;
                plot(obj.timeVector, forcesMag, 'LineWidth', 1.5, 'Color', 'k');
                title('Magnitude of interaction forces');
            else
                plot(plot_axis,obj.timeVector, forcesMag, 'LineWidth', 1.5, 'Color', 'k');
            end
            xlabel('Time (seconds)')
            ylabel('Force (N)');
        end        
        
        % Plots the interaction angle into the Z axis.
        % TODO: This function needs to be made generic since not all 
        % interaction angles want to be measured with reference to the Z 
        % axis only. Can refer to IDConstraintInteractionForceAngleCone 
        % to incorporate the F_centre parameter.
        function plotInteractionForceAngles(obj, links_to_plot, plot_axis)
            CASPR_log.Assert(~isempty(obj.interactionWrench), 'Cannot plot since interactionWrench is empty');
            
            if nargin <= 1 || isempty(links_to_plot)
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
            
            if nargin <= 2 || isempty(plot_axis)
                figure;
                plot(obj.timeVector, angles, 'LineWidth', 1.5, 'Color', 'k');
                title('Interaction angles');
            else
                plot(plot_axis,obj.timeVector, angles, 'LineWidth', 1.5, 'Color', 'k');
            end
        end
        
        % Plots the magnitude of interaction moments trajectory. Users
        % need to specify the plot axis and also which links (as an array
        % of numbers) to plot (it is possible to default to plot all links
        % if the array is []).
        function plotInteractionMomentMagnitudes(obj, links_to_plot, plot_axis)
            CASPR_log.Assert(~isempty(obj.interactionWrench), 'Cannot plot since interactionWrench is empty');
            
            if nargin <= 1 || isempty(links_to_plot)
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
            
            if nargin <= 2 || isempty(plot_axis)
                figure;
                plot(obj.timeVector, momentsMag, 'LineWidth', 1.5, 'Color', 'k');
                title('Magnitude of interaction moments'); 
            else
                plot(plot_axis, obj.timeVector, momentsMag, 'LineWidth', 1.5, 'Color', 'k');
            end
        end
        
        % Plots the magnitude of Z dir interaction force trajectory. Users
        % need to specify the plot axis and also which links (as an array
        % of numbers) to plot (it is possible to default to plot all links
        % if the array is []).
        function plotInteractionForceZ(obj, links_to_plot, plot_axis)
            CASPR_log.Assert(~isempty(obj.interactionWrench), 'Cannot plot since interactionWrench is empty');
            
            if nargin <= 1 || isempty(links_to_plot)
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
            
            if nargin <= 2 || isempty(plot_axis)
                figure;
                plot(obj.timeVector, forcesZ, 'LineWidth', 1.5, 'Color', 'k');
                title('Interaction forces (Z component)'); 
            else
                plot(plot_axis, obj.timeVector, forcesZ, 'LineWidth', 1.5, 'Color', 'k');
            end
        end
    end
    
    
    % Set of GUI plotting functions
    methods
        function guiPlotCableForces(obj, plot_ax)
            obj.plotCableForces([], plot_ax);
        end        
        function guiPlotInteractionForceMagnitudes(obj, plot_ax)
            obj.plotInteractionForceMagnitudes([], plot_ax);
        end        
    end
end

