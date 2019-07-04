% PSO black box optimiser
% 
% Author        : Darwin LAU
% Created       : 2016
% Description	:
%   Just a basic implementation, need to add many more features (like some
%   in the PSO toolbox, such as different bounding box handling methods or 
%   equality constraints etc).
%   - Support parallel computing (06-2018)
%   - Add draw graph functions (06-2018)
classdef PSOOptimiser < BBOptimiserBase
    properties (SetAccess = private)    
        numParticles        % Number of particles
        maxIterations       % Maximum number of iterations
        % Rule of thumb: phi_g = phi_p = (w+1)^2/2
        w_vel = 0.73        % Velocity weight, use values from 0.4 to 0.9
        w_vel_f = 0.4       % Final velocity weight
        phi_p = 1.5         % Particle best weighting, phi_p + phi_g <= 4
        phi_g = 1.5         % Global best weighting   
       
        x_min               % Min bound for input
        x_max               % Max bound for input
        v_min               % Min bound for input velocity
        v_max               % Max bound for input velocity
        
        % Result records
        input_array         % Save input
        std_array           % Save max standard deviation of the particles among input channels 
        output_array        % Save history of best Q
        
        % Graphs
        graph_opt = 'rg'    % rg: red-green; gb: grey-black
        graph_path          % Path for saving graphs        
    end
    
    methods
        function op = PSOOptimiser(xmin, xmax, objectiveFn, maxIterations, numParticles)
            op@BBOptimiserBase(length(xmin), objectiveFn);
            op.x_min = xmin;
            op.x_max = xmax;
            op.v_min = - (xmax - xmin)/2;
            op.v_max = (xmax - xmin)/2;
            
            if nargin > 3
                op.maxIterations = maxIterations;
                if nargin > 4
                    op.numParticles = numParticles;
                else       
                    op.numParticles = ceil(10 + 2*sqrt(2*op.numVars));
                end
            else
                op.maxIterations = 10;
                op.numParticles = ceil(10 + 2*sqrt(2*op.numVars));
            end
        end
        
        function [x_opt, Q_opt] = optimise(obj, graph_path)
            Q_opt = Inf;
            Q_particles_opt = Inf * ones(obj.numParticles,1);
            x_particles = zeros(obj.numVars, obj.numParticles);
            v_particles = zeros(obj.numVars, obj.numParticles);
            pBest = zeros(obj.numVars, obj.numParticles);
            gBest = zeros(obj.numVars, 1);
            gBestId = 0;
            
            % Start parallel pool   
            try 
                poolObj = gcp('nocreate');
                if isempty(poolObj)
                    poolObj = parpool('local');
                end  
                n_workers = poolObj.NumWorkers;
                isParallel = true;
                CASPR_log.Info('Starting PSO with parallel computing...');   
            catch
                isParallel = false;
                CASPR_log.Info('Starting PSO...');
                CASPR_log.Info('The optimisation process can be slow without parallel computing.');
            end
            
            % Graph log
            if nargin > 1
                obj.graph_path = graph_path;
            else
                obj.graph_path = [];
            end
            
            % Initialise the initial random particles
            w_init = obj.w_vel;
            
            CASPR_log.SetLoggingDetails(CASPRLogLevel.ERROR);
            if isParallel                             
                % Evaluate cost for each particle
                spmd         
                    for p = labindex:n_workers:obj.numParticles

                        x_particles(:, p) = PSOOptimiser.random_vector(obj.x_min, obj.x_max);
                        v_particles(:, p) = PSOOptimiser.random_vector(obj.v_min, obj.v_max);                        

                        Q = obj.objectiveFn(x_particles(:, p));                        

                        if (Q <= Q_particles_opt(p))
                            pBest(:, p) = x_particles(:, p);
                            Q_particles_opt(p) = Q;                            
                        end                        
                    end  
                end
                
                % Rebuild Q_particles_opt
                tmp_Q_particles_opt = zeros(obj.numParticles,1);
                tmp_pBest = zeros(obj.numVars, obj.numParticles);
                tmp_x_particles = zeros(obj.numVars, obj.numParticles);
                tmp_v_particles = zeros(obj.numVars, obj.numParticles);
                for w = 1:n_workers
                    for p = w:n_workers:obj.numParticles                    
                        current_Q_particles_opt = Q_particles_opt{w};
                        current_pBest = pBest{w};
                        current_x_particles = x_particles{w};
                        current_v_particles = v_particles{w};
                        tmp_Q_particles_opt(p) = current_Q_particles_opt(p);
                        tmp_pBest(:, p) = current_pBest(:, p);     
                        tmp_x_particles(:, p) = current_x_particles(:,p);
                        tmp_v_particles(:, p) = current_v_particles(:,p);
                    end
                end
                Q_particles_opt = tmp_Q_particles_opt;
                pBest = tmp_pBest;
                x_particles = tmp_x_particles;
                v_particles = tmp_v_particles;
            else
                % Evaluate cost for each particle
                for p = 1:obj.numParticles
                    x_particles(:, p) = PSOOptimiser.random_vector(obj.x_min, obj.x_max);
                    v_particles(:, p) = PSOOptimiser.random_vector(obj.v_min, obj.v_max);                        

                    Q = obj.objectiveFn(x_particles(:, p));                        

                    if (Q <= Q_particles_opt(p))
                        pBest(:, p) = x_particles(:, p);
                        Q_particles_opt(p) = Q;                            
                    end                    
                end  
            end             
            CASPR_log.SetLoggingDetails(CASPRLogLevel.INFO);
            
            % Save results
            obj.input_array{end+1} = x_particles;     
            obj.std_array{end+1} = max(std(x_particles, 0, 2));
            
            % Update best values
            for p = 1:obj.numParticles
                if (Q_particles_opt(p) <= Q_opt)
                    gBest = pBest(:, p);
                    gBestId = p;
                    Q_opt = Q_particles_opt(p);
                end
            end        
            obj.output_array{end+1} = Q_opt;
            CASPR_log.Info('Initialisation Stage:');
            CASPR_log.Info(sprintf('- Q_opt: %.4f, gBest particle %d', Q_opt, gBestId));
            
            % Run for the iterations            
            for it = 1:obj.maxIterations  
                CASPR_log.SetLoggingDetails(CASPRLogLevel.ERROR);
                if isParallel                    
                    spmd
                        for p = labindex:n_workers:obj.numParticles
                            rp = PSOOptimiser.random_vector(zeros(obj.numVars, 1), ones(obj.numVars, 1));
                            rg = PSOOptimiser.random_vector(zeros(obj.numVars, 1), ones(obj.numVars, 1));
                            v_particles(:, p) = obj.w_vel * v_particles(:, p) + obj.phi_p * rp .* (pBest(:, p) - x_particles(:, p)) + obj.phi_g * rg .* (gBest - x_particles(:, p));
                            v_particles(:, p) = PSOOptimiser.bound_vector(v_particles(:, p), obj.v_min, obj.v_max);
                            x_particles(:, p) = x_particles(:, p) + v_particles(:, p);
                            x_particles(:, p) = PSOOptimiser.bound_vector(x_particles(:, p), obj.x_min, obj.x_max);

                            Q = obj.objectiveFn(x_particles(:, p));

                            if (Q <= Q_particles_opt(p))
                                pBest(:, p) = x_particles(:, p);
                                Q_particles_opt(p) = Q;                                
                            end                            
                        end   
                    end
                    
                    % Rebuild Q_particles_opt
                    tmp_Q_particles_opt = zeros(obj.numParticles,1);
                    tmp_pBest = zeros(obj.numVars, obj.numParticles);
                    tmp_x_particles = zeros(obj.numVars, obj.numParticles);
                    tmp_v_particles = zeros(obj.numVars, obj.numParticles);
                    for w = 1:n_workers
                        for p = w:n_workers:obj.numParticles                    
                            current_Q_particles_opt = Q_particles_opt{w};
                            current_pBest = pBest{w};
                            current_x_particles = x_particles{w};
                            current_v_particles = v_particles{w};
                            tmp_Q_particles_opt(p) = current_Q_particles_opt(p);
                            tmp_pBest(:, p) = current_pBest(:, p);     
                            tmp_x_particles(:, p) = current_x_particles(:,p);
                            tmp_v_particles(:, p) = current_v_particles(:,p);
                        end
                    end
                    Q_particles_opt = tmp_Q_particles_opt;
                    pBest = tmp_pBest;
                    x_particles = tmp_x_particles;
                    v_particles = tmp_v_particles;
                else
                    for p = 1:obj.numParticles
                        rp = PSOOptimiser.random_vector(zeros(obj.numVars, 1), ones(obj.numVars, 1));
                        rg = PSOOptimiser.random_vector(zeros(obj.numVars, 1), ones(obj.numVars, 1));
                        v_particles(:, p) = obj.w_vel * v_particles(:, p) + obj.phi_p * rp .* (pBest(:, p) - x_particles(:, p)) + obj.phi_g * rg .* (gBest - x_particles(:, p));
                        v_particles(:, p) = PSOOptimiser.bound_vector(v_particles(:, p), obj.v_min, obj.v_max);
                        x_particles(:, p) = x_particles(:, p) + v_particles(:, p);
                        x_particles(:, p) = PSOOptimiser.bound_vector(x_particles(:, p), obj.x_min, obj.x_max);

                        Q = obj.objectiveFn(x_particles(:, p));

                        if (Q <= Q_particles_opt(p))
                            pBest(:, p) = x_particles(:, p);
                            Q_particles_opt(p) = Q;                                
                        end                        
                    end   
                end                
                CASPR_log.SetLoggingDetails(CASPRLogLevel.INFO);
                
                % Save results
                obj.input_array{end+1} = x_particles;
                obj.std_array{end+1} = max(std(x_particles, 0, 2));
                
                % Update best values
                for p = 1:obj.numParticles
                    if (Q_particles_opt(p) <= Q_opt)
                        gBest = pBest(:, p);
                        gBestId = p;
                        Q_opt = Q_particles_opt(p);
                    end
                end       
                obj.output_array{end+1} = Q_opt;
                CASPR_log.Info(sprintf('Iteration: %d', it));
                CASPR_log.Info(sprintf('- Q_opt: %.4f, gBest particle %d', Q_opt, gBestId));
                
                % Check convergence
                if obj.isConverged()
                    CASPR_log.Info('Convergence criteria met.');
                    break;
                end 
                % Reduce velocity weight
                obj.w_vel = obj.w_vel - (w_init - obj.w_vel_f)/obj.maxIterations;                
            end
            
            % Finishing message
            obj.finishMessage(Q_opt, gBestId);
            
            obj.plotResults();
            
            % Return value
            x_opt = gBest;   
        end
        
        % Plot results
        function plotResults(obj)
            % Plot Qopt history
            obj.plotQopt();
            
            % Draw graphs
            if ~isempty(obj.graph_path)
                obj.drawGraphs();
            end
        end
        
        % Getters
        % Retrievers for inputs and outputs
        function value = getInputArray(obj)
            value = obj.input_array;
        end
        function value = getOutputArray(obj)
            value = obj.output_array;
        end
    end
    
    methods (Access = private)
        % Plot Qopt history       
        function plotQopt(obj)            
            Qopt_history = cell2mat(obj.output_array);            
            iteration_array = 1:1:size(Qopt_history, 2);
            CASPR_log.Assert(length(Qopt_history)>1, 'Qopt history is too short for plotting.'); 
            figure;
            plot(iteration_array, Qopt_history', 'LineWidth', 1.5);
            title('PSO - Q Optimal Curve');
            xlabel('Iterations');
            ylabel('Best Q');
            xlim([1 size(Qopt_history, 2)]);
        end
        
        % Draw graphs
        function drawGraphs(obj)
            CASPR_log.Assert(obj.numVars > 1, 'Too few number of variables for plotting.');
            
            % Init graphs       
            graph_array = cell(1, ceil(obj.numVars/2));
            for g = 1:length(graph_array)
                graph_array{g} = figure;
                figure(graph_array{g});
                hold on
                box on
                title('PSO');                
                if length(graph_array)*2 ~= obj.numVars && g == length(graph_array) % last input  
                    xlabel(sprintf('Input %d', 2*(g-1)+1));
                    ylabel(sprintf('Input %d', 2*(g-1)+1));  
                    axis([obj.x_min(2*(g-1)+1) obj.x_max(2*(g-1)+1) obj.x_min(2*(g-1)+1) obj.x_max(2*(g-1)+1)]);
                else                        
                    xlabel(sprintf('Input %d', 2*(g-1)+1));
                    ylabel(sprintf('Input %d', 2*g)); 
                    axis([obj.x_min(2*(g-1)+1) obj.x_max(2*(g-1)+1) obj.x_min(2*g) obj.x_max(2*g)]);
                end                    
            end    
            
            % Min and Max std
            std_min = min(cell2mat(obj.std_array));
            std_max = max(cell2mat(obj.std_array));
           
            % Draw graphs
            % Loop through each generation
            for i = 1:length(obj.input_array) 
                current_input = obj.input_array{i};  
                current_std = obj.std_array{i};
                
                % Loop through each graph
                for g = 1:length(graph_array)                       
                    figure(graph_array{g});
                    hold on
                    if length(graph_array)*2 ~= obj.numVars && g == length(graph_array) % last input
                        for p = 1:size(current_input,2)
%                             color_ratio = (current_std - std_min)/(std_max - std_min);
                            color_ratio = 1 - i/length(obj.input_array);
                            
                            if strcmp(obj.graph_opt, 'gb')
                                c = [0.8*color_ratio 0.8*color_ratio 0.8*color_ratio];
                            elseif strcmp(obj.graph_opt, 'rg')                          
                                c = [color_ratio 1-color_ratio 0];  
                            else
                                CASPR_log.Warn('Invalid color option for PSO graphs.');
                            end                           
                            scatter(current_input(2*(g-1)+1,p), current_input(2*(g-1)+1,p), 10, 'MarkerFaceColor', c, ...
                                'MarkerEdgeColor', c);
                        end
                        drawnow;
                    else                        
                        for p = 1:size(current_input,2)
%                             color_ratio = (current_std - std_min)/(std_max - std_min);
                            color_ratio = 1 - i/length(obj.input_array);
                            
                            if strcmp(obj.graph_opt, 'gb')
                                c = [0.8*color_ratio 0.8*color_ratio 0.8*color_ratio];
                            elseif strcmp(obj.graph_opt, 'rg')                          
                                c = [color_ratio 1-color_ratio 0];  
                            else
                                CASPR_log.Warn('Invalid color option for PSO graphs.');
                            end                         
                            scatter(current_input(2*(g-1)+1,p), current_input(2*g,p), 10, 'MarkerFaceColor', c, ...
                                'MarkerEdgeColor', c);
                        end
                        drawnow;
                    end
                    
                    % Save to gif
                    frame = getframe(gcf);
                    im = frame2im(frame);
                    [imind, cm] = rgb2ind(im, 256);
                    
                    filename = sprintf('%s/PSO_fig%d.gif', obj.graph_path, g);
                    if i == 1
                        imwrite(imind, cm, filename, 'gif', 'DelayTime', 0.2, 'Loopcount', inf);
                    elseif i == length(obj.input_array)
                        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 1);
                    else                        
                        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.2);
                    end
                    
                    % Save final graph
                    if i == length(obj.input_array) 
                        saveas(gcf, sprintf('%s/PSO_fig%d_final.jpg', obj.graph_path, g));
                    end
                end
            end
        end
        
        % Check convergence (not yet added any convergence conditions)
        function [flag] = isConverged(~)
            flag = false;
        end
        
        % Finish message
        function finishMessage(~, Q_opt, gBestId)
            CASPR_log.Info('');
            CASPR_log.Info(repmat('*', 1, 40));
            CASPR_log.Info('Particle Swarm Optimisation is finished.');
            CASPR_log.Info(sprintf('- Optimal Q: %.4f', Q_opt));
            CASPR_log.Info(sprintf('- Global best Particle: %d', gBestId));
            CASPR_log.Info(repmat('*', 1, 40));
            CASPR_log.Info('');
        end
    end
    
    methods (Static, Access = private)
        function v = random_value(lower, upper)
            v = lower + (upper - lower)*rand();
        end
        
        function v = random_vector(lower, upper)
            v = lower + (upper - lower).*rand(size(lower));
        end
        
        function v = bound_vector(v_in, lower, upper)
            v = v_in;
            for i=1:length(v_in)
                if v(i) > upper(i)
                    v(i) = upper(i);
                elseif v(i) < lower(i)
                    v(i) = lower(i);
                end
            end
        end        
    end    
end
