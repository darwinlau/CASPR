% Covariance Matrix Adaption Evolution Strategy
% 
% Author        : Dominic Chan
% Created       : 2018
% Description	:
%   Modified based on the code in "The CMA Evolution Strategy: A Tutorial" by
%   Nikolaus Hansen (2016)
%   https://arxiv.org/abs/1604.00772
classdef CMAES < BBOptimiserBase
    properties (SetAccess = private)        
        % User defined input parameters     
        xmean                   % objective variables initial point
        sigma                   % coordinate wise standard deviation (step size)
        stopfitness             % stop if fitness < stopfitness (minimization)
        stopgeneration          % stop after stopgeneration of generations
        stopeval                % stop after stopeval number of function evaluations

        % Strategy parameter setting: Selection  
        lambda                  % population size, offspring number
        mu                      % number of parents/points for recombination
        weights                 % muXone array for weighted recombination             
        mueff                   % variance-effectiveness of sum w_i x_i

        % Strategy parameter setting: Adaptation
        cc                      % time constant for cumulation for C
        cs                      % t-const for cumulation for sigma control
        c1                      % learning rate for rank-one update of C
        cmu                     % and for rank-mu update
        damps                   % damping for sigma 
                                                 
        % Initialize dynamic (internal) strategy parameters and constants
        pc                      % evolution paths for C 
        ps                      % evolution paths for sigma
        B                       % B defines the coordinate system
        D                       % diagonal D defines the scaling
        C                       % covariance matrix C
        invsqrtC                % C^-1/2 
        eigeneval               % track update of B and D
        chiN                    % expectation of ||N(0,I)|| == norm(randn(N,1))
        
        % Result records
        input_array             % Save input
        fitness_array           % Save fitness
        output_array            % Save fittest result
        std_array               % Save max standard deviation of the particles among input channels        
        
        % Graph log
        graph_opt = 'rg'        % Graph color option (rg: red-green; gb: grey-black)
        graph_path              % Path for saving graphs
    end
    
    methods
        function obj = CMAES(xmean, sigma, generation, objectiveFn)
            obj@BBOptimiserBase(length(xmean), objectiveFn);
            obj.xmean = xmean;
            obj.sigma = sigma;
            obj.stopgeneration = generation;
            obj.initialise();
        end
        
        function [x_opt, Q_opt] = optimise(obj, graph_path)            
            % Start parallel pool   
            try 
                poolObj = gcp('nocreate');
                if isempty(poolObj)
                    poolObj = parpool('local');
                end  
                n_workers = poolObj.NumWorkers;
                isParallel = true;
                CASPR_log.Info('Starting CMA-ES with parallel computing...');
            catch
                isParallel = false;
                CASPR_log.Info('Starting CMA-ES...');
                CASPR_log.Info('The optimisation process can be slow without parallel computing.');
            end
            
            % Graphs
            if nargin > 1
                obj.graph_path = graph_path;
            else
                obj.graph_path = [];
            end                        
            
            % Main loop            
            % Init
            counteval = 0;             
            Q_opt = Inf;
            while counteval < obj.stopeval                 
                % Generate and evaluate lambda offspring
                arx = zeros(obj.numVars, obj.lambda);
                arfitness = zeros(1, obj.lambda);
                
                CASPR_log.SetLoggingDetails(CASPRLogLevel.ERROR);
                if isParallel                    
                    spmd         
                        for k = labindex:n_workers:obj.lambda
                            % m + sig * Normal(0,C)
                            arx(:,k) = obj.xmean + obj.sigma * obj.B * (obj.D .* randn(obj.numVars,1));  
                            % objective function call
                            arfitness(k) = obj.objectiveFn(arx(:,k));                              
                        end
                    end
                    
                    % Reconstruct arx and arfitness
                    tmp_arx = zeros(length(obj.xmean), obj.lambda);
                    tmp_arfitness = zeros(1, obj.lambda);
                    for w = 1:n_workers
                        for k = w:n_workers:obj.lambda
                            current_arx = arx{w};
                            current_arfitness = arfitness{w};
                            tmp_arx(:,k) = current_arx(:,k);
                            tmp_arfitness(k) = current_arfitness(k);
                        end
                    end
                    arx = tmp_arx;
                    arfitness = tmp_arfitness;  
                else
                    for k = 1:obj.lambda
                        % m + sig * Normal(0,C)
                        arx(:,k) = obj.xmean + obj.sigma * obj.B * (obj.D .* randn(obj.numVars,1));  
                        % objective function call
                        arfitness(k) = obj.objectiveFn(arx(:,k));                          
                    end
                end
                CASPR_log.SetLoggingDetails(CASPRLogLevel.INFO);
                
                counteval = counteval+obj.lambda;
                
                % Save results
                obj.input_array{end+1} = arx;
                obj.fitness_array{end+1} = arfitness;                
                obj.std_array{end+1} = max(std(arx, 0, 2));

                % Sort by fitness and compute weighted mean into xmean
                [~, arindex] = sort(arfitness); % minimization
                xold = obj.xmean;
                obj.xmean = arx(:,arindex(1:obj.mu))*obj.weights;   % recombination, new mean value
                
                % Update x_opt and Q_opt
                if Q_opt > arfitness(arindex(1))
                    x_opt = arx(:, arindex(1));
                    Q_opt = arfitness(arindex(1));                    
                end
                obj.output_array{end+1} = Q_opt;
                CASPR_log.Info(sprintf('After %d evaluations...', counteval));
                CASPR_log.Info(sprintf('- Best cost: %.4f', Q_opt));

                % Cumulation: Update evolution paths
                obj.ps = (1-obj.cs)*obj.ps ... 
                    + sqrt(obj.cs*(2-obj.cs)*obj.mueff)*obj.invsqrtC*(obj.xmean-xold)/obj.sigma; 
                hsig = norm(obj.ps)/sqrt(1-(1-obj.cs)^(2*counteval/obj.lambda))/obj.chiN < 1.4 + 2/(obj.numVars+1);
                obj.pc = (1-obj.cc)*obj.pc ...
                    + hsig*sqrt(obj.cc*(2-obj.cc)*obj.mueff)*(obj.xmean-xold)/obj.sigma;

                % Adapt covariance matrix C
                artmp = (1/obj.sigma)*(arx(:,arindex(1:obj.mu))-repmat(xold,1,obj.mu));
                obj.C = (1-obj.c1-obj.cmu)*obj.C ...                % regard old matrix  
                    + obj.c1*(obj.pc*obj.pc' ...                    % plus rank one update
                    + (1-hsig)*obj.cc*(2-obj.cc)*obj.C) ...         % minor correction if hsig==0
                    + obj.cmu*artmp*diag(obj.weights)*artmp';       % plus rank mu update

                % Adapt step size sigma
                obj.sigma = obj.sigma*exp((obj.cs/obj.damps)*(norm(obj.ps)/obj.chiN - 1)); 

                % Decomposition of C into B*diag(D.^2)*B' (diagonalization)
                if counteval - obj.eigeneval > obj.lambda/(obj.c1+obj.cmu)/obj.numVars/10  % to achieve O(N^2)
                    obj.eigeneval = counteval;
                    obj.C = triu(obj.C) + triu(obj.C,1)';   % enforce symmetry
                    [obj.B,obj.D] = eig(obj.C);             % eigen decomposition, B==normalized eigenvectors
                    obj.D = sqrt(diag(obj.D));              % D is a vector of standard deviations now
                    obj.invsqrtC = obj.B*diag(obj.D.^-1)*obj.B';
                end                
                
                % Break, if fitness is good enough or condition exceeds 1e14, better termination methods are advisable 
                if arfitness(1) <= obj.stopfitness || max(obj.D) > 1e7 * min(obj.D)
                    break;
                end
            end
            
            % Finishing message
            obj.finishMessage(Q_opt);
            
            obj.plotResults();
        end
        
        % Result plotting
        function plotResults(obj)
            % Plot the fittest curve
            obj.plotFittest();
            
            % Draw scatter graphs
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
    
    methods (Access=private)
        function initialise(obj)
            % --------------------  Initialization --------------------------------  
            % Strategy parameter setting: Selection  
            obj.lambda = 4+floor(3*log(obj.numVars)); 
            obj.mu = obj.lambda/2;             
            obj.weights = log(obj.mu+1/2)-log(1:obj.mu)'; 
            obj.mu = floor(obj.mu);        
            obj.weights = obj.weights/sum(obj.weights);     % normalize
            obj.mueff=sum(obj.weights)^2/sum(obj.weights.^2); 
            
            % User defined input parameters  
            obj.stopfitness = 1e-10;  
            obj.stopeval = obj.lambda*obj.stopgeneration;            

            % Strategy parameter setting: Adaptation
            obj.cc = (4+obj.mueff/obj.numVars) / (obj.numVars+4 + 2*obj.mueff/obj.numVars);  
            obj.cs = (obj.mueff+2) / (obj.numVars+obj.mueff+5);  
            obj.c1 = 2 / ((obj.numVars+1.3)^2+obj.mueff);    
            obj.cmu = min(1-obj.c1, 2*(obj.mueff-2+1/obj.mueff)/((obj.numVars+2)^2+obj.mueff)); 
            obj.damps = 1 + 2*max(0, sqrt((obj.mueff-1)/(obj.numVars+1))-1) + obj.cs; 
                                                      
            % Initialize dynamic (internal) strategy parameters and constants
            obj.pc = zeros(obj.numVars,1); 
            obj.ps = zeros(obj.numVars,1);   
            obj.B = eye(obj.numVars,obj.numVars);                       
            obj.D = ones(obj.numVars,1);                     
            obj.C = obj.B*diag(obj.D.^2) * obj.B';            
            obj.invsqrtC = obj.B*diag(obj.D.^-1)*obj.B';   
            obj.eigeneval = 0;                      
            obj.chiN=obj.numVars^0.5*(1-1/(4*obj.numVars)+1/(21*obj.numVars^2)); 
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
                title('CMA-ES');
                if length(graph_array)*2 ~= obj.numVars && g == length(graph_array) % last input  
                    xlabel(sprintf('Input %d', 2*(g-1)+1));
                    ylabel(sprintf('Input %d', 2*(g-1)+1));                        
                else                        
                    xlabel(sprintf('Input %d', 2*(g-1)+1));
                    ylabel(sprintf('Input %d', 2*g)); 
                end                    
            end    
            
            % Min and Max variance
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
                            color_ratio = (current_std - std_min)/(std_max - std_min);
                            
                            if strcmp(obj.graph_opt, 'gb')
                                c = [0.8*color_ratio 0.8*color_ratio 0.8*color_ratio];
                            elseif strcmp(obj.graph_opt, 'rg')                          
                                c = [color_ratio 1-color_ratio 0];  
                            else
                                CASPR_log.Warn('Invalid color option for CMA-ES graphs.');
                            end                           
                            scatter(current_input(2*(g-1)+1,p), current_input(2*(g-1)+1,p), 10, 'MarkerFaceColor', c, ...
                                'MarkerEdgeColor', c);
                        end
                        drawnow;
                    else                        
                        for p = 1:size(current_input,2)
                            color_ratio = (current_std - std_min)/(std_max - std_min);
                            
                            if strcmp(obj.graph_opt, 'gb')
                                c = [0.8*color_ratio 0.8*color_ratio 0.8*color_ratio];
                            elseif strcmp(obj.graph_opt, 'rg')                          
                                c = [color_ratio 1-color_ratio 0];  
                            else
                                CASPR_log.Warn('Invalid color option for CMA-ES graphs.');
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
                    
                    filename = sprintf('%s/CMAES_fig%d.gif', obj.graph_path, g);
                    if i == 1
                        imwrite(imind, cm, filename, 'gif', 'DelayTime', 0.2, 'Loopcount', inf);
                    elseif i == length(obj.input_array)
                        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 1);
                    else                        
                        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.2);
                    end
                    
                    % Save final graph
                    if i == length(obj.input_array) 
                        saveas(gcf, sprintf('%s/CMAES_fig%d_final.jpg', obj.graph_path, g));
                    end
                end
            end
        end
        
        % Plot fittest result
        function plotFittest(obj)
            fittest = cell2mat(obj.output_array);
            generation_array = 1:1:size(fittest, 2);
            CASPR_log.Assert(length(fittest)>1, 'Fittest array is too short for plotting.'); 
            figure;
            plot(generation_array, fittest', 'LineWidth', 1.5);
            title('CMA-ES - Fitness Curve');
            xlabel('Generations');
            ylabel('Fittest value');
            xlim([1 size(fittest, 2)]);
        end
        
        % Finish message
        function finishMessage(~, Q_opt)
            CASPR_log.Info('');
            CASPR_log.Info(repmat('*', 1, 40));
            CASPR_log.Info('CMA-ES is finished.');
            CASPR_log.Info(sprintf('- Fittest value: %.4f', Q_opt));
            CASPR_log.Info(repmat('*', 1, 40));
            CASPR_log.Info('');
        end
    end
end

