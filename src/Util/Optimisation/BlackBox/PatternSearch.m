% Class for Pattern Search
%
% Author        : Dominic Chan
% Created       : 2018
% Description    :
%    Black Box Optimisation using Pattern Search
classdef PatternSearch < BBOptimiserBase  
    properties  (Access = private) 
        % Basic
        input                   % input cells - u       
        m                       % no. of input channels
        output                  % output - y
        n                       % no. of output channels        
        
        % Update loop
        n_update                % Number of update
        current_update          % Counter of current_update
        
        % Operating input
        operating_input         % Operating Input
        operating_output        % Output of the operating input
        
        % Arrays
        sample_step             % Step size of taking samples
        input_samples           % array of input samples
        output_samples          % outputs of the input samples
        
        % Results
        input_array             % Array storing input results
        output_array            % Array storing output results
        
        % Parallel pool
        poolObj 
        isParallel
    end
    
    % Default settings
    properties  (Constant)        
        k_shrink    = 0.5        
        init_step   = 1
        step_tol    = 1e-2
    end
    
    methods
        % Constructor
        % m: no. of input channels; n: no. of output channels
        % n_update: no. of maximum update
        function ps = PatternSearch(m, n, n_update, init_input, objectiveFn)
            ps@BBOptimiserBase(m, objectiveFn);            
            ps.m = m;         
            ps.n = n;             
            ps.n_update = n_update;
            ps.operating_input = init_input;
            ps.initialise();
        end   
        % Initailise the ILC
        function initialise(obj)
            % Init variables 
            obj.input_array       = cell(1, 0);
            obj.output_array      = cell(1, 0);
            obj.current_update      = 1;   
            obj.sample_step         = obj.init_step;
        end
               
        % Function updating the memory buffer of input and output
        function updateILC(obj, input, output)
            % Record the best result
            obj.input_array{end+1}    = input;
            obj.output_array{end+1}   = output;                   
        end   
        
        %%%%%%%%%%%%%%%%%%%%%
        % Optimise function %
        %%%%%%%%%%%%%%%%%%%%%
        function [input_opt, output_opt] = optimise(obj)            
            % Start parallel pool   
            try 
                obj.poolObj = gcp('nocreate');
                if isempty(obj.poolObj)
                    obj.poolObj = parpool('local');
                end   
                CASPR_log.Info('Starting PS with parallel computing...'); 
                obj.isParallel = true;
            catch
                obj.poolObj = [];
                CASPR_log.Info('Starting PS...'); 
                CASPR_log.Info('The optimisation process can be slow without parallel computing.');
                obj.isParallel = false;
            end            

            % Calculate the output for init simplex                    
            start_tic = tic;            
            CASPR_log.Info('Calculating Initial Input...'); 
            obj.operating_output = obj.objectiveFn(obj.operating_input);                     
            obj.updateILC(obj.operating_input, obj.operating_output);
            CASPR_log.Info(sprintf('Initial output: %.4f', obj.operating_output));
            
            % Optimisation loop
            while obj.current_update <= obj.n_update
                CASPR_log.Info('');
                CASPR_log.Info(sprintf('*** Update: %d ***', obj.current_update));                
                
                if obj.terminate()
                    break;
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%
                %     Main procedure    %
                %%%%%%%%%%%%%%%%%%%%%%%%%
                % 1. Create input samples
                pos_samples = repmat(obj.operating_input, 1, obj.m) + obj.sample_step.*eye(obj.m);
                neg_samples = repmat(obj.operating_input, 1, obj.m) - obj.sample_step.*eye(obj.m);
                obj.input_samples = [pos_samples, neg_samples];
                obj.output_samples = zeros(obj.m*2, 1);
                % 2. Evaluate samples
                obj.evaluateSamples();   
                % 3. Select min point as next operating point
                [min_output_sample, min_index] = min(obj.output_samples);
                min_input_sample = obj.input_samples(:,min_index);
                
                if min_output_sample < obj.operating_output
                    obj.operating_input = min_input_sample;
                    obj.operating_output = min_output_sample;
%                     obj.sample_step = obj.sample_step/obj.k_shrink;
                    CASPR_log.Info(sprintf('Min sample output: %.4f', min_output_sample));
                else
                    % Shrink                    
                    obj.sample_step = obj.sample_step*obj.k_shrink;
                    CASPR_log.Info(sprintf('Shrinking sample size to %f...', obj.sample_step));
                end
                obj.updateILC(obj.operating_input, obj.operating_output);
                obj.current_update = obj.current_update + 1;
            end 
            % Find opt            
            input_opt = obj.operating_input;
            output_opt = obj.operating_output;
            
            % Finish message
            obj.finishMessage(start_tic);   
            
            % Plot
            obj.plotResults();
        end      
        
        % Plotting functions
        function f = plotResults(obj)
            f(1) = obj.plotInputResults();
            f(2) = obj.plotOutputResults();
        end
        function f = plotInputResults(obj)
            input_soln_array = cell2mat(obj.input_array);
            iteration_array = 1:1:size(input_soln_array, 2);
            CASPR_log.Assert(length(iteration_array)>1, 'Input array is too short for plotting.');                
            f = figure;          
            hold on; box on; grid on;
            plot(iteration_array, input_soln_array', 'LineWidth', 2); 
            title('Null Space Exploration Parameters u');
            xlabel('Iteration');
            ylabel('Null Space Exploration Parameters');
            xlim([1 size(input_soln_array, 2)]);            
        end
        function f = plotOutputResults(obj)
            output_soln_array = cell2mat(obj.output_array);
            iteration_array = 1:1:size(output_soln_array, 2);
            CASPR_log.Assert(length(iteration_array)>1, 'Output array is too short for plotting.');   
            f = figure;
            hold on; box on; grid on;
            plot(iteration_array, output_soln_array', 'LineWidth', 1.5,'Color', 'k');
            title('Performance Function P(u)');
            xlabel('Iteration');            
            ylabel('Performance Function');
            xlim([1 size(output_soln_array, 2)]);
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
        function evaluateSamples(obj)   
            CASPR_log.Info(sprintf('Evaluating Samples with step size = %f', obj.sample_step));
            if obj.isParallel           
                n_workers = obj.poolObj.NumWorkers;                
                for j = 1:ceil((obj.m*2)/n_workers) 
                    CASPR_log.SetLoggingDetails(CASPRLogLevel.ERROR);
                    spmd
                        index = n_workers*(j-1) + labindex;                         
                        if index <= (obj.m*2)
                            current_output = obj.objectiveFn(obj.input_samples(:,index));                       
                        end
                    end 
                    CASPR_log.SetLoggingDetails(CASPRLogLevel.INFO);
                    % Load input and cost array to ioarray
                    for k = 1:n_workers
                        if index{k} <= (obj.m*2)                            
                            obj.output_samples(index{k}) = current_output{k};                             
                        end                      
                    end  
                end
            else
                for j = 1:(obj.m*2)
                    obj.output_samples(j) = obj.objectiveFn(obj.input_samples(:,j)); 
                end
            end     
        end
        function finishMessage(obj, start_tic)
            CASPR_log.Info('');
            CASPR_log.Info(repmat('*', 1, 40));   
            
            CASPR_log.Info(sprintf('Pattern Search is finished in %.2f seconds', toc(start_tic)));
            CASPR_log.Info(sprintf('%d Updates have been performed.', obj.current_update-1));
            first_output = obj.output_array{1};
            last_output = obj.output_array{end};
            cost_improvement = (last_output(1) - first_output(1))/first_output(1);
            CASPR_log.Info(sprintf('Improvements of %.2f %s has been achieved.', -cost_improvement*100, '%%'));
            
            CASPR_log.Info(repmat('*', 1, 40));
            CASPR_log.Info('');
        end
        function flag = terminate(obj)
            if obj.sample_step < obj.step_tol
                flag = true;
            else
                flag = false;
            end                
        end
    end    
end

