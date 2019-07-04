% Class for BFGS 
%
% Author        : Dominic Chan
% Created       : 2018
% Description    :
%    Black Box Optimisation using the  
%    Broyden-Flectcher-Goldfarb-Shanno (BFGS) algorithm
classdef BFGS < BBOptimiserBase  
    properties  (Access = private) 
        % Basic
        input                   % input cells - u       
        m                       % no. of input channels
        output                  % output - y
        n                       % no. of output channels
        q                       % learning rate 
        currentInput            % current input vector
        nextInput               % calculated next input according to update law  
        n_update                % Number of update
        current_update          % Counter of current_update
        
        % Results
        input_array             % Array storing input results
        output_array            % Array storing output results
        
        % BFGS
        Binv                    % B inverse
        pre_grad                % Previous gradient   
        pre_update_vec          % Previous update vector
        Binv_d                  % Difference of B inverse
        
        % Parallel pool
        poolObj
        
        % Gradient
        grad                    % Gradient of the objective function 
        grad_vec                % Matrix version of the gradient
        delta                   % Perturbation
        experiment              % Flag for experimental update 
        
        % Line search
        a0                      % Initial step in line search
        r                       % Reduction rate
        c                       % Decreasing coefficient
        max_search              % Max search iteration
        pre_search_iteration    % Previous number of search iteration
        
        % Convergence 
        conv_r                  % Convergence ratio
        steady_cond             % Condition of the number of iteration for steady state
        steady_iteration        % Record the lastest iteration that meets the convergence ratio
        steady_count            % Count the number of consecutive steady iterations
        pos_grad                % Flag for positive gradient
    end
    
    % Default settings
    properties  (Constant)
        % Gradient
        q_def               = 1e-4      
        delta_def           = 0.1
        % Line search
        a0_def              = 4
        r_def               = 0.2
        c_def               = 0.3
        max_search_def      = 2
        % Convergence 
        conv_r_def          = 1e-6      
        steady_cond_def     = 5
    end
    
    methods
        % Constructor
        % m: no. of input channels; n: no. of output channels
        % n_update: no. of maximum update
        function bfgs = BFGS(m, n, n_update, init_input, objectiveFn)
            bfgs@BBOptimiserBase(m, objectiveFn);            
            bfgs.m = m;         
            bfgs.n = n;             
            bfgs.n_update = n_update;
            bfgs.currentInput = init_input;
            % Default settings
            bfgs.setDefParameters();            
            bfgs.initialise();
        end   
        % Initailise the ILC
        function initialise(obj)
            % Init variables
            obj.gradInit();
            obj.Binv                = eye(obj.m);   
            obj.Binv_d              = zeros(obj.m);
            obj.steady_iteration    = -1;     
            obj.steady_count        = 0;    
            obj.pos_grad            = false;  
            obj.input_array       = cell(1, 0);
            obj.output_array      = cell(1, 0);
            obj.current_update      = 1;
            obj.pre_search_iteration= 1;           
        end
               
        % Function updating the memory buffer of input and output
        function updateILC(obj, input, output)
            obj.input{end+1}    = input;
            obj.output{end+1}   = output;
            index = length(obj.input);
            if index > 1 
                d_input = (obj.input{index}-obj.input{1});
                % update grad
                obj.grad{index-1} = (obj.output{index}-obj.output{1})/...
                    d_input(index-1);
            end
            if index == obj.m + 1
                % Record results
                obj.input_array{end+1} = obj.input{1};
                obj.output_array{end+1} = obj.output{1};
                CASPR_log.Info(sprintf('Output: %.2f', obj.output{1}));
                obj.grad_vec = cell2mat(obj.grad');                
                obj.gradientUpdate();             
                obj.experiment = false;                
            else
                obj.experimentUpdate();
                obj.experiment = true;
            end            
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
                CASPR_log.Info('Starting BFGS with parallel computing...'); 
            catch
                obj.poolObj = [];
                CASPR_log.Info('Starting BFGS...'); 
                CASPR_log.Info('The optimisation process can be slow without parallel computing.');
            end
            
            % Init
            convergence_flag = false;           
            start_tic = tic;
            % Optimisation loop
            while obj.current_update <= obj.n_update
                CASPR_log.Info('');
                CASPR_log.Info(sprintf('*** Update: %d ***', obj.current_update));
                % Find Gradient                
                obj.findGradient();
                % Check convergence
                if obj.isConvergence()
                    convergence_flag = true;
                    break;
                end
                % Line Search                
                obj.lineSearch();                
                obj.current_update = obj.current_update + 1;
            end      
            input_opt = obj.input_array{end};
            output_opt = obj.output_array{end};
            
            % Finish message
            obj.finishMessage(start_tic, convergence_flag);            
        end       
        
        % Line search
        function lineSearch(obj)
            CASPR_log.Info('Performing line search...');
            if ~isempty(obj.poolObj)
                isParallel = true;
                n_workers = obj.poolObj.NumWorkers;
                n_searches = n_workers;
            else
                isParallel = false;
                n_searches = 4; % Default number of searches
            end
            % Get searching direction from the update vector
            search_direction = (obj.pre_update_vec)/norm(obj.pre_update_vec);
            % Init a0
            a_0 = obj.a0/obj.pre_search_iteration;           
            % Flag for Armijo condition
            isArmijo = false;    
            search_iteration = 1;
            operating_input = obj.currentInput;
            while ~isArmijo
                % Array that saves searching outputs
                search_output_array = zeros(1, n_searches);
                
                CASPR_log.SetLoggingDetails(CASPRLogLevel.ERROR);
                if isParallel         
                    % Search
                    spmd                               
                        a = obj.r^(labindex-1)*a_0;
                        current_input = operating_input + a*search_direction;
                        % Get output
                        current_output = obj.objectiveFn(current_input); 
                    end                
                    
                    % Reconstruct output                    
                    for i=1:length(search_output_array)
                        search_output_array(i) = current_output{i};
                    end
                else
                    % Default searching times: 4
                    for i=1:n_searches                            
                        a = obj.r^(i-1)*a_0;
                        current_input = operating_input + a*search_direction;
                        % Get output
                        search_output_array(i) = obj.objectiveFn(current_input); 
                    end  
                end
                CASPR_log.SetLoggingDetails(CASPRLogLevel.INFO);
                
                % Optain min ouput
                [minval, minindex] = min(search_output_array);
                minindex = minindex - 1;
                
                % compare result from line search and grad
                [min_v, min_i] = min(obj.grad_vec);
                min_v = min_v*obj.delta; % convert back to cost difference
                min_grad_cost = obj.output_array{end} + min_v;
                obj.pos_grad = min_v >= 0;                
              
                % Check armijo condition
                if (obj.output_array{end} - obj.c*a_0*obj.r^minindex*abs(obj.grad_vec'*search_direction) >= minval)  
                    isArmijo = true;
                    CASPR_log.Info(sprintf('Armijo met.'));                    
                    if min_grad_cost < minval   % grad is better
                        update_vec = zeros(size(obj.grad_vec));
                        update_vec(min_i) = obj.delta;
                        CASPR_log.Info(sprintf('Input updated with delta: %f at index: %d', obj.delta, min_i));
                    else                % line search is better                        
                        update_vec = a_0*obj.r^minindex*search_direction;                        
                        CASPR_log.Info(sprintf('Input updated with r: %f from Line Search', obj.r^((search_iteration-1)*n_searches+minindex)));
                    end                    
                    obj.currentInput = operating_input + update_vec;                   
                    % Update s
                    obj.pre_update_vec = update_vec;
                    obj.pre_search_iteration = search_iteration;
                elseif (search_iteration == obj.max_search)  
                    CASPR_log.Info(sprintf('Max Search Iteration reached'));                    
                    % Compare cost
                    if obj.output_array{end} > minval || obj.output_array{end} > min_grad_cost % better result(s) found
                        if minval > min_grad_cost % grad is better
                            update_vec = zeros(size(obj.grad_vec));
                            update_vec(min_i) = obj.delta;
                            CASPR_log.Info(sprintf('Input updated with delta: %f at index: %d', obj.delta, min_i));
                        else % line search is better
                            update_vec = a_0*obj.r^minindex*search_direction;
                            CASPR_log.Info(sprintf('Input updated with r: %f', obj.r^((search_iteration-1)*n_searches+minindex)));
                        end
                    else   % use gradient descent
                        update_vec = -obj.q*obj.grad_vec/norm(obj.grad_vec);
                        CASPR_log.Info(sprintf('Input Updated with gradient descent.')); 
                    end  
                    
                    obj.currentInput = operating_input + update_vec;                  
                    % Update s
                    obj.pre_update_vec = update_vec;
                    obj.pre_search_iteration = search_iteration;                   
                    break;                    
                else
                    a_0 = a_0*obj.r^n_searches;
                    search_iteration = search_iteration + 1;
                end  
            end 
            if search_iteration > 1
                CASPR_log.Info(sprintf('Line search ended in %d searches', search_iteration));
            else
                CASPR_log.Info('Line search ended in 1 search');            
            end
        end
        
        % Find Gradient
        function findGradient(obj)
            CASPR_log.Info('Finding gradient...');
            % Check parallel
            if ~isempty(obj.poolObj)
                isParallel = true;
                n_workers = obj.poolObj.NumWorkers;                
            else
                isParallel = false;               
            end
            
            ioarray = cell(2, obj.m+1);
            
            CASPR_log.SetLoggingDetails(CASPRLogLevel.ERROR);
            if isParallel
                % Calculate output for current point and each deviated input channel
                for j = 1:ceil((obj.m+1)/obj.poolObj.NumWorkers) 
                    spmd
                        index = n_workers*(j-1) + labindex; 
                        if index <= obj.m+1
                            current_input = obj.currentInput;
                            % Deviations
                            if index ~= 1
                                current_input(index-1) = current_input(index-1) + obj.delta;
                            end
                            current_output = obj.objectiveFn(current_input);                       
                        end
                    end 
                    
                    % Load input and cost array to ioarray
                    for k = 1:n_workers
                        if index{k} <= length(ioarray)
                            ioarray{1, index{k}} = current_input{k};
                            ioarray{2, index{k}} = current_output{k};  
                        end                      
                    end  
                end
            else
                % Loop for 1+m times (current point + no. of input
                % channels)
                for j = 1:obj.m+1
                    current_input = obj.currentInput;
                    % Deviations
                    if j ~= 1
                        current_input(j-1) = current_input(j-1) + obj.delta;
                    end
                    current_output = obj.objectiveFn(current_input);
                    % Save results to ioarray
                    ioarray{1, j} = current_input;
                    ioarray{2, j} = current_output;
                end
            end   
            CASPR_log.SetLoggingDetails(CASPRLogLevel.INFO);
            
            % Update ILC with ioarray
            for i = 1:length(ioarray)
                obj.updateILC(ioarray{1,i}, ioarray{2,i}); 
            end   
        end
        
        % Getters
        function [value] = getNextInput(obj)
            value = obj.nextInput;                 
        end         
        function [value] = getExperiment(obj)
            value = obj.experiment;
        end
        function [value] = getGrad(obj)
            value = cell2mat(obj.grad);
        end
        function [value] = getBinvd(obj)
            value = obj.Binv_d;
        end
        
        % Setters
        function setPreUpdateVec(obj, s)
            obj.pre_update_vec = s;
        end   
         % Set the default parameters for ILC
        function setDefParameters(obj)
            obj.setGradParameters(obj.q_def, obj.delta_def);
            obj.setLineParameters(obj.a0_def, obj.r_def, obj.c_def, obj.max_search_def);
            obj.setConvParameters(obj.conv_r_def, obj.steady_cond_def);
        end
        % Set the gradient parameters for ILC
        function setGradParameters(obj, q, delta)
            obj.q       = q;
            obj.delta   = delta;
        end
        % Set the line search parameters for ILC
        function setLineParameters(obj, a0, r, c, max_search)
            obj.a0          = a0;
            obj.r           = r;
            obj.c           = c;    
            obj.max_search  = max_search;
        end
        % Set the convergence parameters for ILC
        function setConvParameters(obj, conv_r, steady_cond)
            obj.conv_r      = conv_r;
            obj.steady_cond = steady_cond;
        end
        
        % Plotting functions
        function plotResults(obj)
            obj.plotInputResults();
            obj.plotOutputResults();
        end
        function plotInputResults(obj)
            input_soln_array = cell2mat(obj.input_array);
            iteration_array = 1:1:size(input_soln_array, 2);
            CASPR_log.Assert(length(iteration_array)>1, 'Input array is too short for plotting.');                
            figure;
            plot(iteration_array, input_soln_array', 'LineWidth', 1.5);
            title('ILC - Input Results');
            xlabel('Iterations');
            ylabel('Input');
            xlim([1 size(input_soln_array, 2)]);
        end
        function plotOutputResults(obj)
            output_soln_array = cell2mat(obj.output_array);
            iteration_array = 1:1:size(output_soln_array, 2);
            CASPR_log.Assert(length(iteration_array)>1, 'Output array is too short for plotting.');   
            figure;
            plot(iteration_array, output_soln_array', 'LineWidth', 1.5);
            title('ILC - Output Results');
            xlabel('Iterations');            
            ylabel('Output');
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
        % Grad init
        function gradInit(obj)
            obj.grad                = cell(1, obj.m); 
            obj.experiment          = false;
            obj.input               = cell(1, 0);
            obj.output              = cell(1, 0); 
        end
        % Gradient Update
        function gradientUpdate(obj)
            u_0 = obj.input{1};            
            if ~isempty(obj.pre_grad)
                y = obj.grad_vec - obj.pre_grad;                
                s = obj.pre_update_vec;                
                if s'*y > 0 % Curvature condition
                    % Update Binv
                    obj.Binv_d = (s'*y + y'*obj.Binv*y)*(s*s')/(s'*y)^2 - ...
                        (obj.Binv*y*s' + s*y'*obj.Binv)/(s'*y);
                    obj.Binv = obj.Binv + obj.Binv_d; 
                else
                    CASPR_log.Info('Failed curvature criterion!');                    
                    obj.Binv = eye(obj.m);   
                    obj.Binv_d = zeros(obj.m);                 
                end
            end
                
            % Update grad
            obj.pre_grad = obj.grad_vec;
            % Update vec            
            update_vec = obj.Binv*(-obj.grad_vec);   
            obj.pre_update_vec = obj.q*update_vec;
            u_0 = u_0 + obj.q*update_vec;            
            obj.nextInput = u_0;
            obj.gradInit();   
        end
        % Experiment Update
        function experimentUpdate(obj)
            index = length(obj.input);
            u_0 = obj.input{1};
            delta_vec = zeros(obj.m, 1);
            delta_vec(index) = obj.delta;
            obj.nextInput = u_0 + delta_vec;
        end        
        % convergence check
        function flag = isConvergence(obj)
            flag = false;
            % Check pos_grad and output
            if obj.pos_grad && obj.output_array{end} > obj.output_array{end-1}
                % last input is already optimal
                obj.input_array = obj.input_array(1:end-1);
                obj.output_array = obj.output_array(1:end-1);
                flag = true;
            elseif length(obj.output_array) > 1  % Check convergence
                if norm(obj.output_array{end} - obj.output_array{end-1})/obj.output_array{end-1} < obj.conv_r
                    % Check if consecutive
                    if obj.steady_iteration ~= obj.current_update - 1
                        obj.steady_count = 0;
                    end
                    obj.steady_count = obj.steady_count + 1;        
                    obj.steady_iteration = obj.current_update;
                end     
                % Check if 
                if obj.steady_count == obj.steady_cond
                    flag = true;
                end                                
            end   
        end
        
        function finishMessage(obj, start_tic, convergence_flag)
            CASPR_log.Info('');
            CASPR_log.Info(repmat('*', 1, 40));   
            CASPR_log.Info(sprintf('BFGS Optimisation is finished in %.2f seconds', toc(start_tic)));
            CASPR_log.Info(sprintf('%d Updates have been performed.', obj.current_update-1));
            cost_improvement = (obj.output_array{end} - obj.output_array{1})/obj.output_array{1};
            CASPR_log.Info(sprintf('Improvements of %.2f %s has been achieved.', -cost_improvement*100, '%%'));
            
            if convergence_flag
                CASPR_log.Info('Optimal solution found.'); 
            else
                CASPR_log.Info('The optimisation has not yet met the convergence conditions.'); 
                CASPR_log.Info('Larger number of updates is suggested.');
            end
            CASPR_log.Info(repmat('*', 1, 40));
            CASPR_log.Info('');
        end
    end    
end

