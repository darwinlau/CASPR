% PSO black box optimiser
% 
% Author        : Darwin LAU
% Created       : 2016
% Description	:
%   Just a basic implementation, need to add many more features (like some
%   in the PSO toolbox, such as different bounding box handling methods or 
%   equality constraints etc).
classdef PSOOptimiser < BBOptimiserBase
    properties (SetAccess = private)
        % Rule of thumb: numParticles = 10 + 2*sqrt(2*numVars)
        numParticles = 30   % Number of particles
        maxIterations = 10 % Maximum number of iterations
        % Rule of thumb: phi_g = phi_p = (w+1)^2/2
        w_vel = 0.73        % Velocity weight, use values from 0.4 to 0.9
        phi_p = 1.5         % Particle best weighting, phi_p + phi_g <= 4
        phi_g = 1.5         % Global best weighting
        
        x_min
        x_max
        v_min
        v_max
    end
    
    methods
        function op = PSOOptimiser(xmin, xmax, objectiveFn)
            op@BBOptimiserBase(length(xmin), objectiveFn);
            op.x_min = xmin;
            op.x_max = xmax;
            op.v_min = - (xmax - xmin)/2;
            op.v_max = (xmax - xmin)/2;
        end
        
        function [x_opt, Q_opt] = optimise(obj)
            Q_opt = Inf;
            Q_particles_opt = Inf * ones(obj.numParticles,1);
            x_particles = zeros(obj.numVars, obj.numParticles);
            v_particles = zeros(obj.numVars, obj.numParticles);
            pBest = zeros(obj.numVars, obj.numParticles);
            gBest = zeros(obj.numVars, 1);
            gBestId = 0;
            
            % Initialise the initial random particles
            for p = 1:obj.numParticles
                x_particles(:, p) = PSOOptimiser.random_vector(obj.x_min, obj.x_max);
                v_particles(:, p) = PSOOptimiser.random_vector(obj.v_min, obj.v_max);
                
                Q = obj.objectiveFn(x_particles(:, p));
                
                if (Q <= Q_particles_opt(p))
                    pBest(:, p) = x_particles(:, p);
                    Q_particles_opt(p) = Q;
                    if (Q_particles_opt(p) <= Q_opt)
                        gBest = pBest(:, p);
                        gBestId = p;
                        Q_opt = Q_particles_opt(p);
                    end
                end
                fprintf('Initialisation stage. Particle: %d, Q: %.4f, Q_opt: %.4f, gBest particle %d\n', p, Q, Q_opt, gBestId);
            end
            
            % Run for the iterations
            for it = 1:obj.maxIterations
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
                        if (Q_particles_opt(p) <= Q_opt)
                            gBest = pBest(:, p);
                            gBestId = p;
                            Q_opt = Q_particles_opt(p);
                        end
                    end
                    
                    fprintf('Iteration: %d, particle: %d, Q: %.4f, Q_opt: %.4f, gBest particle %d\n', it, p, Q, Q_opt, gBestId);
                end
            end
            x_opt = gBest;
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

