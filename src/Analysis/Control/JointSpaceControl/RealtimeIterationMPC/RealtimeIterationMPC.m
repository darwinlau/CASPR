% Nonlinear Model predictive control using realtime iteration solving
% scheme, i.e. perform one SQP iteration in each control cycle
%
% Author        : Chen SONG
% Created       : 2018
% Description    :
%   The controller uses CASPR built-in linearized model. RK4 algorithm used
%   for integration. 
%   The inputs are:
%       refTrajObj:     the CASPR reference class object
%       N:              the prediction horizon
%       Wf, Wp, Wv:     weight matrices on cable forces, position errors and
%                       velocity errors
%       deltaT:         the control interval
%       terminalSetDef: terminal set (a vector represents the radius of the
%                       terminal set box)
%       terminalWeight: terminal weight
%       opt_type:       1 - a single shooting approach
%                       2 - a multiple shooting approach

classdef RealtimeIterationMPC < ControllerBase

    properties (SetAccess = private)
        % optimization formulation type
        opt_type
        
        % control parameters
        N               % the prediction horizon
        Wf              % the weighting matrix for (current and predicted) cable forces
        Wp              % the weighting matrix for (current and predicted) joint poses
        Wv              % the weighting matrix for (current and predicted) joint velocities
        deltaT          % control interval
        
        % model related parameters
        fmin
        fmax
        numDofs
        numActuators

        % intermediate variables (for MPC formulation)
        Q_hat           % the complete predicted poses in a cell array format
        Q_dot_hat       % the complete predicted pose velocities in a cell array format
        F_hat           % the complete predicted cable forces in a cell array format
        Q_ref           % the complete reference poses in a cell array format
        Q_dot_ref       % the complete reference pose velocities in a cell array format
        q_ddot_ref      % the last reference acceleration of the previous horizon, for local controller
        E_hat           % the complete predicted pose errors in a cell array format
        E_dot_hat       % the complete predicted pose velocity errors in a cell array format
        Q_residue       % f(x_hat_i) - x_hat_i+1
        Q_dot_residue

        % linearized models (using the last predicted horizon of q and q_dot)
        %   x^+ = A * x + B * u
        %   x = [dq; dq_dot]
        %   u = df 
        %   dq = q - q_hat, dq_dot = q_dot - q_dot_hat, df = f - f_hat
        A_hat           % a cell array holds the A matrices for the whole horizon. Linearization references are the last predicted states
        B_hat           % a cell array holds the B matrices for the whole horizon. Linearization references are the last predicted states

        opt_options     % optimization option
        refTrajObj      % the reference trajectory object
        
        terminalSetDef  % the size of terminal set, a vector of length 2*numDofs
        terminalWeight  % defines the terminal cost, a diagonal matrix with a dim of 2*numDofs
        use_terminal_set
        use_terminal_cost
    end

    methods
        % A constructor for a computed torque controller.
        function c = RealtimeIterationMPC(dynModel, refTrajObj, N, Wf, Wp, Wv, deltaT, terminalSetDef, terminalWeight, opt_type)
            c@ControllerBase(dynModel);
            
%             c.fdObj = ForwardDynamics(FDSolverType.ODE4);
            
            c.refTrajObj = refTrajObj;
            c.N = N;
            c.Wf = Wf;
            c.Wp = Wp;
            c.Wv = Wv;
            c.deltaT = deltaT;
            c.opt_type = opt_type;
            
            c.fmin          =   c.dynModel.actuationForcesMin;
            c.fmax          =   c.dynModel.actuationForcesMax;
            c.numDofs       =   c.dynModel.numDofs;
            c.numActuators     =   c.dynModel.numActuators;
            
            c.Q_hat         =   cell(N+1, 1);
            c.Q_dot_hat     =   cell(N+1, 1);
            c.Q_ref         =   cell(N, 1);
            c.Q_dot_ref     =   cell(N, 1);
            c.F_hat         =   cell(N, 1);
            c.A_hat         =   cell(N, 1);
            c.B_hat         =   cell(N, 1);
            c.E_hat         =   cell(N, 1);
            c.E_dot_hat     =   cell(N, 1);
            c.Q_residue     =   cell(N, 1);
            c.Q_dot_residue =   cell(N, 1);

            c.opt_options = optimoptions('quadprog', 'Display', 'off', 'MaxIter', 100, 'ConstraintTolerance', 1e-8);
            
            % derive the prediction for the first horizon (the reference will be used)
            f_prev = c.fmin;
            for i=1:N-1
                q_current       =   c.refTrajObj.q{i};
                q_dot_current   =   c.refTrajObj.q_dot{i};
                q_ddot_current  =   c.refTrajObj.q_ddot{i};
                c.Q_hat{i}      =   q_current;
                c.Q_dot_hat{i}  =   q_dot_current;
                c.Q_ref{i}      =   q_current;
                c.Q_dot_ref{i}  =   q_dot_current;
                c.dynModel.update(q_current, q_dot_current, zeros(c.numDofs, 1), zeros(c.numDofs, 1));
                H       =   eye(c.numActuators);
                f       =   zeros(c.numActuators, 1);
                A       =   [];
                b       =   [];
                A_eq    =   [-(c.dynModel.L_active)', c.dynModel.A];
                b_eq    =   c.dynModel.M*q_ddot_current + c.dynModel.C + c.dynModel.G;
                x_lb    =   c.fmin;
                x_ub    =   c.fmax;
                x_0     =   f_prev;
                
                
                [f_prev, ~, exitflag] = quadprog(H, f, A, b, A_eq, b_eq, x_lb, x_ub, x_0, c.opt_options);
                
                if exitflag == -2
                    CASPR_log.Warn('Reference infeasible: MPC controller construction failed.');
                else
                    c.F_hat{i} = f_prev;
                end
            end
            c.Q_hat{N}      =   c.refTrajObj.q{N};
            c.Q_dot_hat{N}  =   c.refTrajObj.q_dot{N};
            c.Q_ref{N}      =   c.refTrajObj.q{N};
            c.Q_dot_ref{N}  =   c.refTrajObj.q_dot{N};
            c.q_ddot_ref   	=   c.refTrajObj.q_ddot{N};
            
            % terminal set/cost definition
            if isempty(terminalSetDef) || length(terminalSetDef) ~= 2*c.numDofs
                CASPR_log.Warn('No terminal set used');
                c.use_terminal_set = false;
            else
                c.use_terminal_set = true;
                c.terminalSetDef = terminalSetDef;
            end
            if isempty(terminalWeight) || length(diag(terminalWeight)) ~= 2*c.numDofs
                CASPR_log.Warn('No terminal cost used');
                c.use_terminal_cost = false;
            else
                c.use_terminal_cost = true;
                c.terminalWeight = terminalWeight;
            end
        end
        
        % function that extends the previous predicted horizon into a guess
        % for the current control cycle. The purpose is to derive a set of
        % reference for model linearization
        function predictionExtender(obj)
            % step 1: extend the predicted control input
                % method 1: use the last control input from the previous
                % prediction horizon as an estimation.
                obj.F_hat{obj.N} = obj.F_hat{obj.N-1};
                % method 2: run a terminal local controller to get a control
                % TODO
                
            % step 2: extend the predicted state (with a very simple double
            % integrator model)
            obj.dynModel.update(obj.Q_hat{obj.N}, obj.Q_dot_hat{obj.N}, zeros(obj.numDofs, 1), zeros(obj.numDofs, 1));
            obj.dynModel.actuationForces = obj.F_hat{obj.N};
            qddot = obj.dynModel.q_ddot_dynamics;
            obj.Q_hat{obj.N+1} = obj.Q_hat{obj.N} + obj.deltaT*obj.Q_dot_hat{obj.N} + 0.5*qddot*obj.deltaT^2;
            obj.Q_dot_hat{obj.N+1} = obj.Q_dot_hat{obj.N} + obj.deltaT*qddot;
        end
        
        % the function to perform discretization and derive the linearized 
        % models for every point on the prediction horizon using the last 
        % predicted trajectory.
        % Hence the last predicted trajectory should be ready before this
        % function call.
        function linearizedModelDerivation(obj)
            Ns = 1;
            for i = 1: obj.N
                x = [obj.Q_hat{i}; obj.Q_dot_hat{i}];
                u = obj.F_hat{i};
                [x, obj.A_hat{i}, obj.B_hat{i}] = obj.ode4AutoDiff(x, u, obj.deltaT, Ns);
                obj.Q_residue{i} = x(1:obj.dynModel.numDofs) - obj.Q_hat{i+1};
                obj.Q_dot_residue{i} = x(1+obj.dynModel.numDofs:2*obj.dynModel.numDofs) - obj.Q_dot_hat{i+1};
            end
        end
        
        % function to derive the reference value for the current horizon.
        % this function exists to deal with the problem with the end of the
        % trajectory and also adding delay compensating shift. Because it
        % also uses the extended previous prediction to derive the
        % predicted error, it should be called after the linearized model
        % derivation function where the horizon will be extended.
        function referenceGenerator(obj, t_index)
            % the delay compensating shift
            ds = 1;
            
            index_max = length(obj.refTrajObj.q);
            
            for i = 1:obj.N
                current_index = t_index + i - 1 + ds;
                if current_index > index_max
                    obj.Q_ref{i} = obj.refTrajObj.q{index_max};
                    obj.Q_dot_ref{i} = obj.refTrajObj.q_dot{index_max};
                else
                    obj.Q_ref{i} = obj.refTrajObj.q{current_index};
                    obj.Q_dot_ref{i} = obj.refTrajObj.q_dot{current_index};
                end
                obj.E_hat{i} = obj.Q_hat{i+1} - obj.Q_ref{i};
                obj.E_dot_hat{i} = obj.Q_dot_hat{i+1} - obj.Q_dot_ref{i};
            end
            obj.q_ddot_ref   	=   obj.refTrajObj.q_ddot{obj.N};
        end
        
        
        function [H, f, A_eq, b_eq, A, b, x_lb, x_ub, x0] = formulateMPC(obj, t_index, q, q_dot)
            
            A = [];
            b = [];
            
            obj.predictionExtender();
            obj.linearizedModelDerivation();
            % generate reference traj as well as error to last predicted
            % traj
            obj.referenceGenerator(t_index);
            dx0 = [q; q_dot] - [obj.Q_hat{1}; obj.Q_dot_hat{1}];
            Wx = blkdiag(obj.Wp, obj.Wv);
            Wu = obj.Wf;
            if (obj.opt_type == 1)
                % option 1: single shooting approach
                % optimizing variables only contain control input
                % deviations
                H = zeros(obj.N*obj.numActuators);
                f = zeros(obj.N*obj.numActuators, 1);
                A_eq = [];
                b_eq = [];
                x_lb = zeros(obj.N*obj.numActuators, 1);
                x_ub = zeros(obj.N*obj.numActuators, 1);
                x0 = zeros(obj.N*obj.numActuators, 1);
                
                transferMat = zeros(2*obj.numDofs, obj.N*obj.numActuators);
                accumulatedA = eye(2*obj.numDofs);
                remainderVec = zeros(2*obj.numDofs, 1);
                predictionError = zeros(2*obj.numDofs, 1);
                for i = 1:obj.N
                    intval_i = (i-1)*obj.numActuators+1 : i*obj.numActuators;
                    
                    % update the objective function
                    predictionError = obj.A_hat{i}*predictionError + [obj.Q_residue{i}; obj.Q_dot_residue{i}];
                    accumulatedA = obj.A_hat{i}*accumulatedA;
                    transferMat = obj.A_hat{i}*transferMat;
                    transferMat(1:2*obj.numDofs, intval_i) = obj.B_hat{i};
                    remainderVec = accumulatedA*dx0 + predictionError + [obj.E_hat{i}; obj.E_dot_hat{i}];
                    tmp = transferMat'*Wx;
                    H = H + tmp*transferMat;
                    f = f + tmp*remainderVec;
                    f(intval_i) = f(intval_i) + Wu*obj.F_hat{i};
                    H(intval_i,intval_i) = H(intval_i,intval_i) + Wu;
                    
                    % update the lower/upper bounds
                    x_lb(intval_i) = obj.fmin - obj.F_hat{i};
                    x_ub(intval_i) = obj.fmax - obj.F_hat{i};
                end
                if obj.use_terminal_cost
                    tmp = transferMat'*(obj.terminalWeight-Wx);
                    H = H + tmp*transferMat;
                    f = f + tmp*remainderVec;
                end
                if obj.use_terminal_set
                    A = zeros(4*obj.numDofs, obj.N*obj.numActuators);
                    b = zeros(4*obj.numDofs, 1);
                    A(1:2*obj.numDofs, :) = transferMat;
                    A(1+2*obj.numDofs:4*obj.numDofs, :) = -transferMat;
                    
                    b(1:2*obj.numDofs) = obj.terminalSetDef - (accumulatedA*dx0 + predictionError + [obj.E_hat{obj.N}; obj.E_dot_hat{obj.N}]);
                    b(1+2*obj.numDofs:4*obj.numDofs) = obj.terminalSetDef + (accumulatedA*dx0 + predictionError + [obj.E_hat{obj.N}; obj.E_dot_hat{obj.N}]);
                    
                end
            elseif (obj.opt_type == 2)
                % option 2: multiple shooting approach
                % optimizing variables contain control input as well as
                % state deviations.
                % the optimizing variables goes like: [u0; x1; u1; x2; ...; uN-1; xN]
                H = zeros(obj.N*(obj.numActuators+2*obj.numDofs));
                f = zeros(obj.N*(obj.numActuators+2*obj.numDofs), 1);
                A_eq = zeros(obj.N*2*obj.numDofs, obj.N*(obj.numActuators+2*obj.numDofs));
                b_eq = zeros(obj.N*2*obj.numDofs, 1);
                x_lb = zeros(obj.N*(obj.numActuators+2*obj.numDofs), 1);
                x_ub = zeros(obj.N*(obj.numActuators+2*obj.numDofs), 1);
                x0 = zeros(obj.N*(obj.numActuators+2*obj.numDofs), 1);
                
                W = blkdiag(Wu, Wx);
                for i = 1:obj.N
                    if i==1
                        intval_cs_i = (i-1)*(obj.numActuators+2*obj.numDofs)+1 : i*(obj.numActuators+2*obj.numDofs);
                        intval_s_i = (i-1)*2*obj.numDofs+1 : i*2*obj.numDofs;
                    else
                        intval_cs_i = (i-1)*(obj.numActuators+2*obj.numDofs)+1 : i*(obj.numActuators+2*obj.numDofs);
                        intval_s_i = (i-1)*2*obj.numDofs+1 : i*2*obj.numDofs;
                        intval_scs_i = (i-2)*(obj.numActuators+2*obj.numDofs)+1+obj.numActuators : (i-2)*(obj.numActuators+2*obj.numDofs)+2*obj.numActuators+4*obj.numDofs;
                    end
                    
                    % update the objective function
                    H(intval_cs_i, intval_cs_i) = W;
%                     f(intval_cs_i) = W*[zeros(obj.numActuators, 1); obj.E_hat{i}; obj.E_dot_hat{i}];
                    f(intval_cs_i) = W*[obj.F_hat{i}; obj.E_hat{i}; obj.E_dot_hat{i}];
                    
                    % update the equality constraints
                    if i==1
                        A_eq(intval_s_i, intval_cs_i) = [obj.B_hat{i}, -eye(2*obj.numDofs)];
                        b_eq(intval_s_i) = -obj.A_hat{i}*dx0 - [obj.Q_residue{i}; obj.Q_dot_residue{i}];
                    else
                        A_eq(intval_s_i, intval_scs_i) = [obj.A_hat{i}, obj.B_hat{i}, -eye(2*obj.numDofs)];
                        b_eq(intval_s_i) =  - [obj.Q_residue{i}; obj.Q_dot_residue{i}];
                    end
                    
                    flag_confine_state_change = false;
                    if flag_confine_state_change
                        delta_pos = 0.05;
                        delta_vel = 0.05;
                        % update the lower/upper bounds
                        x_lb(intval_cs_i) = [obj.fmin - obj.F_hat{i}; -delta_pos*ones(obj.numDofs, 1); -delta_vel*ones(obj.numDofs, 1)];
                        x_ub(intval_cs_i) = [obj.fmax - obj.F_hat{i}; delta_pos*ones(obj.numDofs, 1); delta_vel*ones(obj.numDofs, 1)];
                    else
                        % update the lower/upper bounds
                        x_lb(intval_cs_i) = [obj.fmin - obj.F_hat{i}; -inf*ones(2*obj.numDofs, 1)];
                        x_ub(intval_cs_i) = [obj.fmax - obj.F_hat{i}; inf*ones(2*obj.numDofs, 1)];
                    end
                end
                
                if obj.use_terminal_cost
                    intval_s_N = (obj.N-1)*(obj.numActuators+2*obj.numDofs)+obj.numActuators+1 : obj.N*(obj.numActuators+2*obj.numDofs);
                    W = obj.terminalWeight;
                    % update the objective function
                    H(intval_s_N, intval_s_N) = W;
                    f(intval_cs_N) = W*[obj.E_hat{obj.N}; obj.E_dot_hat{obj.N}];
                end
                if obj.use_terminal_set
                    intval_s_N = (obj.N-1)*(obj.numActuators+2*obj.numDofs)+obj.numActuators+1 : obj.N*(obj.numActuators+2*obj.numDofs);
                    A = zeros(4*obj.numDofs, obj.N*(obj.numActuators+2*obj.numDofs));
                    b = zeros(4*obj.numDofs, 1);
                    A(1:2*obj.numDofs, intval_s_N) = eye(2*obj.numDofs);
                    A(1+2*obj.numDofs:4*obj.numDofs, intval_s_N) = -eye(2*obj.numDofs);
                    
                    b(1:2*obj.numDofs) = obj.terminalSetDef - [obj.E_hat{obj.N}; obj.E_dot_hat{obj.N}];
                    b(1+2*obj.numDofs:4*obj.numDofs) = obj.terminalSetDef + [obj.E_hat{obj.N}; obj.E_dot_hat{obj.N}];
                    
                end
                
            else
                CASPR_log.Error('Unrecognized optimization type.');
            end
            
        end

        % The implementation of the abstract executeFunction for the
        % controller class.
        function [f_active, result_model, exit_flag] = executeFunction(obj, q, q_d, ~, ~, ~, ~, t_index)
            
            [H, f, A_eq, b_eq, A, b, x_lb, x_ub, x_0] = obj.formulateMPC(t_index, q, q_d);
            H = H/(obj.N*1.0);
            f = f/(obj.N*1.0);
            [x_res, ~, exit_flag] = quadprog(H, f, A, b, A_eq, b_eq, x_lb, x_ub, x_0, obj.opt_options);
            
            if exit_flag == -2
                CASPR_log.Error(strcat('Controller infeasible at time ', num2str(obj.refTrajObj.timeVector(t_index))));
            else
                if exit_flag ~= 1
                    wtfman = true
                end
                % result treatment: since two optimization formulations
                % (single-shooting and multiple shooting) are different in
                % terms of optimizing variables, they will be treated
                % differently.
                % details:
                %   1. N-1 future control inputs will be saved to obj.F_hat
                %   2. N future state predictions will be saved to
                %   obj.Q_hat and obj.Q_dot_hat
                if (obj.opt_type == 1)
                    % force command (the first control input on the
                    % horizon)
                    f_active = obj.F_hat{1} + x_res(1:obj.numActuators);
                    % initial state deviation
                    dx_prev = [q; q_d] - [obj.Q_hat{1}; obj.Q_dot_hat{1}];
                    for i=1:obj.N
                        intval_c_i = (i-1)*obj.numActuators+1 : i*obj.numActuators;
                        % propogate the state deviation (using the previous
                        % one to derive the current one)
                        dx_prev = obj.A_hat{i}*dx_prev + obj.B_hat{i}*x_res(intval_c_i) + [obj.Q_residue{i}; obj.Q_dot_residue{i}];
                        % add the current state deviation and the last
                        % predicted state for the current time instant to
                        % derive an updated predicted state
                        obj.Q_hat{i} = obj.Q_hat{i+1} + dx_prev(1:obj.numDofs);
                        obj.Q_dot_hat{i} = obj.Q_dot_hat{i+1} + dx_prev(obj.numDofs+1:2*obj.numDofs);
                        % derive an updated predicted control input
                        if i >= 2
                            obj.F_hat{i-1} = obj.F_hat{i} + x_res(intval_c_i);
                        end
                    end
                elseif (obj.opt_type == 2)
                    f_active = obj.F_hat{1} + x_res(1:obj.numActuators);
                    % initial state deviation
                    for i = 1:obj.N
                        intval_s1_i = (i-1)*(2*obj.numDofs+obj.numActuators)+obj.numActuators+1:(i-1)*(2*obj.numDofs+obj.numActuators)+obj.numActuators+obj.numDofs;
                        intval_s2_i = (i-1)*(2*obj.numDofs+obj.numActuators)+obj.numDofs+obj.numActuators+1:i*(2*obj.numDofs+obj.numActuators);
                        intval_c_i = (i-1)*(2*obj.numDofs+obj.numActuators)+1:(i-1)*(2*obj.numDofs+obj.numActuators)+obj.numActuators;
                        obj.Q_hat{i} = obj.Q_hat{i+1} + x_res(intval_s1_i);
                        obj.Q_dot_hat{i} = obj.Q_dot_hat{i+1} + x_res(intval_s2_i);
                        if i >= 2
                            obj.F_hat{i-1} = obj.F_hat{i} + x_res(intval_c_i);
                        end
                    end
                else
                    CASPR_log.Error('Unrecognized optimization type.');
                end
                obj.dynModel.actuationForces = f_active;
                obj.dynModel.update(q, q_d, zeros(obj.numDofs), zeros(obj.numDofs));
                result_model = obj.dynModel;
            end
        end
        
        
        %% numerical tools
        % RK-4 with automatic differentiator
        function [x_n, A, B] = ode4AutoDiff(obj, x, u, T, Ns)
            x_n = x;
            A = eye(length(x));
            B = zeros(length(x),length(u));
            Ts = T/Ns;
            for n = 1:Ns
                % the 1-st RK cycle
                k1 = obj.eom(x_n, u);
                [A_tmp, B_tmp] = obj.dynModel.getLinearisedModel();
                dk1 = A_tmp*[A, B] + [zeros(size(A)), B_tmp];
                
                % the 2-nd RK cycle
                k2 = obj.eom(x_n+0.5*Ts*k1, u);
                [A_tmp, B_tmp] = obj.dynModel.getLinearisedModel();
                dk2 = A_tmp*([A, B]+0.5*Ts*dk1) + [zeros(size(A)), B_tmp];
                
                % the 3-rd RK cycle
                k3 = obj.eom(x_n+0.5*Ts*k2, u);
                [A_tmp, B_tmp] = obj.dynModel.getLinearisedModel();
                dk3 = A_tmp*([A, B]+0.5*Ts*dk2) + [zeros(size(A)), B_tmp];
                
                % the 4-th RK cycle
                k4 = obj.eom(x_n+Ts*k3, u);
                [A_tmp, B_tmp] = obj.dynModel.getLinearisedModel();
                dk4 = A_tmp*([A, B]+0.5*Ts*dk3) + [zeros(size(A)), B_tmp];
                
                % update the subsequent state and matrices A, B
                x_n = x_n + Ts/6*(k1+2*k2+2*k3+k4);
                AB = [A,B] + Ts/6*(dk1+2*dk2+2*dk3+dk4);
                A = AB(1:size(A,1),1:size(A,2));
                B = AB(1:size(B,1),1+size(A,2):end);
            end
        end
        
        % get the eom using caspr model
        function x_dot = eom(obj, x, u)
            q = x(1:obj.dynModel.numDofs);
            q_d = x(obj.dynModel.numDofs+1:end);
            zerovec = zeros(obj.dynModel.numDofs,1);
            obj.dynModel.actuationForces = u;
            obj.dynModel.update(q, q_d, zerovec, zerovec);
            x_dot = zeros(size(x));
            x_dot(1:obj.dynModel.numDofs) = obj.dynModel.q_deriv;
            x_dot(obj.dynModel.numDofs+1:2*obj.dynModel.numDofs) = obj.dynModel.q_ddot_dynamics;
        end
    end
end
