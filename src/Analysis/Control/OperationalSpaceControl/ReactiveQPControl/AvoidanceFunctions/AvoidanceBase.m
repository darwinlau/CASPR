% Base class for avoidance functions in reactive QP controller
%
% Author        : Dominic Chan
% Created       : 2019
% Description   : Responsible for both objective function and hard
% constraints
classdef AvoidanceBase < handle
    properties
        cdpr        % CDPR model
        dt          % Time step of trajectory
        k_q         % Scaling factor for grad_q
        epsilon     % Minimum increase for hard constraints
        k_qd        % Scaling factor for grad_qd
    end

    methods
        % Constructor
        function ab = AvoidanceBase(cdpr, dt, k_q, epsilon, k_qd)
            ab.cdpr     = cdpr;
            ab.dt       = dt;
            ab.k_q      = k_q;
            ab.epsilon  = epsilon;
            if nargin > 4
                ab.k_qd = k_qd;
            else
                ab.k_qd = [];
            end
        end     
        
        % Avoidance function q_dd_hat_A + Hard constraints \Phi, \rho
        function [q_dd_hat_A, Phi, rho] = avoid(obj, q, q_d)  
            % Init
            q_dd_hat_A = zeros(obj.cdpr.numDofs,1);
            Phi = zeros(1, obj.cdpr.numDofs+obj.cdpr.numActuatorsActive);
            rho = 0;
            obj.initVariables();
            
            % Check if avoidance is needed
            flag_obj = obj.isAvoidanceObjective(q, q_d);
            flag_con = obj.isAvoidanceConstraints(q, q_d);
            if flag_obj || flag_con
                [grad_q, grad_qd] = obj.gradient(q, q_d);
                                
                % Avoidance function  
                if flag_obj
                    if isempty(obj.k_qd)
                        q_dd_hat_A = obj.k_q/obj.dt*(grad_q' - q_d);
                    else
                        q_dd_hat_A = obj.k_qd*grad_qd' + obj.k_q/obj.dt*(grad_q' - q_d);
                    end    
                end

                % Hard constraints
                if flag_con
                    Phi = [-obj.dt*grad_q - grad_qd, zeros(1,obj.cdpr.numActuatorsActive)];
                    rho = grad_q*q_d - obj.epsilon;
                end
            end
        end               
        
        % Gradient (Default: numerical)
        % - Both grads are row vectors
        function [grad_q, grad_qd] = gradient(obj, q, q_d)            
            n_dofs = obj.cdpr.numDofs;           
            
            % Operating point
            h_0 = obj.evaluate(q, q_d);            
            dq = 1e-5;            
            % Grad wrt q
            grad_q = zeros(1, n_dofs);            
            for i = 1:n_dofs
                this_q = q;
                this_q(i) = this_q(i) + dq;                
                this_h = obj.evaluate(this_q, q_d);
                grad_q(i) = this_h - h_0;
            end
            grad_q = grad_q./dq;
            % Clear any disturbance made during evaluations
            obj.cdpr.update(q, q_d, zeros(obj.cdpr.numDofs,1), zeros(obj.cdpr.numDofs,1));
            obj.initVariables();
            
            % Grad wrt q_d
            grad_qd = zeros(1, n_dofs);          
            if ~isempty(obj.k_qd)
                for i = 1:n_dofs
                    this_qd = q_d;
                    this_qd(i) = this_qd(i) + dq;                
                    this_h = obj.evaluate(q, this_qd);
                    grad_qd(i) = this_h - h_0;
                end
                grad_qd = grad_qd./dq;
            end
            % Clear any disturbance made during evaluations
            obj.cdpr.update(q, q_d, zeros(obj.cdpr.numDofs,1), zeros(obj.cdpr.numDofs,1));
            obj.initVariables();            
        end        
    end

    methods (Abstract)       
        % Function Value
        value = evaluate(obj, q, q_d);  
        % Init variables before every new gradient estimation
        initVariables(obj);
        % Triggers
        flag  = isAvoidanceObjective(obj, q, q_d);
        flag  = isAvoidanceConstraints(obj, q, q_d);
    end    
end
