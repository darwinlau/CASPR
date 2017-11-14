% Disturbance observver.
%
% Author        : Chen SONG
% Created       : 2017
% Description    :
%    Disturbance observer using ESO (extended state observer): the
%    disturbance is assumed to be in joint space acceleration form, and the
%    control input is assumed to be the cable forces

classdef DisturbanceObserverESO < ObserverBase

    properties (SetAccess = private)
        q_hat           % the estimated state
        q_d_hat         % the estimated state
        d_hat           % the estimated disturbance
        pole_vec        % pole of the observer error system
        NUM_DOFS
        alpha_1
        alpha_2
        alpha_3
    end

    methods
        % A constructor for a computed torque controller.
        function c = DisturbanceObserverESO(dyn_model, delta_t, pole_vec)
            c@ObserverBase(dyn_model, delta_t);
            c.pole_vec      =   pole_vec;
            c.NUM_DOFS      =   dyn_model.numDofs;
            c.q_hat         =   zeros(dyn_model.numDofs, 1);
            c.q_d_hat       =   zeros(dyn_model.numDofs, 1);
            c.d_hat         =   zeros(dyn_model.numDofs, 1);
            c.alpha_1       =   3*pole_vec;
            c.alpha_2       =   3*pole_vec.*pole_vec;
            c.alpha_3       =   pole_vec.*pole_vec.*pole_vec;
        end

        % The implementation of the abstract executeFunction for the
        % controller class.
        function [q_est, q_dot_est, q_ddot_disturbance_est, wrench_disturbance_est] = executeFunction(obj, q, q_d, u, w_ext_active, ~)
            % the control u here for CDPRs is assumed be the control wrench
            
            if obj.first_time == true
                % this is effectively the initialization of the observer
                obj.first_time = false;
                obj.q_hat = q;
                obj.q_d_hat = q_d;
                [q_dot_hat, q_ddot_hat, disturbance_dot] = obj.observerDynmaticFunction(q, q_d, u, w_ext_active);
                obj.q_hat = obj.q_hat + obj.delta_t*q_dot_hat;
                obj.q_d_hat = obj.q_d_hat + obj.delta_t*q_ddot_hat;
                obj.d_hat = obj.d_hat + obj.delta_t*disturbance_dot;
                % do model update using current state feedback and fill up
                % the required outputs (this part will differ with
                % different observer)
                obj.dynModel.update(q, q_d, zeros(obj.NUM_DOFS,1), zeros(obj.NUM_DOFS,1));
                wrench_disturbance_est  =   obj.dynModel.M*obj.d_hat + obj.dynModel.C + obj.dynModel.G;
                q_ddot_disturbance_est  =   obj.dynModel.M\wrench_disturbance_est;
                q_est                   =   obj.q_hat;
                q_dot_est             	=   obj.q_d_hat;
            else                
                [q_dot_hat, q_ddot_hat, disturbance_dot] = obj.observerDynmaticFunction(q, q_d, u, w_ext_active);
                obj.q_hat = obj.q_hat + obj.delta_t*q_dot_hat;
                obj.q_d_hat = obj.q_d_hat + obj.delta_t*q_ddot_hat;
                obj.d_hat = obj.d_hat + obj.delta_t*disturbance_dot;
                % do model update using current state feedback and fill up
                % the required outputs (this part will differ with
                % different observer)
                obj.dynModel.update(q, q_d, zeros(obj.NUM_DOFS,1), zeros(obj.NUM_DOFS,1));
                wrench_disturbance_est  =   obj.dynModel.M*obj.d_hat + obj.dynModel.C + obj.dynModel.G;
                q_ddot_disturbance_est  =   obj.dynModel.M\wrench_disturbance_est;
                q_est                   =   obj.q_hat;
                q_dot_est             	=   obj.q_d_hat;
            end            
        end
        
        
        % among the inputs: the control u here for CDPRs is assumed be cable forces
        %                   q is used to derive the estimation error
        %                   w_ext_active is the expected output wrench
        % among the outputs: disturbance_dot is in the form of q_ddot
        function [q_dot_hat, q_ddot_hat, disturbance_dot] = observerDynmaticFunction(obj, q, ~, u, w_ext_active)
            
            % here update the model with the estimated variables
            obj.dynModel.update(obj.q_hat, obj.q_d_hat, zeros(obj.NUM_DOFS,1), zeros(obj.NUM_DOFS,1));
            
            e = (q - obj.q_hat);
            M = obj.dynModel.M;
%             C = obj.dynModel.C;
%             G = obj.dynModel.G;
            L = obj.dynModel.L;
            
            q_dot_hat           = obj.q_d_hat + diag(obj.alpha_1)*e;
%             q_ddot_hat          = M\(-C - G - w_ext_active - L'*u) + obj.d_hat + diag(obj.alpha_2)*e;
            q_ddot_hat          = obj.d_hat + diag(obj.alpha_2)*e + M\(-w_ext_active - L'*u);
            disturbance_dot     = diag(obj.alpha_3)*e;
        end
    end
end
