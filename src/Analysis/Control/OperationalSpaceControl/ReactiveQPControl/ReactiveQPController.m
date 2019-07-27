% Operational Space Reactive QP Controller
%
% Author        : Dominic Chan
% Created       : 2019
% Description   : Reactive QP Controller for Operational Space Control
% - Kp, Kd      : control gains
% - u           : null space exploration parameter (n + 2)

classdef ReactiveQPController < ControllerBase
    properties (SetAccess = private)   
        % Operational space
        op_space        % Operational space of the CDPR    
        
        % Time step
        dt              % Time step
        
        % Tracking Function
        trackFun        % Function handler for tracking function
        Kp              % The P Gain of the error dynamics
        Kd              % The D Gain of the error dynamics  
        W_q             % Tracking weight
        W_q_inv         % Tracking weight inverse
        
        ori_error_array
        
        % Actuation Function
        W_a             % Actuation weight
        
        % Null space exploration
        u               % Null space exploration parameter
        d               % Null space diagonal vector
        alpha           % Scaling factor for actuation
        beta            % Scaling factor for avoidance  
        
        % Avoidance class
        avoidance_array % Array of avoidance classes
        
        % Others
        f_prev          % Stores the last feasible force command    
    end
    
    properties (Constant)
        qp_options = optimoptions('quadprog','Display','off',...
                'ConstraintTolerance',1e-6,'OptimalityTolerance',1e-6);      
    end

    methods
        % Constructor 
        function c = ReactiveQPController(dynModel, dt, Kp, Kd, d, alpha, beta, avoidance_array)
            c@ControllerBase(dynModel);
            c.dt = dt;
            c.Kp = Kp;
            c.Kd = Kd;                           
            c.avoidance_array = avoidance_array;            
            c.setu([d; log(alpha); log(beta)]);
            c.initVariables();
            c.getOpSpace();
        end
        
        % Initialise variables
        function initVariables(obj)
            obj.W_q         = eye(obj.dynModel.numDofs);
            obj.W_q_inv     = eye(obj.dynModel.numDofs);
            obj.W_a         = eye(obj.dynModel.numActuatorsActive);
            obj.f_prev      = zeros(obj.dynModel.numCables, 1);  
        end
        
        % Identify operational space
        function getOpSpace(obj)
            % Only one op space is supported for now
            body_index = obj.dynModel.bodyModel.operationalSpaceBodyIndices;
            op_space_body = obj.dynModel.bodyModel.bodies{body_index};
            obj.op_space = op_space_body.operationalSpace;
            
            % Set function handler for different types of op space
            if isa(obj.op_space, 'OperationalPosition')
                obj.trackFun = @obj.trackPosition;
            elseif isa(obj.op_space, 'OperationalOrientationEulerXYZ')
                n_ori = sum(sum(obj.op_space.selection_matrix(:,4:6)));   
                CASPR_log.Assert(n_ori==3, ...
                    sprintf('All 3 DoFs for orientation should be defined. (Only %d has been defined now)',n_ori));
                obj.trackFun = @obj.trackOrientationEulerXYZ;
            elseif isa(obj.op_space, 'OperationalPoseEulerXYZ')
                n_ori = sum(sum(obj.op_space.selection_matrix(:,4:6)));   
                CASPR_log.Assert(n_ori==3, ...
                    sprintf('All 3 DoFs for orientation should be defined. (Only %d has been defined now)',n_ori));
                obj.trackFun = @obj.trackPoseEulerXYZ;
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Different types of tracking function for
        % different types of operational space.
        % This reduces time for checking class type 
        % using 'isa' every time step.
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function q_dd_hat_T = trackPosition(obj, ~, q_d, y_ref, y_ref_d, y_ref_dd)
            % Tracking Function g_t
            J = obj.dynModel.J;
            J_w_pinv = obj.W_q_inv*J'*inv(J*obj.W_q_inv*J');
            b = y_ref_dd - obj.dynModel.J_dot*q_d - obj.Kd*(J*q_d - y_ref_d) -...
                obj.Kp*(obj.dynModel.y - y_ref);
            
            % Null space exploration
            D = diag(obj.d);
            q_dd_hat_T = J_w_pinv*b + (eye(obj.dynModel.numDofs) - J_w_pinv*J)*D*J_w_pinv*b;
        end
        
        function q_dd_hat_T = trackOrientationEulerXYZ(obj, ~, q_d, y_ref, y_ref_d, ~)
            % Error
            R_d = eul2rotm(y_ref', 'XYZ');
            R_y = eul2rotm(obj.dynModel.y', 'XYZ');
            R_error = R_d*R_y';
            e_q = Quaternion.FromRotationMatrix(R_error);
            e_q_vector = e_q.toVector();
            ori_error = e_q_vector(2:4);
            
            % Error dot
            ori_ref_next_est = y_ref + y_ref_d*obj.dt;
            R_d_next = eul2rotm(ori_ref_next_est','XYZ');
            R_diff = R_d_next*R_d';
            w_ref_axang = rotm2axang(R_diff)';
            w_ref = w_ref_axang(4)/obj.dt*w_ref_axang(1:3);

            % - Calculate quaternion ref dot
            this_quad       = Quaternion.FromRotationMatrix(R_y);
            this_quad_ref   = Quaternion.FromRotationMatrix(R_d);
            this_quad       = this_quad.toVector();
            this_quad_ref   = this_quad_ref.toVector();
            B_q = [-this_quad(2:4)';
                this_quad(1)*eye(3) - MatrixOperations.SkewSymmetric(this_quad(2:4))];
            B_q_ref = [-this_quad_ref(2:4)';
                this_quad_ref(1)*eye(3) - MatrixOperations.SkewSymmetric(this_quad_ref(2:4))];
            ori_ref_d = B_q_ref*w_ref;              
            
            % Control law          
            J = obj.dynModel.J;
            D = diag(obj.d);
            J_w_pinv = obj.W_q_inv*J'/(J*obj.W_q_inv*J');            
            
            % - Orientation
            b = pinv(B_q)*(ori_ref_d + 2*diag([0;diag(obj.Kp)])*[0;ori_error]);            
            q_d_hat_T = J_w_pinv*b + (eye(obj.dynModel.numDofs) - J_w_pinv*J)*D*J_w_pinv*b;
            q_dd_hat_T = (q_d_hat_T - q_d)/obj.dt;
        end
        
        function q_dd_hat_T = trackPoseEulerXYZ(obj, ~, q_d, y_ref, y_ref_d, y_ref_dd)
            % Select position and orientation references
            S_vec       = sum(obj.op_space.selection_matrix,1);  
            n_pos       = sum(S_vec(1:3));
            
            pos_ref     = y_ref(1:n_pos);
            pos_ref_d   = y_ref_d(1:n_pos);
            pos_ref_dd  = y_ref_dd(1:n_pos);                         
            ori_ref     = y_ref(n_pos+1:end);
            ori_ref_d   = y_ref_d(n_pos+1:end);
            
            % Control parameters
            Kp_pos      = obj.Kp(1:n_pos,1:n_pos);
            Kp_ori      = obj.Kp(n_pos+1:end,n_pos+1:end);
            Kd_pos      = obj.Kd(1:n_pos,1:n_pos);            
            
            % Position errors
            pos_error = pos_ref - obj.dynModel.y(1:n_pos);
            y_d = obj.dynModel.y_dot;
            pos_error_d = pos_ref_d - y_d(1:n_pos);
            
            % Orientation error in quaternion
            R_d = eul2rotm(ori_ref', 'XYZ');
            R_y = eul2rotm(obj.dynModel.y(n_pos+1:end)', 'XYZ');
            R_error = R_d*R_y';
            e_q = Quaternion.FromRotationMatrix(R_error);
            e_q_vector = e_q.toVector();
            ori_error = e_q_vector(2:4);
            
            % - Estimate w_ref
            ori_ref_next_est = ori_ref + ori_ref_d*obj.dt;
            R_d_next = eul2rotm(ori_ref_next_est','XYZ');
            R_diff = R_d_next*R_d';
            w_ref_axang = rotm2axang(R_diff)';
            w_ref = w_ref_axang(4)/obj.dt*w_ref_axang(1:3);

            % - Calculate quaternion ref dot
            this_quad       = Quaternion.FromRotationMatrix(R_y);
            this_quad_ref   = Quaternion.FromRotationMatrix(R_d);
            this_quad       = this_quad.toVector();
            this_quad_ref   = this_quad_ref.toVector();
            B_q = [-this_quad(2:4)';
                this_quad(1)*eye(3) - MatrixOperations.SkewSymmetric(this_quad(2:4))];
            B_q_ref = [-this_quad_ref(2:4)';
                this_quad_ref(1)*eye(3) - MatrixOperations.SkewSymmetric(this_quad_ref(2:4))];
            ori_ref_d = B_q_ref*w_ref;              
      
            % Control law          
            J = obj.dynModel.J;
            D = diag(obj.d);
            J_w_pinv = obj.W_q_inv*J'/(J*obj.W_q_inv*J');
            J_w_pinv_pos = J_w_pinv(:,1:n_pos);
            J_w_pinv_ori = J_w_pinv(:,n_pos+1:end);
            
            % - Position
            b_pos = pos_ref_dd + Kd_pos*pos_error_d + Kp_pos*pos_error - obj.dynModel.J_dot(1:n_pos,:)*q_d;
            q_dd_hat_T_pos = J_w_pinv_pos*b_pos + (eye(obj.dynModel.numDofs) - J_w_pinv_pos*J(1:n_pos,:))*D*J_w_pinv_pos*b_pos;
                
            % - Orientation
            b_ori = pinv(B_q)*(ori_ref_d + 2*diag([0;diag(Kp_ori)])*[0;ori_error]);            
            q_d_hat_T_ori = J_w_pinv_ori*b_ori + (eye(obj.dynModel.numDofs) - J_w_pinv_ori*J(n_pos+1:end,:))*D*J_w_pinv_ori*b_ori;
            q_dd_hat_T_ori = (q_d_hat_T_ori - q_d)/obj.dt;
            
            % Combine position & orientation
            q_dd_hat_T = q_dd_hat_T_pos + q_dd_hat_T_ori;
        end
                
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % The implementation of the abstract executeFunction for the
        % controller class.
        function [cable_force_active, result_model, exitflag] = executeFunction(obj, q, q_d, ~, y_ref, y_ref_d, y_ref_dd, ~)
            % Trying to be neat        
            n_dofs = obj.dynModel.numDofs;            
            n_a = obj.dynModel.numActuatorsActive;            
            
            %%%%%%%%%%%%%%%%%%%%%
            % Objective Function
            %%%%%%%%%%%%%%%%%%%%%
            q_dd_hat_T = obj.trackFun(q, q_d, y_ref, y_ref_d, y_ref_dd);
            
            %%%%%%%%%%%%%%%%%%
            % Actuation Function g_f - Determined by obj.W_a
            %%%%%%%%%%%%%%%%%%
            
            %%%%%%%%%%%%%%%%%%
            % Avoidance Function g_a
            avoid_qdd_array = cell(size(obj.avoidance_array));
            avoid_Phi_array = cell(size(obj.avoidance_array));
            avoid_rho_array = cell(size(obj.avoidance_array));
            % Each avoidance class returns q_dd_hat_A (Objective Function),
            % Phi and rho (Hard constraints)
            for i = 1:length(obj.avoidance_array)
                [avoid_qdd_array{i},avoid_Phi_array{i},avoid_rho_array{i}] = ...
                    obj.avoidance_array{i}.avoid(q, q_d);
            end
            % Combine every q_dd_a,i (average for now)
            q_dd_hat_A = zeros(n_dofs,1);
            for i = 1:length(avoid_qdd_array)
                q_dd_hat_A = q_dd_hat_A + 1/n_dofs*avoid_qdd_array{i};
            end            
            
            %%%%%%%%%%%%%%%%%%
            % QP Weight
            H_qp = [eye(n_dofs) + obj.beta*eye(n_dofs), zeros(n_dofs, n_a);                
                zeros(n_a,n_dofs), obj.alpha*obj.W_a];
            f_qp = [-q_dd_hat_T - obj.beta*q_dd_hat_A; zeros(n_a, 1)];
            
            %%%%%%%%%%%%%%%%%%%%%
            % Constraints
            %%%%%%%%%%%%%%%%%%%%%
            % Equation of Motion
            b_eq_eom = obj.dynModel.C + obj.dynModel.G;
            A_eq_eom = [-obj.dynModel.M, -obj.dynModel.L', obj.dynModel.A];  
            
            % Hard constraints for avoidance
            A_ineq_avoid = zeros(length(avoid_Phi_array), n_dofs+n_a);
            b_ineq_avoid = zeros(length(avoid_Phi_array), 1);
            for i = 1:length(avoid_Phi_array)
                A_ineq_avoid(i,:)   = avoid_Phi_array{i};
                b_ineq_avoid(i)     = avoid_rho_array{i};
            end           
                        
            % QP          
            [x, ~, qpexitflag]  = quadprog(H_qp, f_qp, A_ineq_avoid, b_ineq_avoid,...
                A_eq_eom, b_eq_eom, [-Inf*ones(n_dofs,1); obj.dynModel.actuationForcesMin], ...
                [Inf*ones(n_dofs,1); obj.dynModel.actuationForcesMax], [], obj.qp_options);
            
            result_model = obj.dynModel;
            if qpexitflag ~= 1
                fprintf('Exit flag: %d\n', qpexitflag);                
                cable_force_active = obj.f_prev; 
                exitflag = ControllerExitType.NO_ERROR;
                if qpexitflag == -2
                    fprintf('Infeasible!\n'); 
                    exitflag = ControllerExitType.INFEASIBLE;
                    return;  
                end
            else 
                cable_force_active = x(n_dofs+1:end);               
                obj.f_prev = cable_force_active;
                exitflag = ControllerExitType.NO_ERROR;
            end             
        end 
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Getters
        function u = getu(obj)
            u = obj.u;
        end
        % Setters
        function setu(obj, u) 
            obj.u       = u;
            obj.d       = obj.u(1:end-2);            
            obj.alpha   = exp(obj.u(end-1));
            obj.beta    = exp(obj.u(end));
        end
        
        function setW_q(obj, W_q)
            obj.W_q = W_q;
            obj.W_q_inv = inv(W_q);
        end
        
        function setW_a(obj, W_a)
            obj.W_a = W_a;
        end
    end
    
    methods (Static)
        % Orientation error with two quaternions (Yuan 1988)
        function e = orientationErrorQuad(q, qd)
            q = [q.q0;q.q1;q.q2;q.q3];
            q_d = [qd.q0;qd.q1;qd.q2;qd.q3];
            
            M = MatrixOperations.SkewSymmetric(q_d(2:4));
            e = q_d(1)*q(2:4)-q(1)*q_d(2:4) + M*q(2:4);
        end
        % Orientation error and L-matrix with two euler angles (Yuan 1988)
        function [e, L] = orientationErrorEuler(o, od)
            % Current R
            Ry = eul2rotm(o', 'XYZ');
            % Desired R
            Rd = eul2rotm(od','XYZ');
            
            % Error
            e = 0.5*(cross(Ry(:,1),Rd(:,1)) + cross(Ry(:,2),Rd(:,2)) + ...
                cross(Ry(:,3),Rd(:,3)));
            
            L = -0.5*(MatrixOperations.SkewSymmetric(Rd(:,1))*MatrixOperations.SkewSymmetric(Ry(:,1)) + ...
                MatrixOperations.SkewSymmetric(Rd(:,2))*MatrixOperations.SkewSymmetric(Ry(:,2)) + ...
                MatrixOperations.SkewSymmetric(Rd(:,3))*MatrixOperations.SkewSymmetric(Ry(:,3)));
        end
    end
end
