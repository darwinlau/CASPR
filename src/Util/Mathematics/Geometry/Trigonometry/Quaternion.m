% Quaternion representation and operations
%
% Author        : Darwin LAU
% Created       : 2014
% Description    :
classdef Quaternion 
    
    properties
        q0
        q1
        q2
        q3
    end
    
    methods
        function q = Quaternion(q0, q1, q2, q3)
            if nargin > 0
                q.q0 = q0;
                q.q1 = q1;
                q.q2 = q2;
                q.q3 = q3;
            end
        end
        
        function q_inv = inv(q)
            q_inv = Quaternion;
            q_conj = conj(q);
            q_norm = norm(q);
            if q_norm == 0
                q_inv.q0 = 0;
                q_inv.q1 = 0;
                q_inv.q2 = 0;
                q_inv.q3 = 0;
            else
                q_inv.q0 = q_conj.q0/q_norm^2;
                q_inv.q1 = q_conj.q1/q_norm^2;
                q_inv.q2 = q_conj.q2/q_norm^2;
                q_inv.q3 = q_conj.q3/q_norm^2;
            end
            % Why not q_inv = q_conj; Since always unit vector
            % Otherwise this is wrong anyway
        end
        
        
        function q = mtimes(qa, qb)
            q = Quaternion;
            if (isnumeric(qa))
                q.q0 = qa*qb.q0;
                q.q1 = qa*qb.q1;
                q.q2 = qa*qb.q2;
                q.q3 = qa*qb.q3;
            elseif (isnumeric(qb))
                q.q0 = qb*qa.q0;
                q.q1 = qb*qa.q1;
                q.q2 = qb*qa.q2;
                q.q3 = qb*qa.q3;
            else
                q.q0 = qa.q0*qb.q0 - qa.q1*qb.q1 - qa.q2*qb.q2 - qa.q3*qb.q3;
                q.q1 = qa.q0*qb.q1 + qa.q1*qb.q0 + qa.q2*qb.q3 - qa.q3*qb.q2;
                q.q2 = qa.q0*qb.q2 - qa.q1*qb.q3 + qa.q2*qb.q0 + qa.q3*qb.q1;
                q.q3 = qa.q0*qb.q3 + qa.q1*qb.q2 - qa.q2*qb.q1 + qa.q3*qb.q0;
            end
        end
        
        function q = plus(qa, qb)
            q = Quaternion(qa.q0+qb.q0, qa.q1+qb.q1, qa.q2+qb.q2, qa.q3+qb.q3);
        end
        
        function q = minus(qa, qb)
            q = Quaternion(qa.q0-qb.q0, qa.q1-qb.q1, qa.q2-qb.q2, qa.q3-qb.q3);
        end
        
        function n = norm(q)
            n = sqrt(q.q0^2 + q.q1^2 + q.q2^2 + q.q3^2);
        end
        
        function q_c = conj(q)
            q_c = Quaternion;
            q_c.q0 = q.q0;
            q_c.q1 = -q.q1;
            q_c.q2 = -q.q2;
            q_c.q3 = -q.q3;
        end
        
        function q_e = exp(q)
            e_p = exp(q.q0);
            v = [q.q1; q.q2; q.q3];
            q_0 = e_p * cos(norm(v));
            if (norm(v) ~= 0)
                q_1 = e_p * v(1) * sin(norm(v))/norm(v); 
                q_2 = e_p * v(2) * sin(norm(v))/norm(v); 
                q_3 = e_p * v(3) * sin(norm(v))/norm(v); 
            else
                q_1 = 0;
                q_2 = 0;
                q_3 = 0;
            end
            q_e = Quaternion(q_0, q_1, q_2, q_3);
        end
        
        function q_vec = toVector(obj)
            q_vec = [obj.q0; obj.q1; obj.q2; obj.q3];
        end
        
        function q_out = normalise(obj)
            if (abs(obj.q0) == 1)
                q_out = Quaternion(obj.q0, 0, 0, 0);
            else
                A = sqrt((obj.q1^2 + obj.q2^2 + obj.q2^2)/(1-obj.q0^2));
                q_out = Quaternion(obj.q0, obj.q1/A, obj.q2/A, obj.q3/A);
            end
        end
    end
    
    methods (Static)
        function q_0p = FromRotationMatrix(R_0p)
            q_0p = Quaternion;
            q_0p.q0 = 1/2*sqrt(1 + R_0p(1,1) + R_0p(2,2) + R_0p(3,3));
            q_0p.q1 = (R_0p(3,2) - R_0p(2,3))/(4*q_0p.q0);
            q_0p.q2 = (R_0p(1,3) - R_0p(3,1))/(4*q_0p.q0);
            q_0p.q3 = (R_0p(2,1) - R_0p(1,2))/(4*q_0p.q0);
        end
        
        function q = FromAxisAngle(a)
            q = Quaternion;
            q.q0 = cos(a.th/2);
            q.q1 = a.kx*sin(a.th/2);
            q.q2 = a.ky*sin(a.th/2);
            q.q3 = a.kz*sin(a.th/2);
        end
        
        function q_d = DerivativeFromAxisAngle(a, a_d)
            q_d = Quaternion;
            q_d.q0 = -a_d.th/2*sin(a.th/2);
            q_d.q1 = a.kx*a_d.th/2*cos(a.th/2);
            q_d.q2 = a.ky*a_d.th/2*cos(a.th/2);
            q_d.q3 = a.kz*a_d.th/2*cos(a.th/2);
        end
        
        function q_dd = DoubleDerivativeFromAxisAngle(a, a_d, a_dd)
            q_dd = Quaternion;
                
            q_dd.q0 = -a_dd.th/2*sin(a.th/2) - a_d.th^2/4*cos(a.th/2);
            q_dd.q1 = a.kx*(a_dd.th/2*cos(a.th/2) - a_d.th^2/4*sin(a.th/2));
            q_dd.q2 = a.ky*(a_dd.th/2*cos(a.th/2) - a_d.th^2/4*sin(a.th/2));
            q_dd.q3 = a.kz*(a_dd.th/2*cos(a.th/2) - a_d.th^2/4*sin(a.th/2));
        end
        
        % Rotation ^0_pR
        function R_0p = ToRotationMatrix(q)
            q_0 = q.q0;
            q_1 = q.q1;
            q_2 = q.q2;
            q_3 = q.q3;
            R_0p = [1-2*(q_2^2+q_3^2), 2*q_1*q_2-2*q_0*q_3, 2*q_0*q_2+2*q_1*q_3; ...
                2*q_1*q_2+2*q_0*q_3, 1-2*(q_1^2+q_3^2), -2*q_0*q_1+2*q_2*q_3; ...
                -2*q_0*q_2+2*q_1*q_3, 2*q_0*q_1+2*q_2*q_3, 1-2*(q_1^2+q_2^2)];
        end
        
        function R_0p_d = ToRotationMatrixDeriv(q, qd)
            q_0 = q.q0;
            q_1 = q.q1;
            q_2 = q.q2;
            q_3 = q.q3;
            q0_d = qd.q0;
            q1_d = qd.q1;
            q2_d = qd.q2;
            q3_d = qd.q3;
            R_0p_d = [-4*q2_d*q_2-4*q3_d*q_3, 2*q1_d*q_2+2*q_1*q2_d-2*q0_d*q_3-2*q_0*q3_d, 2*q0_d*q_2+2*q_0*q2_d+2*q1_d*q_3+2*q_1*q3_d; ...
                2*q1_d*q_2+2*q_1*q2_d+2*q0_d*q_3+2*q_0*q3_d, -4*q1_d*q_1-4*q3_d*q_3, -2*q0_d*q_1-2*q_0*q1_d+2*q2_d*q_3+2*q_2*q3_d; ...
                -2*q0_d*q_2-2*q_0*q2_d+2*q1_d*q_3+2*q_1*q3_d, 2*q0_d*q_1+2*q_0*q1_d+2*q2_d*q_3+2*q_2*q3_d, -4*q1_d*q_1-4*q2_d*q_2];
        end
        
        function R_0p_dd = ToRotationMatrixDoubleDeriv(q, qd, qdd)
            q_0 = q.q0;
            q_1 = q.q1;
            q_2 = q.q2;
            q_3 = q.q3;
            q0_d = qd.q0;
            q1_d = qd.q1;
            q2_d = qd.q2;
            q3_d = qd.q3;
            q0_dd = qdd.q0;
            q1_dd = qdd.q1;
            q2_dd = qdd.q2;
            q3_dd = qdd.q3;
            R_0p_dd = [-4*q2_dd*q_2-4*q2_d^2-4*q3_dd*q_3-4*q3_d^2, 2*q1_dd*q_2+4*q1_d*q2_d+2*q_1*q2_dd-2*q0_dd*q_3-4*q0_d*q3_d-2*q_0*q3_dd, 2*q0_dd*q_2+4*q0_d*q2_d+2*q_0*q2_dd+2*q1_dd*q_3+4*q1_d*q3_d+2*q_1*q3_dd; ...
                2*q1_dd*q2_d+4*q1_d*q2_d+2*q_1*q2_dd+2*q0_dd*q_3+4*q0_d*q3_d+2*q_0*q3_dd, -4*q1_dd*q_1-4*q1_d^2-4*q3_dd*q_3-4*q3_d^2, -2*q0_dd*q_1-4*q0_d*q1_d-2*q_0*q1_dd+2*q2_dd*q_3+4*q2_d*q3_d+2*q_2*q3_dd; ...
                -2*q0_dd*q_2-4*q0_d*q2_d-2*q_0*q2_dd+2*q1_dd*q_3+4*q1_d*q3_d+2*q_1*q3_dd, 2*q0_dd*q_1+4*q0_d*q1_d+2*q_0*q1_dd+2*q2_dd*q_3+4*q2_d*q3_d+2*q_2*q3_dd, -4*q1_dd*q_1-4*q1_d^2-4*q2_dd*q_2-4*q2_d^2];
        end
        
        function [q, q_d, q_dd] = GenerateInterpolation(q_s, q_e, time)
            % Step 1 Quaternion axis and rotation (possibly to optimise in
            % the future)
            R_0s = Quaternion.ToRotationMatrix(q_s);
            R_0e = Quaternion.ToRotationMatrix(q_e);
            R_se = R_0s'*R_0e;
            q_se = Quaternion.FromRotationMatrix(R_se);
            axis_angle_se = AxisAngle.FromQuaternion(q_se);
            % Step 2 Interpolate quaternion angle
            [th_t, th_dot_t, th_ddot_t] = Spline.QuinticInterpolation(0, 0, 0, axis_angle_se.th, 0, 0, time);
            
            for t = 1:length(time)
                th = th_t(t);
                th_dot = th_dot_t(t);
                th_ddot = th_ddot_t(t);
                
                % Step 3 Determine quaternion variables and rotation matrix for
                % trajectory
                a = AxisAngle(th, axis_angle_se.kx, axis_angle_se.ky, axis_angle_se.kz);
                a_d = AxisAngle(th_dot, axis_angle_se.kx, axis_angle_se.ky, axis_angle_se.kz);
                a_dd = AxisAngle(th_ddot, axis_angle_se.kx, axis_angle_se.ky, axis_angle_se.kz);
                
                q(t) = Quaternion.FromAxisAngle(a);
                q_d(t) = Quaternion.DerivativeFromAxisAngle(a, a_d);
                q_dd(t) = Quaternion.DoubleDerivativeFromAxisAngle(a, a_d, a_dd);
            end
        end
    end
    
end

