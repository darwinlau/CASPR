classdef Quaternion 
    %QUATERNION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        q0
        q1
        q2
        q3
    end
    
    methods (Static)
        function q_0p = FromRotationMatrix(R_0p)
            q_0p = Quaternion;
            
            q_0p.q0 = 1/2*sqrt(1 + R_0p(1,1) + R_0p(2,2) + R_0p(3,3));
            q_0p.q1 = (R_0p(2,3) - R_0p(3,2))/(4*q_0p.q0);
            q_0p.q2 = (R_0p(3,1) - R_0p(1,3))/(4*q_0p.q0);
            q_0p.q3 = (R_0p(1,2) - R_0p(2,1))/(4*q_0p.q0);
        end
        
        function q = FromAxisAngle(a)
            q = Quaternion;
            q.q0 = cos(a.th/2);
            q.q1 = a.kx*sin(a.th/2);
            q.q2 = a.ky*sin(a.th/2);
            q.q3 = a.kz*sin(a.th/2);
        end
        
        function q = DerivativeFromAxisAngle(a, a_d)                
            q = Quaternion;
            q.q0 = -a_d.th/2*sin(a.th/2);
            q.q1 = a.kx*a_d.th/2*cos(a.th/2);
            q.q2 = a.ky*a_d.th/2*cos(a.th/2);
            q.q3 = a.kz*a_d.th/2*cos(a.th/2);
        end
        
        function q = DoubleDerivativeFromAxisAngle(a, a_d, a_dd)
            q = Quaternion;
                
            q.q0 = -a_dd.th/2*sin(a.th/2) - a_d.th^2/4*cos(a.th/2);
            q.q1 = a.kx*(a_dd.th/2*cos(a.th/2) - a_d.th^2/4*sin(a.th/2));
            q.q2 = a.ky*(a_dd.th/2*cos(a.th/2) - a_d.th^2/4*sin(a.th/2));
            q.q3 = a.kz*(a_dd.th/2*cos(a.th/2) - a_d.th^2/4*sin(a.th/2));
        end
        
        function R_0p = ToRotationMatrix(q)
            q_0 = q.q0;
            q_1 = q.q1;
            q_2 = q.q2;
            q_3 = q.q3;
            R_0p = [1-2*(q_2^2+q_3^2), 2*q_1*q_2+2*q_0*q_3, -2*q_0*q_2+2*q_1*q_3; ...
                2*q_1*q_2-2*q_0*q_3, 1-2*(q_1^2+q_3^2), 2*q_0*q_1+2*q_2*q_3; ...
                2*q_0*q_2+2*q_1*q_3, -2*q_0*q_1+2*q_2*q_3, 1-2*(q_1^2+q_2^2)];
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
            R_0p_d = [-4*q2_d*q_2-4*q3_d*q_3, 2*q1_d*q_2+2*q_1*q2_d+2*q0_d*q_3+2*q_0*q3_d, -2*q0_d*q_2-2*q_0*q2_d+2*q1_d*q_3+2*q_1*q3_d; ...
                2*q1_d*q_2+2*q_1*q2_d-2*q0_d*q_3-2*q_0*q3_d, -4*q1_d*q_1-4*q3_d*q_3, 2*q0_d*q_1+2*q_0*q1_d+2*q2_d*q_3+2*q_2*q3_d; ...
                2*q0_d*q_2+2*q_0*q2_d+2*q1_d*q_3+2*q_1*q3_d, -2*q0_d*q_1-2*q_0*q1_d+2*q2_d*q_3+2*q_2*q3_d, -4*q1_d*q_1-4*q2_d*q_2];
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
            R_0p_dd = [-4*q2_dd*q_2-4*q2_d^2-4*q3_dd*q_3-4*q3_d^2, 2*q1_dd*q2_d+4*q1_d*q2_d+2*q_1*q2_dd+2*q0_dd*q_3+4*q0_d*q3_d+2*q_0*q3_dd, -2*q0_dd*q_2-4*q0_d*q2_d-2*q_0*q2_dd+2*q1_dd*q_3+4*q1_d*q3_d+2*q_1*q3_dd; ...
                2*q1_dd*q_2+4*q1_d*q2_d+2*q_1*q2_dd-2*q0_dd*q_3-4*q0_d*q3_d-2*q_0*q3_dd, -4*q1_dd*q_1-4*q1_d^2-4*q3_dd*q_3-4*q3_d^2, 2*q0_dd*q_1+4*q0_d*q1_d+2*q_0*q1_dd+2*q2_dd*q_3+4*q2_d*q3_d+2*q_2*q3_dd; ...
                2*q0_dd*q_2+4*q0_d*q2_d+2*q_0*q2_dd+2*q1_dd*q_3+4*q1_d*q3_d+2*q_1*q3_dd, -2*q0_dd*q_1-4*q0_d*q1_d-2*q_0*q1_dd+2*q2_dd*q_3+4*q2_d*q3_d+2*q_2*q3_dd, -4*q1_dd*q_1-4*q1_d^2-4*q2_dd*q_2-4*q2_d^2];
        end
    end
    
end

