classdef JointTrajectory < handle
    %TRAJECTORYINTERPOLATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        q
        q_dot
        q_ddot
        timeVector
        totalTime
        timeStep
    end
    
    methods
        function plotJointSpace(obj, states_to_plot)
            n_dof = length(obj.q{1});
            
            if nargin == 1 || isempty(states_to_plot)
                states_to_plot = 1:n_dof;
            end
            
            q_vector = cell2mat(obj.q);
            qd_vector = cell2mat(obj.q_dot);
            qdd_vector = cell2mat(obj.q_ddot);
            
            figure;
            title('Trajectory');
            plot(obj.timeVector, q_vector(states_to_plot, :), 'Color', 'k', 'LineWidth', 1.5);
            
            figure;
            title('Trajectory velocity');
            plot(obj.timeVector, qd_vector(states_to_plot, :), 'Color', 'k', 'LineWidth', 1.5);
            
            figure;
            title('Trajectory acceleration');
            plot(obj.timeVector, qdd_vector(states_to_plot, :), 'Color', 'k', 'LineWidth', 1.5);
        end
    end
    
    methods (Static)
        function trajectory = LoadXmlObj(xmlObj, kin)
            assert(strcmp(xmlObj.getNodeName, 'trajectory'), 'Element should be <trajectory>');
            total_time = str2double(xmlObj.getElementsByTagName('time_total').item(0).getFirstChild.getData);
            time_step = str2double(xmlObj.getElementsByTagName('time_step').item(0).getFirstChild.getData);
            beginObj = xmlObj.getElementsByTagName('begin').item(0);
            endObj = xmlObj.getElementsByTagName('end').item(0);
            
            q_s = XmlOperations.StringToVector(char(beginObj.getElementsByTagName('q').item(0).getFirstChild.getData));
            q_s_d = XmlOperations.StringToVector(char(beginObj.getElementsByTagName('q_dot').item(0).getFirstChild.getData));
            q_s_dd = XmlOperations.StringToVector(char(beginObj.getElementsByTagName('q_ddot').item(0).getFirstChild.getData));
            q_e = XmlOperations.StringToVector(char(endObj.getElementsByTagName('q').item(0).getFirstChild.getData));
            q_e_d = XmlOperations.StringToVector(char(endObj.getElementsByTagName('q_dot').item(0).getFirstChild.getData));
            q_e_dd = XmlOperations.StringToVector(char(endObj.getElementsByTagName('q_ddot').item(0).getFirstChild.getData));
            
            assert(length(q_s) == kin.numDofs, sprintf('Trajectory config does not contain correct number of DoFs for q begin, desired : %d, specified : %d', kin.numDofs, length(q_s)));
            assert(length(q_s_d) == kin.numDofs, sprintf('Trajectory config does not contain correct number of DoFs for q_dot begin, desired : %d, specified : %d', kin.numDofs, length(q_s_d)));
            assert(length(q_s_dd) == kin.numDofs, sprintf('Trajectory config does not contain correct number of DoFs for q_ddot begin, desired : %d, specified : %d', kin.numDofs, length(q_s_dd)));
            assert(length(q_e) == kin.numDofs, sprintf('Trajectory config does not contain correct number of DoFs for q end, desired : %d, specified : %d', kin.numDofs, length(q_e)));
            assert(length(q_e_d) == kin.numDofs, sprintf('Trajectory config does not contain correct number of DoFs for q_dot end, desired : %d, specified : %d', kin.numDofs, length(q_e_d)));
            assert(length(q_e_dd) == kin.numDofs, sprintf('Trajectory config does not contain correct number of DoFs for q_ddot end, desired : %d, specified : %d', kin.numDofs, length(q_e_dd)));
            
            trajectory = JointTrajectory.GenerateTrajectory(kin, q_s, q_s_d, q_s_dd, q_e, q_e_d, q_e_dd, total_time, time_step);
        end
        
        function trajectory = GenerateTrajectory(kin, q_s, q_s_d, q_s_dd, q_e, q_e_d, q_e_dd, total_time, time_step)
            trajectory = JointTrajectory;
            n_dof = kin.numDofs;    
            t = 0:time_step:total_time;
            trajectory.timeVector = t;
            trajectory.totalTime = total_time;
            trajectory.timeStep = time_step;
                     
            q_array = zeros(n_dof, length(t));
            qd_array = zeros(n_dof, length(t));
            qdd_array = zeros(n_dof, length(t));
            
            index = 1;
            for k = 1:kin.numLinks
                ind_k_s = index;
                ind_k_e = index+kin.bodyKinematics.bodies{k}.joint.numDofs-1;
                qk_s = q_s(ind_k_s:ind_k_e);
                qk_e = q_e(ind_k_s:ind_k_e);
                qk_s_d = q_s_d(ind_k_s:ind_k_e);
                qk_e_d = q_e_d(ind_k_s:ind_k_e);
                qk_s_dd = q_s_dd(ind_k_s:ind_k_e);
                qk_e_dd = q_e_dd(ind_k_s:ind_k_e);
                
                switch kin.bodyKinematics.bodies{k}.joint.type
                    case JointType.R_X
                        [qk, qk_dot, qk_ddot] = JointTrajectory.InterpolateVariable(qk_s, qk_s_d, qk_s_dd, qk_e, qk_e_d, qk_e_dd, t);
                    case JointType.R_Y
                        [qk, qk_dot, qk_ddot] = JointTrajectory.InterpolateVariable(qk_s, qk_s_d, qk_s_dd, qk_e, qk_e_d, qk_e_dd, t);
                    case JointType.R_Z
                        [qk, qk_dot, qk_ddot] = JointTrajectory.InterpolateVariable(qk_s, qk_s_d, qk_s_dd, qk_e, qk_e_d, qk_e_dd, t);
                    case JointType.PLANAR_XY
                        [qk(1, :), qk_dot(1, :), qk_ddot(1, :)] = JointTrajectory.InterpolateVariable(qk_s(1), qk_s_d(1), qk_s_dd(1), qk_e(1), qk_e_d(1), qk_e_dd(1), t);
                        [qk(2, :), qk_dot(2, :), qk_ddot(2, :)] = JointTrajectory.InterpolateVariable(qk_s(2), qk_s_d(2), qk_s_dd(2), qk_e(2), qk_e_d(2), qk_e_dd(2), t);
                        [qk(3, :), qk_dot(3, :), qk_ddot(3, :)] = JointTrajectory.InterpolateVariable(qk_s(3), qk_s_d(3), qk_s_dd(3), qk_e(3), qk_e_d(3), qk_e_dd(3), t);
                    case JointType.S_EULER_XYZ
                        [qk, qk_dot, qk_ddot] = JointTrajectory.InterpolateXYZEuler(qk_s, qk_s_d, qk_s_dd, qk_e, qk_e_d, qk_e_dd, t);
                    case JointType.S_FIXED_XYZ
                        [qk, qk_dot, qk_ddot] = JointTrajectory.InterpolateXYZFixed(qk_s, qk_s_d, qk_s_dd, qk_e, qk_e_d, qk_e_dd, t);
                    otherwise 
                        error('Joint type is not defined');
                end                
                q_array(ind_k_s:ind_k_e, :) = qk;
                qd_array(ind_k_s:ind_k_e, :) = qk_dot;
                qdd_array(ind_k_s:ind_k_e, :) = qk_ddot;
                index = index+kin.bodyKinematics.bodies{k}.joint.numDofs;
            end
            s = size(q_array);
            trajectory.q = mat2cell(q_array, s(1), ones(1, s(2)));
            trajectory.q_dot = mat2cell(qd_array, s(1), ones(1, s(2)));
            trajectory.q_ddot = mat2cell(qdd_array, s(1), ones(1, s(2)));
        end  
        
        function [q, q_dot, q_ddot] = InterpolateXYZFixed(q_s, ~, ~, q_e, ~, ~, time)            
            % Step 1 Quaternion axis and rotation
            R_0s = SphericalFixedXYZ.RelRotationMatrix(q_s);
            R_0e = SphericalFixedXYZ.RelRotationMatrix(q_e);
            R_se = R_0s'*R_0e;
            
            q_se = Quaternion.FromRotationMatrix(R_se);
            axis_angle_se = AxisAngle.FromQuaternion(q_se);
            
            % Step 2 Interpolate quaternion angle
            [th_t, th_dot_t, th_ddot_t] = JointTrajectory.InterpolateVariable(0, 0, 0, axis_angle_se.th, 0, 0, time);
            
            q = zeros(3, length(time));
            q_dot = zeros(3, length(time));
            q_ddot = zeros(3, length(time));
            
            for t = 1:length(time)
                th = th_t(t);
                th_dot = th_dot_t(t);
                th_ddot = th_ddot_t(t);
                
                % Step 3 Determine quaternion variables and rotation matrix for
                % trajectory
                a = AxisAngle(th, axis_angle_se.kx, axis_angle_se.ky, axis_angle_se.kz);
                a_d = AxisAngle(th_dot, axis_angle_se.kx, axis_angle_se.ky, axis_angle_se.kz);
                a_dd = AxisAngle(th_ddot, axis_angle_se.kx, axis_angle_se.ky, axis_angle_se.kz);
                
                q_sp = Quaternion.FromAxisAngle(a);
                q_sp_d = Quaternion.DerivativeFromAxisAngle(a, a_d);
                q_sp_dd = Quaternion.DoubleDerivativeFromAxisAngle(a, a_d, a_dd);
             

                % Step 4
                R_sp = Quaternion.ToRotationMatrix(q_sp);
                R_0p = R_0s*R_sp;
                R_sp_dot = Quaternion.ToRotationMatrixDeriv(q_sp, q_sp_d);
                R_0p_dot = R_0s*R_sp_dot;
                R_sp_ddot = Quaternion.ToRotationMatrixDoubleDeriv(q_sp, q_sp_d, q_sp_dd);
                R_0p_ddot = R_0s*R_sp_ddot;
                
                b = -asin(R_0p(3,1));
                g = atan2(R_0p(2,1), R_0p(1,1));
                a = atan2(R_0p(3,2), R_0p(3,3));  
                
                a = roundn(a, -10);
                b = roundn(b, -10);
                g = roundn(g, -10);
                
                b_d = -R_0p_dot(3,1)/cos(b);                
                g_d = (-b_d*sin(b)*cos(g)-R_0p_dot(1,1))/(cos(b)*sin(g));
                a_d = (-b_d*cos(a)*sin(b)-R_0p_dot(3,3))/(sin(a)*cos(b));
                
                if (~isfinite(a_d))
                    if (t>1)
                        a_d = (a-q(1,t-1))/(time(t)-time(t-1));
                    else
                        a_d = 0;
                    end
                end
                if (~isfinite(b_d))
                    if (t>1)
                        b_d = (b-q(2,t-1))/(time(t)-time(t-1));
                    else
                        b_d = 0;
                    end
                end
                if (~isfinite(g_d))
                    if (t>1)
                        g_d = (g-q(3,t-1))/(time(t)-time(t-1));
                    else
                        g_d = 0;
                    end
                end
                
                a_d = roundn(a_d, -10);
                b_d = roundn(b_d, -10);
                g_d = roundn(g_d, -10);
                
                b_dd = (-R_0p_ddot(3,1)+b_d^2*sin(b))/cos(b);
                g_dd = (-b_dd*sin(b)*cos(g)-b_d*(b_d*cos(b)*cos(g)-g_d*sin(b)*sin(g))-g_d*(-b_d*sin(b)*sin(g)+g_d*cos(b)*cos(g))-R_0p_ddot(1,1))/(cos(b)*sin(g));
                a_dd = (-a_d*(a_d*cos(a)*cos(b)-b_d*sin(a)*sin(b))-b_dd*cos(a)*sin(b)-b_d*(-a_d*sin(a)*sin(b)+b_d*cos(a)*cos(b))-R_0p_ddot(3,3))/(sin(a)*cos(b));
                
                if (~isfinite(a_dd))
                    if (t>1)
                        a_dd = (a_d-q_dot(1,t-1))/(time(t)-time(t-1));
                    else
                        a_dd = 0;
                    end
                end
                if (~isfinite(b_dd))
                    if (t>1)
                        b_dd = (b_d-q_dot(2,t-1))/(time(t)-time(t-1));
                    else
                        b_dd = 0;
                    end
                end
                if (~isfinite(g_dd))
                    if (t>1)
                        g_dd = (g_d-q_dot(3,t-1))/(time(t)-time(t-1));
                    else
                        g_dd = 0;
                    end
                end               
                
                a_dd = roundn(a_dd, -10);
                b_dd = roundn(b_dd, -10);
                g_dd = roundn(g_dd, -10);
                
                q(1,t) = a;
                q(2,t) = b;
                q(3,t) = g;
                q_dot(1,t) = a_d;
                q_dot(2,t) = b_d;
                q_dot(3,t) = g_d;
                q_ddot(1,t) = a_dd;
                q_ddot(2,t) = b_dd;
                q_ddot(3,t) = g_dd;
            end
        end
        
        function [q, q_dot, q_ddot] = InterpolateXYZEuler(q_s, ~, ~, q_e, ~, ~, time)            
            % Step 1 Quaternion axis and rotation
            R_0s = SphericalEulerXYZ.RelRotationMatrix(q_s);
            R_0e = SphericalEulerXYZ.RelRotationMatrix(q_e);
            R_se = R_0s'*R_0e;
            
            q_se = Quaternion.FromRotationMatrix(R_se);
            axis_angle_se = AxisAngle.FromQuaternion(q_se);
            
            % Step 2 Interpolate quaternion angle
            [th_t, th_dot_t, th_ddot_t] = JointTrajectory.InterpolateVariable(0, 0, 0, axis_angle_se.th, 0, 0, time);
            
            q = zeros(3, length(time));
            q_dot = zeros(3, length(time));
            q_ddot = zeros(3, length(time));
            
            for t = 1:length(time)
                th = th_t(t);
                th_dot = th_dot_t(t);
                th_ddot = th_ddot_t(t);
                
                % Step 3 Determine quaternion variables and rotation matrix for
                % trajectory
                a = AxisAngle(th, axis_angle_se.kx, axis_angle_se.ky, axis_angle_se.kz);
                a_d = AxisAngle(th_dot, axis_angle_se.kx, axis_angle_se.ky, axis_angle_se.kz);
                a_dd = AxisAngle(th_ddot, axis_angle_se.kx, axis_angle_se.ky, axis_angle_se.kz);
                
                q_sp = Quaternion.FromAxisAngle(a);
                q_sp_d = Quaternion.DerivativeFromAxisAngle(a, a_d);
                q_sp_dd = Quaternion.DoubleDerivativeFromAxisAngle(a, a_d, a_dd);
             

                % Step 4
                R_sp = Quaternion.ToRotationMatrix(q_sp);
                R_0p = R_0s*R_sp;
                R_sp_dot = Quaternion.ToRotationMatrixDeriv(q_sp, q_sp_d);
                R_0p_dot = R_0s*R_sp_dot;
                R_sp_ddot = Quaternion.ToRotationMatrixDoubleDeriv(q_sp, q_sp_d, q_sp_dd);
                R_0p_ddot = R_0s*R_sp_ddot;
                
                b = asin(R_0p(1,3));
                g = -atan2(R_0p(1,2), R_0p(1,1));
                a = -atan2(R_0p(2,3), R_0p(3,3));
                
                a = roundn(a, -10);
                b = roundn(b, -10);
                g = roundn(g, -10);
                
                b_d = R_0p_dot(1,3)/cos(b);
                g_d = (-b_d*sin(b)*cos(g)-R_0p_dot(1,1))/(cos(b)*sin(g));
                a_d = (-b_d*cos(a)*sin(b)-R_0p_dot(3,3))/(sin(a)*cos(b));
                
                if (~isfinite(a_d))
                    if (t>1)
                        a_d = (a-q(1,t-1))/(time(t)-time(t-1));
                    else
                        a_d = 0;
                    end
                end
                if (~isfinite(b_d))
                    if (t>1)
                        b_d = (b-q(2,t-1))/(time(t)-time(t-1));
                    else
                        b_d = 0;
                    end
                end
                if (~isfinite(g_d))
                    if (t>1)
                        g_d = (g-q(3,t-1))/(time(t)-time(t-1));
                    else
                        g_d = 0;
                    end
                end
                
                a_d = roundn(a_d, -10);
                b_d = roundn(b_d, -10);
                g_d = roundn(g_d, -10);
                
                b_dd = (R_0p_ddot(1,3)+b_d^2*sin(b))/cos(b);
                g_dd = (-b_dd*sin(b)*cos(g)-b_d*(b_d*cos(b)*cos(g)-g_d*sin(b)*sin(g))-g_d*(-b_d*sin(b)*sin(g)+g_d*cos(b)*cos(g))-R_0p_ddot(1,1))/(cos(b)*sin(g));
                a_dd = (-a_d*(a_d*cos(a)*cos(b)-b_d*sin(a)*sin(b))-b_dd*cos(a)*sin(b)-b_d*(-a_d*sin(a)*sin(b)+b_d*cos(a)*cos(b))-R_0p_ddot(3,3))/(sin(a)*cos(b));
                
                if (~isfinite(a_dd))
                    if (t>1)
                        a_dd = (a_d-q_dot(1,t-1))/(time(t)-time(t-1));
                    else
                        a_dd = 0;
                    end
                end
                if (~isfinite(b_dd))
                    if (t>1)
                        b_dd = (b_d-q_dot(2,t-1))/(time(t)-time(t-1));
                    else
                        b_dd = 0;
                    end
                end
                if (~isfinite(g_dd))
                    if (t>1)
                        g_dd = (g_d-q_dot(3,t-1))/(time(t)-time(t-1));
                    else
                        g_dd = 0;
                    end
                end
                
                a_dd = roundn(a_dd, -10);
                b_dd = roundn(b_dd, -10);
                g_dd = roundn(g_dd, -10);
                
                q(1,t) = a;
                q(2,t) = b;
                q(3,t) = g;
                q_dot(1,t) = a_d;
                q_dot(2,t) = b_d;
                q_dot(3,t) = g_d;
                q_ddot(1,t) = a_dd;
                q_ddot(2,t) = b_dd;
                q_ddot(3,t) = g_dd;
            end
        end
        
        function [x, x_dot, x_ddot] = InterpolateVariable(x_s, x_s_dot, x_s_ddot, x_e, x_e_dot, x_e_ddot, t)
            t_s = t(1);
            t_e = t(length(t));
            
            T = [1 t_s t_s^2 t_s^3 t_s^4 t_s^5; ...
                0 1 2*t_s 3*t_s^2 4*t_s^3 5*t_s^4; ...
                0 0 2 6*t_s 12*t_s^2 20*t_s^3; ...
                1 t_e t_e^2 t_e^3 t_e^4 t_e^5; ...
                0 1 2*t_e 3*t_e^2 4*t_e^3 5*t_e^4; ...
                0 0 2 6*t_e 12*t_e^2 20*t_e^3];
            b = [x_s; x_s_dot; x_s_ddot; x_e; x_e_dot; x_e_ddot];
            a = T\b;

            x = a(1) + a(2)*t + a(3)*t.^2 + a(4)*t.^3 + a(5)*t.^4 + a(6)*t.^5;
            x_dot = a(2) + 2*a(3)*t + 3*a(4)*t.^2 + 4*a(5)*t.^3 + 5*a(6)*t.^4;
            x_ddot = 2*a(3) + 6*a(4)*t + 12*a(5)*t.^2 + 20*a(6)*t.^3;
        end
    end
    
end

