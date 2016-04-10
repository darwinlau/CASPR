% Lyapunov-based control law with gravity compensation 
%
% Please cite the following paper when using this for multilink cable
% robots:
% A. B. Alp, and S. K. Agrawal, "Cable Suspended Robots: Design, Planning
% and Control", in Proc. IEEE Int. Conf. Robot. Autom., pp. 4275-4280,
% 2002.
%
% Author        : Darwin LAU
% Created       : 2016
% Description    :
%    This version assumes that Kp and Kd are constant values set by the
%    constructor.
classdef LyapunovStaticCompensation < ControllerBase
    
    properties (SetAccess = private)
        id_solver
        Kp
        Kd
    end
    
    methods
        function c = LyapunovStaticCompensation(dyn_model, id_solver, Kp, Kd)
            c@ControllerBase(dyn_model);
            c.id_solver = id_solver;
            c.Kp = Kp;
            c.Kd = Kd;
        end
        
        function [cable_forces] = executeFunction(obj, q, q_d, ~, q_ref, q_ref_d, ~, ~)
            error_comp = obj.Kp * (q_ref - q) + obj.Kd * (q_ref_d - q_d);
            cable_forces = obj.id_solver.resolve(q, zeros(obj.dynModel.numDofs,1), zeros(obj.dynModel.numDofs,1), error_comp);
        end
        
    end
end