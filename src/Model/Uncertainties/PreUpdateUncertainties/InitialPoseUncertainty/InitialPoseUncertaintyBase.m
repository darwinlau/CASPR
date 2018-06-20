% Base class for uncertainties associated with initial pose error
%
% Author        : Jonathan EDEN
% Created       : 2017
% Description    :
%    Base class for uncertainties on the initial pose
classdef (Abstract) InitialPoseUncertaintyBase < PreUpdateUncertaintyBase
    properties
        fk_solver           % A forward kinematics solver
        model               % Model object
        q_prev              % The previous q value
        q_d_prev            % The previous q derivative value
        l_prev              % The previous l  
        cable_indices       % The cable indices (SET TO ALWAYS ALL CABLES FOR THE MOMENT)
    end
    
    methods
        
        function ipu = InitialPoseUncertaintyBase(fk_solver, model, q_initial, q_d_initial) 
            ipu.model           =   model;
            ipu.fk_solver       =   fk_solver;
            ipu.q_prev          =   q_initial;
            ipu.q_d_prev        =   q_d_initial;
            ipu.model.update(ipu.q_prev,ipu.q_d_prev,zeros(model.numDofs,1),zeros(model.numDofs,1));
            ipu.l_prev          =   ipu.model.cableLengths;
            ipu.cable_indices   =   1:ipu.model.numCables;
        end
        
%         % Implementation of applyPreUpdateUncertainty
%         function [update_q,update_q_dot,update_q_ddot] = applyPreUpdateUncertainty(obj,q,q_dot,q_ddot,dt)
%             obj.model.update(q,q_dot,q_ddot,zeros(obj.model.numDofs));
%             l = obj.model.cableLengths;
%             [update_q, update_q_dot] = obj.fk_solver.compute(l,obj.l_prev,obj.cable_indices,obj.q_prev,obj.q_d_prev,dt);
%             obj.q_prev   =   q;
%             obj.q_d_prev =   q_dot;
%             update_q_ddot = q_ddot; 
%         end
    end
    methods(Abstract)
        % Apply initial length uncertainty before the update
        [update_q,update_q_dot] = applyInitialOffset(obj,q,q_dot);
    end
end

