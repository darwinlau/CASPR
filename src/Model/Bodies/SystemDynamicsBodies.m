classdef SystemDynamicsBodies < handle
    %BODYSYSTEMKINEMATICS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)          
        bodies                  % Cell array of BodyDynamics objects        
        
        % M_b*q_ddot + C_b = G_b + forces in body space (external forces, interaction forces, cable forces (-L_b^T*f)) 
        M_b                         % Body mass-inertia matrix 
        C_b                         % Body C matrix
        G_b                         % Body G matrix
        
        % M*q_ddot + C + G = external forces in joint space (external forces, cable forces (-L^T*f))
        M
        C
        G
        
        numLinks
        numDofs
    end
    
    properties (Dependent)
        massInertiaMatrix       % Mass-inertia 6p x 6p matrix
    end
    
    methods
        function b = SystemDynamicsBodies(num_links, num_dofs)
            b.numLinks = num_links;
            b.numDofs = num_dofs;
        end
        
        % bodyKinematics an SystemKinematicsBodies object
        function update(obj, bodyKinematics)
            obj.M_b = obj.massInertiaMatrix*bodyKinematics.W;
            obj.C_b = obj.massInertiaMatrix*bodyKinematics.C_a;
            for k = 1:obj.numLinks
                obj.C_b(6*k-2:6*k) = obj.C_b(6*k-2:6*k) + cross(bodyKinematics.bodies{k}.w, obj.bodies{k}.I_G*bodyKinematics.bodies{k}.w);
            end
            obj.G_b = zeros(6*obj.numLinks, 1);
            for k = 1:obj.numLinks
                obj.G_b(6*k-5:6*k-3) = bodyKinematics.bodies{k}.R_0k'*[0; 0; -obj.bodies{k}.m*SystemDynamics.GRAVITY_CONSTANT];
            end        
        
            obj.M =   bodyKinematics.W' * obj.M_b;
            obj.C =   bodyKinematics.W' * obj.C_b;
            obj.G = - bodyKinematics.W' * obj.G_b;
        end
        
        function M = get.massInertiaMatrix(obj)
            M = zeros(6*obj.numLinks, 6*obj.numLinks);
            for k = 1:obj.numLinks
                M(6*k-5:6*k, 6*k-5:6*k) = [obj.bodies{k}.m*eye(3) zeros(3,3); zeros(3,3) obj.bodies{k}.I_G];
            end
        end
    end
    
end

