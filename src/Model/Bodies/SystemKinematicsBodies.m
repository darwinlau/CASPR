classdef SystemKinematicsBodies < handle
    %BODYSYSTEMKINEMATICS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected)        
        bodies                  % Cell array of BodyKinematics objects
        % B Matrix
        connectivityGraph       % p x p connectivity matrix, if (i,j) = 1 means link i-1 is the parent of link j
        % D Matrix
        bodiesPathGraph         % p x p matrix that governs how to track to particular bodies, (i,j) = 1 means that to get to link j we must pass through link i 
        
        S                       % S matrix representing relationship between relative body velocities (joint) and generalised coordinates velocities
        S_dot                   % Derivative of S
        P                       % 6p x 6p matrix representing mapping between absolute body velocities (CoG) and relative body velocities (joint)
        W                       % W = P*S : 6p x n matrix representing mapping \dot{\mathbf{x}} = W \dot{\mathbf{q}}
        
        q
        q_dot
        q_ddot
        
        x_dot                   % Absolute velocities
        x_ddot                  % Absolute accelerations (x_ddot = W(q)*q_ddot + C_a(q,q_dot))
        
        C_a                     % Relationship between body and joint accelerations \ddot{\mathbf{x}} = W \ddot{\mathbf{q}} + C_a
        
        numDofs
    end
    
    properties (Dependent)
        numLinks
    end
    
    methods
        function b = SystemKinematicsBodies(bodies, num_dofs)
            b.bodies = bodies;
            b.numDofs = num_dofs;
            
            b.connectivityGraph = zeros(b.numLinks, b.numLinks);
            b.bodiesPathGraph = zeros(b.numLinks, b.numLinks);
            b.S = zeros(6*b.numLinks, b.numDofs);
            b.P = zeros(6*b.numLinks, 6*b.numLinks);
            b.W = zeros(6*b.numLinks, b.numDofs);
        end
        
        function update(obj, q, q_dot, q_ddot)
            % Assign q, q_dot, q_ddot
            obj.q = q;
            obj.q_dot = q_dot;
            obj.q_ddot = q_ddot;
            
            % Update each body first
            index = 1;
            for k = 1:obj.numLinks
                q_k = q(index:index+obj.bodies{k}.joint.numDofs-1);
                q_dot_k = q_dot(index:index+obj.bodies{k}.joint.numDofs-1);
                q_ddot_k = q_ddot(index:index+obj.bodies{k}.joint.numDofs-1);
                obj.bodies{k}.update(q_k, q_dot_k, q_ddot_k);
                %obj.bodies{k}.joint.update(q_k, q_dot_k, q_ddot_k);
                index = index + obj.bodies{k}.joint.numDofs;
            end
            
            % Now the global system updates
            % Set bodies kinematics (rotation matrices)
            for k = 1:obj.numLinks
                parent_link_num = obj.bodies{k}.parentLinkId;
                assert(parent_link_num < k, 'Problem with numbering of links with parent and child');
                
                % Determine rotation matrix
                % Determine joint location
                if parent_link_num > 0
                    obj.bodies{k}.R_0k = obj.bodies{parent_link_num}.R_0k*obj.bodies{k}.joint.R_pe;
                    obj.bodies{k}.r_OP = obj.bodies{k}.joint.R_pe.'*(obj.bodies{parent_link_num}.r_OP + obj.bodies{k}.r_Parent + obj.bodies{k}.joint.r_rel);
                else
                    obj.bodies{k}.R_0k = obj.bodies{k}.joint.R_pe;
                    obj.bodies{k}.r_OP = obj.bodies{k}.joint.R_pe.'*(obj.bodies{k}.r_Parent + obj.bodies{k}.joint.r_rel);
                end
                % Determine absolute position of COG
                obj.bodies{k}.r_OG = obj.bodies{k}.r_OP + obj.bodies{k}.r_G;
                % Determine absolute position of link's ending position
                obj.bodies{k}.r_OPe = obj.bodies{k}.r_OP + obj.bodies{k}.r_P;
            end
            
            % Set S (joint state matrix) and S_dot
            index = 1;
            for k = 1:obj.numLinks
                obj.S(6*k-5:6*k, index:index+obj.bodies{k}.joint.numDofs-1) = obj.bodies{k}.joint.S;  
                obj.S_dot(6*k-5:6*k, index:index+obj.bodies{k}.joint.numDofs-1) = obj.bodies{k}.joint.S_dot;  
                index = index + obj.bodies{k}.joint.numDofs;
            end
            
            % Set P (relationship with joint propagation)
            for k = 1:obj.numLinks
                for a = 1:k
                    R_ka = obj.bodies{k}.R_0k.'*obj.bodies{a}.R_0k;   
                    Pak = obj.bodiesPathGraph(a,k)*[R_ka*obj.bodies{a}.joint.R_pe.' -R_ka*MatrixOperations.SkewSymmetric(-obj.bodies{a}.r_OP + R_ka.'*obj.bodies{k}.r_OG); ...
                        zeros(3,3) R_ka]; 
                    obj.P(6*k-5:6*k, 6*a-5:6*a) = Pak;
                end
            end
            
            % W = P*S
            obj.W = obj.P*obj.S;
            
            % Determine x_dot
            obj.x_dot = obj.W*obj.q_dot;
            % Extract absolute velocities
            for k = 1:obj.numLinks
                obj.bodies{k}.v_OG = obj.x_dot(6*k-5:6*k-3); 
                obj.bodies{k}.w = obj.x_dot(6*k-2:6*k);
            end
            
            % Determine x_ddot            
            ang_mat = zeros(6*obj.numLinks, 6*obj.numLinks);
            for k = 1:obj.numLinks
                kp = obj.bodies{k}.parentLinkId;
                if (kp > 0)
                    w_kp = obj.bodies{kp}.w;
                else 
                    w_kp = zeros(3,1);
                end
                w_k = obj.bodies{k}.w;
                ang_mat(6*k-5:6*k, 6*k-5:6*k) = [2*MatrixOperations.SkewSymmetric(w_kp) zeros(3,3); zeros(3,3) MatrixOperations.SkewSymmetric(w_k)];
            end
            
            obj.C_a = obj.P*obj.S_dot*obj.q_dot + obj.P*ang_mat*obj.S*obj.q_dot;
            for k = 1:obj.numLinks
                for a = 1:k
                    ap = obj.bodies{a}.parentLinkId;
                    if (ap > 0 && obj.bodiesPathGraph(a,k))
                        obj.C_a(6*k-5:6*k-3) = obj.C_a(6*k-5:6*k-3) + obj.bodies{k}.R_0k.'*obj.bodies{ap}.R_0k*cross(obj.bodies{ap}.w, cross(obj.bodies{ap}.w, obj.bodies{a}.r_Parent + obj.bodies{a}.joint.r_rel));
                    end
                end
                obj.C_a(6*k-5:6*k-3) = obj.C_a(6*k-5:6*k-3) + cross(obj.bodies{k}.w, cross(obj.bodies{k}.w, obj.bodies{k}.r_G));
            end
            
            obj.x_ddot = obj.P*obj.S*obj.q_ddot + obj.C_a;
            % Extract absolute accelerations
            for k = 1:obj.numLinks
                obj.bodies{k}.a_OG = obj.x_ddot(6*k-5:6*k-3); 
                obj.bodies{k}.w_dot = obj.x_ddot(6*k-2:6*k);
            end
        end
        
        function sim_update(obj, q, q_dot, q_ddot)
            % Assign q, q_dot, q_ddot
            obj.q = q;
            obj.q_dot = q_dot;
            obj.q_ddot = q_ddot;
            
            % Update each body first
            index = 1;
            for k = 1:obj.numLinks
                q_k = q(index:index+obj.bodies{k}.joint.numDofs-1);
                q_dot_k = q_dot(index:index+obj.bodies{k}.joint.numDofs-1);
                q_ddot_k = q_ddot(index:index+obj.bodies{k}.joint.numDofs-1);
                obj.bodies{k}.update(q_k, q_dot_k, q_ddot_k);
                %obj.bodies{k}.joint.update(q_k, q_dot_k, q_ddot_k);
                index = index + obj.bodies{k}.joint.numDofs;
            end
            
            % Now the global system updates
            % Set bodies kinematics (rotation matrices)
            for k = 1:obj.numLinks
                parent_link_num = obj.bodies{k}.parentLinkId;
                assert(parent_link_num < k, 'Problem with numbering of links with parent and child');
                
                % Determine rotation matrix
                % Determine joint location
                if parent_link_num > 0
                    obj.bodies{k}.R_0k = obj.bodies{parent_link_num}.R_0k*obj.bodies{k}.joint.R_pe;
                    obj.bodies{k}.r_OP = obj.bodies{k}.joint.R_pe.'*(obj.bodies{parent_link_num}.r_OP + obj.bodies{k}.r_Parent + obj.bodies{k}.joint.r_rel);
                else
                    obj.bodies{k}.R_0k = obj.bodies{k}.joint.R_pe;
                    obj.bodies{k}.r_OP = obj.bodies{k}.joint.R_pe.'*(obj.bodies{k}.r_Parent + obj.bodies{k}.joint.r_rel);
                end
                % Determine absolute position of COG
                obj.bodies{k}.r_OG = obj.bodies{k}.r_OP + obj.bodies{k}.r_G;
                % Determine absolute position of link's ending position
                obj.bodies{k}.r_OPe = obj.bodies{k}.r_OP + obj.bodies{k}.r_P;
            end
            
            % Set S (joint state matrix) and S_dot
            index = 1; obj.S = sym(zeros(size(obj.S))); obj.S_dot = sym(zeros(size(obj.S_dot)));
            for k = 1:obj.numLinks
                obj.S(6*k-5:6*k, index:index+obj.bodies{k}.joint.numDofs-1) = obj.bodies{k}.joint.S;  
                obj.S_dot(6*k-5:6*k, index:index+obj.bodies{k}.joint.numDofs-1) = obj.bodies{k}.joint.S_dot;  
                index = index + obj.bodies{k}.joint.numDofs;
            end
                        
            % Set P (relationship with joint propagation)
            obj.P = sym(zeros(size(obj.P)));
            for k = 1:obj.numLinks
                for a = 1:k
                    R_ka = obj.bodies{k}.R_0k.'*obj.bodies{a}.R_0k;   
                    Pak = obj.bodiesPathGraph(a,k)*[R_ka*obj.bodies{a}.joint.R_pe.' -R_ka*MatrixOperations.SkewSymmetric(-obj.bodies{a}.r_OP + R_ka.'*obj.bodies{k}.r_OG); ...
                        zeros(3,3) R_ka]; 
                    obj.P(6*k-5:6*k, 6*a-5:6*a) = Pak;
                end
            end
            obj.P = simplify(obj.P);
            
            % W = P*S
            obj.W = obj.P*obj.S;
            obj.W = simplify(obj.W);
            
            % Determine x_dot
            obj.x_dot = obj.W*obj.q_dot;
            % Extract absolute velocities
            for k = 1:obj.numLinks
                obj.bodies{k}.v_OG = obj.x_dot(6*k-5:6*k-3); 
                obj.bodies{k}.w = obj.x_dot(6*k-2:6*k);
            end
            
            % Determine x_ddot            
            ang_mat = sym(zeros(6*obj.numLinks, 6*obj.numLinks));
            for k = 1:obj.numLinks
                kp = obj.bodies{k}.parentLinkId;
                if (kp > 0)
                    w_kp = obj.bodies{kp}.w;
                else 
                    w_kp = zeros(3,1);
                end
                w_k = obj.bodies{k}.w;
                ang_mat(6*k-5:6*k, 6*k-5:6*k) = [2*MatrixOperations.SkewSymmetric(w_kp) zeros(3,3); zeros(3,3) MatrixOperations.SkewSymmetric(w_k)];
            end
            
            obj.C_a = obj.P*obj.S_dot*obj.q_dot + obj.P*ang_mat*obj.S*obj.q_dot;
            for k = 1:obj.numLinks
                for a = 1:k
                    ap = obj.bodies{a}.parentLinkId;
                    if (ap > 0 && obj.bodiesPathGraph(a,k))
                        obj.C_a(6*k-5:6*k-3) = obj.C_a(6*k-5:6*k-3) + obj.bodies{k}.R_0k.'*obj.bodies{ap}.R_0k*cross(obj.bodies{ap}.w, cross(obj.bodies{ap}.w, obj.bodies{a}.r_Parent + obj.bodies{a}.joint.r_rel));
                    end
                end
                obj.C_a(6*k-5:6*k-3) = obj.C_a(6*k-5:6*k-3) + cross(obj.bodies{k}.w, cross(obj.bodies{k}.w, obj.bodies{k}.r_G));
            end
            obj.C_a = simplify(obj.C_a);
            
            obj.x_ddot = obj.P*obj.S*obj.q_ddot + obj.C_a;
            % Extract absolute accelerations
            for k = 1:obj.numLinks
                obj.bodies{k}.a_OG = obj.x_ddot(6*k-5:6*k-3); 
                obj.bodies{k}.w_dot = obj.x_ddot(6*k-2:6*k);
            end
        end
        
        function formConnectiveMap(obj)
            for k = 1:obj.numLinks
                obj.connectBodies(obj.bodies{k}.parentLinkId, k, obj.bodies{k}.r_Parent);
            end
        end
        
        function connectBodies(obj, parent_link_num, child_link_num, r_parent_loc)   
            assert(parent_link_num < child_link_num, 'Parent link number must be smaller than child');
            assert(~isempty(obj.bodies{child_link_num}), 'Child link does not exist');
            if parent_link_num > 0
                assert(~isempty(obj.bodies{parent_link_num}), 'Parent link does not exist');
            end
            
            obj.bodiesPathGraph(child_link_num, child_link_num) = 1;
            child_link = obj.bodies{child_link_num};
            if parent_link_num == 0
                parent_link = [];
            else
                parent_link = obj.bodies{parent_link_num};
            end
            child_link.addParent(parent_link, r_parent_loc);
            obj.connectivityGraph(parent_link_num+1, child_link_num) = 1;
            
            if (parent_link_num > 0)
                obj.bodiesPathGraph(parent_link_num, child_link_num) = 1;
                obj.bodiesPathGraph(:, child_link_num) = obj.bodiesPathGraph(:, child_link_num) | obj.bodiesPathGraph(:, parent_link_num);
            end
        end
        
        function n = get.numLinks(obj)
            n = length(obj.bodies);
        end
    end
    
    methods (Static)
        function b = LoadXmlObj(body_prop_xmlobj)
            assert(strcmp(body_prop_xmlobj.getNodeName, 'links'), 'Root element should be <links>');
            allLinkItems = body_prop_xmlobj.getChildNodes;
                        
            num_links = allLinkItems.getLength;
            num_dofs = 0;
            links = cell(1,num_links);
            
            % Creates all of the links first
            for k = 1:num_links
                % Java uses 0 indexing
                currentLinkItem = allLinkItems.item(k-1);
                
                num_k = str2double(currentLinkItem.getAttribute('num'));
                assert(num_k == k, sprintf('Link number does not correspond to its order, order: %d, specified num: %d ', k, num_k));
                     
                type = char(currentLinkItem.getNodeName);
                if (strcmp(type, 'link_rigid'))
                    %currentLinkItem.getChildNodes.getElementsByTagName('parent').item(0).getChildNodes.getLength
                    links{k} = BodyKinematicsRigid.LoadXmlObj(currentLinkItem);
                else
                    error('Unknown link type: %s', type);
                end
                num_dofs = num_dofs + links{k}.numDofs;
            end
            
            % Create the actual object to return
            b = SystemKinematicsBodies(links, num_dofs);
            b.formConnectiveMap();
        end
    end
end

