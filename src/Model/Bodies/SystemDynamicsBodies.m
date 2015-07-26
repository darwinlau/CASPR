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
    end
    
    properties (Dependent)
        massInertiaMatrix       % Mass-inertia 6p x 6p matrix
        numLinks
    end
    
    methods
        function b = SystemDynamicsBodies(links)
            b.bodies = links;
        end
        
        % bodyKinematics an SystemKinematicsBodies object
        function update(obj, bodyKinematics)
            for k = 1:obj.numLinks
                obj.bodies{k}.update(bodyKinematics);
            end
            
            obj.M_b = obj.massInertiaMatrix*bodyKinematics.W;
            obj.C_b = obj.massInertiaMatrix*bodyKinematics.C_a;
            for k = 1:obj.numLinks
                obj.C_b(6*k-2:6*k) = obj.C_b(6*k-2:6*k) + cross(bodyKinematics.bodies{k}.w, obj.bodies{k}.I_G*bodyKinematics.bodies{k}.w);
            end
            obj.G_b = MatrixOperations.Initialise(6*obj.numLinks,1,isa(bodyKinematics.q,'sym'));
            for k = 1:obj.numLinks
                obj.G_b(6*k-5:6*k-3) = bodyKinematics.bodies{k}.R_0k.'*[0; 0; -obj.bodies{k}.m*SystemKinematicsDynamics.GRAVITY_CONSTANT];
            end
            
            obj.M =   bodyKinematics.W.' * obj.M_b;
            obj.C =   bodyKinematics.W.' * obj.C_b;
            obj.G = - bodyKinematics.W.' * obj.G_b;
        end
        
        function M = get.massInertiaMatrix(obj)
            M = zeros(6*obj.numLinks, 6*obj.numLinks);
            for k = 1:obj.numLinks
                M(6*k-5:6*k, 6*k-5:6*k) = [obj.bodies{k}.m*eye(3) zeros(3,3); zeros(3,3) obj.bodies{k}.I_G];
            end
        end
        
        function N = calculate_N(obj,bodyKinematics)
            %% Initialisiation
            q = bodyKinematics.q;
            W = bodyKinematics.W;
            S = bodyKinematics.S;
            P = bodyKinematics.P;
            flag = isa(q,'sym');
            MassInertiaMatrix = obj.massInertiaMatrix;
            %% PS_dot term
            n_q = length(q); n_l = bodyKinematics.numLinks;
            N_j = MatrixOperations.Initialise(n_q,n_q^2,flag);
            A = MatrixOperations.Initialise(6*n_l,n_q,flag);
            offset_x = 1; offset_y = 1;
            for i=1:n_l
                [N_jt,A_t] = bodyKinematics.bodies{i}.joint.QuadMatrix(q);
                n_qt = size(N_jt,1);
                N_j(offset_x:offset_x+n_qt-1,(i-1)*n_q+offset_y:(i-1)*n_q+offset_y+n_qt^2-1) = N_jt;
                A((i-1)*6+1:(i-1)*6+6,offset_x:offset_x+n_qt-1) = A_t;
                offset_x = offset_x + n_qt; offset_y = offset_y + n_qt^2;
            end
            % Project correctly
            C1 = MatrixOperations.MatrixProdLeftQuad(W.'*MassInertiaMatrix*P*A,N_j);
            
            %% P_dotS term
            C2 = MatrixOperations.Initialise(n_q,n_q^2,flag);
            for i=1:n_l
                ip = bodyKinematics.bodies{i}.parentLinkId;
                if (ip > 0)
                    W_ip = W(6*(ip-1)+4:6*(ip-1)+6,:);
                else
                    W_ip = zeros(3,n_q);
                end
                W_i = W(6*(i-1)+4:6*(i-1)+6,:);
                N_c1 = [zeros(n_q,1),-2*W_ip(3,:).',2*W_ip(2,:).',2*W_ip(3,:).',zeros(n_q,1),-2*W_ip(1,:).',-2*W_ip(2,:).',2*W_ip(1,:).',zeros(n_q,1)];
                N_c2 = [zeros(n_q,1),-W_i(3,:).',W_i(2,:).',W_i(3,:).',zeros(n_q,1),-W_i(1,:).',-W_i(2,:).',W_i(1,:).',zeros(n_q,1)];
                N_cr1 = MatrixOperations.MatrixProdRightQuad(N_c1,S(6*(i-1)+1:6*(i-1)+3,:));
                T_1 = zeros(6*n_l,3);
                T_1(6*(i-1)+1:6*(i-1)+3,:) = eye(3);
                N_cr2 = MatrixOperations.MatrixProdRightQuad(N_c2,S(6*(i-1)+4:6*(i-1)+6,:));
                T_2 = zeros(6*n_l,3);
                T_2(6*(i-1)+4:6*(i-1)+6,:) = eye(3);
                C2 = MatrixOperations.MatrixProdLeftQuad(W.'*MassInertiaMatrix*P*T_1,N_cr1) + MatrixOperations.MatrixProdLeftQuad(W.'*MassInertiaMatrix*P*T_2,N_cr2);
            end
            
            %% Cross product time
            C3 = MatrixOperations.Initialise(n_q,n_q^2,flag);
            for i = 1:n_l
                N_xf = MatrixOperations.Initialise(n_q,3*n_q,flag);
                for j = 1:i
                    jp = bodyKinematics.bodies{j}.parentLinkId;
                    if (jp > 0 && bodyKinematics.bodiesPathGraph(j,i))
                        W_ip = W(6*(jp-1)+4:6*(jp-1)+6,:);
                        N_x = [zeros(n_q,1),-W_ip(3,:).',W_ip(2,:).',W_ip(3,:).',zeros(n_q,1),-W_ip(1,:).',-W_ip(2,:).',W_ip(1,:).',zeros(n_q,1)];
                        R = MatrixOperations.SkewSymmetric(bodyKinematics.bodies{j}.r_Parent + bodyKinematics.bodies{j}.joint.r_rel)*W_ip;
                        N_xr = -MatrixOperations.MatrixProdRightQuad(N_x,R);
                        N_xf = N_xf + MatrixOperations.MatrixProdLeftQuad(bodyKinematics.bodies{i}.R_0k.'*bodyKinematics.bodies{jp}.R_0k,N_xr);
                    end
                end
                W_i = W(6*(i-1)+4:6*(i-1)+6,:);
                N_x = [zeros(n_q,1),-W_i(3,:).',W_i(2,:).',W_i(3,:).',zeros(n_q,1),-W_i(1,:).',-W_i(2,:).',W_i(1,:).',zeros(n_q,1)];
                R = MatrixOperations.SkewSymmetric(bodyKinematics.bodies{1}.r_G)*W_i;
                N_xf = N_xf - MatrixOperations.MatrixProdRightQuad(N_x,R);
                T = zeros(6*n_l,3);
                T(6*(i-1)+1:6*(i-1)+3,:) = eye(3);
                C3 = C3 + MatrixOperations.MatrixProdLeftQuad(W.'*MassInertiaMatrix*T,N_xf);
            end
            
            %% Final body related terms
            C4 = MatrixOperations.Initialise(n_q,n_q^2,flag);
            for i = 1:n_l
                W_i = W(6*(i-1)+4:6*(i-1)+6,:);
                N_x = [zeros(n_q,1),-W_i(3,:).',W_i(2,:).',W_i(3,:).',zeros(n_q,1),-W_i(1,:).',-W_i(2,:).',W_i(1,:).',zeros(n_q,1)];
                N_xf = MatrixOperations.MatrixProdRightQuad(N_x,obj.bodies{i}.I_G*W_i);
                T = zeros(6*n_l,3);
                T(6*(i-1)+4:6*(i-1)+6,:) = eye(3);
                C4 = C4 + MatrixOperations.MatrixProdLeftQuad(W.'*T,N_xf);
            end
            
            %% Compute N
            N = C1 + C2 + C3 + C4;
            if(flag)
                N = simplify(N);
                V = MatrixOperations.Initialise(n_q,1,flag);
                for i=1:n_q
                    V(i) = bodyKinematics.q_dot.'*N(:,(i-1)*n_q+1:(i-1)*n_q+n_q)*bodyKinematics.q_dot;
                end
                simplify(V)
            end
        end
        
        function n = get.numLinks(obj)
            n = length(obj.bodies);
        end
    end
    
    
    methods (Static)
        function b = LoadXmlObj(body_prop_xmlobj)
            assert(strcmp(body_prop_xmlobj.getNodeName, 'links'), 'Root elemnt should be <links>');
            allLinkItems = body_prop_xmlobj.getChildNodes;
            
            num_links = allLinkItems.getLength;
            links = cell(1,num_links);
            
            % Creates all of the links first
            for k = 1:num_links
                % Java uses 0 indexing
                currentLinkItem = allLinkItems.item(k-1);
                
                num_k = str2double(char(currentLinkItem.getAttribute('num')));
                assert(num_k == k, sprintf('Link number does not correspond to its order, order: %d, specified num: %d ', k, num_k));
                
                type = char(currentLinkItem.getNodeName);
                if (strcmp(type, 'link_rigid'))
                    links{k} = BodyDynamicsRigid.LoadXmlObj(currentLinkItem);
                else
                    error('Unknown link type: %s', type);
                end
            end
            
            % Create the actual object to return
            b = SystemDynamicsBodies(links);
        end
    end
end