classdef (Abstract) SystemKinematicsCables < handle
    %CABLESYSTEMKINEMATICS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        V
    end
    
    properties (SetAccess = protected)
        cables = {};                 % cell array of CableKinematics object
        
        numCables = 0;
        numLinks = 0;
    end
    
    properties (Dependent)
        numSegmentsMax
        lengths                      % vector of forces from cables
    end
    
    methods
        function ck = SystemKinematicsCables(numCables, numLinks)
            ck.numLinks = numLinks;
            ck.numCables = numCables;
            for i = 1:numCables
                ck.cables{i} = CableKinematics(sprintf('Cable %d', i), ck.numLinks);
            end
        end
        
        function update(obj, bodyKinematics)
            assert(bodyKinematics.numLinks == obj.numLinks, 'Number of links between the cable and body kinematics must be consistent');
            % Set each cable's kinematics (absolute attachment locations
            % and segment vectors)
            for i = 1:obj.numCables
                for j = 1:obj.cables{i}.numSegments
                    % cycle through links 0 to p, linkNum = k-1
                    obj.cables{i}.segments{j}.segmentVector = [0;0;0];
                    for k = 1:obj.numLinks+1
                        % First : compute absolute attachment locations
                        if obj.getCRMTerm(i,j,k) ~= 0
                            % k == 1 is base link
                            if k == 1
                                obj.cables{i}.segments{j}.attachmentsAbs{k} = obj.cables{i}.segments{j}.attachmentsLocal{k};
                            else
                                % bodies{k-1} because bodyNum = k - 1;
                                obj.cables{i}.segments{j}.attachmentsAbs{k} = bodyKinematics.bodies{k-1}.r_OG + obj.cables{i}.segments{j}.attachmentsLocal{k};
                            end
                        end
                        % Second : compute cable segment vectors
                        % k == 1 is base link
                        if k == 1
                            obj.cables{i}.segments{j}.segmentVector = obj.cables{i}.segments{j}.segmentVector + obj.getCRMTerm(i,j,k)*obj.cables{i}.segments{j}.attachmentsAbs{k};
                        else
                            obj.cables{i}.segments{j}.segmentVector = obj.cables{i}.segments{j}.segmentVector + obj.getCRMTerm(i,j,k)*(bodyKinematics.bodies{k-1}.R_0k*obj.cables{i}.segments{j}.attachmentsAbs{k});
                        end
                    end
                end
            end
            
            % Determine V
            for i = 1:obj.numCables
                for k = 1:obj.numLinks
                    % linkNum = k - 1
                    V_ixk_T = [0; 0; 0];
                    V_itk_T = [0; 0; 0];
                    for j = 1:obj.cables{i}.numSegments
                        V_ijk_T = obj.getCRMTerm(i,j,k+1)*bodyKinematics.bodies{k}.R_0k'*obj.cables{i}.segments{j}.segmentVector/obj.cables{i}.segments{j}.length;
                        V_ixk_T = V_ixk_T + V_ijk_T;
                        V_itk_T = V_itk_T + cross(obj.cables{i}.segments{j}.attachmentsLocal{k+1}, V_ijk_T);
                    end
                    obj.V(i, 6*k-5:6*k) = [V_ixk_T' V_itk_T'];
                end
            end
        end
        
        % Returns the c_{ijk} element of the CRM
        % CRM is m x s x (p+1) matrix representing the cable-routing
        function c_ijk = getCRMTerm(obj, i, j, k)
            assert(i <= obj.numCables, 'Invalid cable number.');
            c_ijk = obj.cables{i}.getCRMTerm(j, k);
        end
        
        %         function C = getCRM(obj)
        %             C = zeros(obj.numCables, obj.numSegmentsMax, obj.numLinks+1);
        %             for i = 1:obj.numCables
        %                 C(i,:,:)
        %             end
        %             obj.cables{:}.numSegments
        %         end
        
        % NOT SURE HOW THIS IS USED YET, but just a demo of what can be
        % done
        function clearCableSegments(obj)
            % remove all cables
            for i = 1:obj.numCables
                obj.cables{i}.clearSegments(); % = CableKinematics(sprintf('Cable %d', i), obj.numLinks);
            end
        end
        
        function s_max = get.numSegmentsMax(obj)
            s_max = 0;
            for i = 1:obj.numCables
                s_max = max([s_max obj.cables{i}.numSegments]);
            end
        end
        
        function value = get.lengths(obj)
            value = zeros(obj.numCables, 1);
            for i = 1:obj.numCables
                value(i) = obj.cables{i}.length;
            end
        end
    end
end

