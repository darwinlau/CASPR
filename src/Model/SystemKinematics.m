classdef SystemKinematics < handle
    %SystemKinematics Contains the information for the kinematic state for
    %the entire cable-driven system
    
    properties (SetAccess = protected)    
        bodyKinematics              % SystemKinematicsBodies object
        cableKinematics             % SystemKinematicsCables object
    end
    
    properties (Dependent)
        numLinks                    % Number of links
        numDofs                     % Number of degrees of freedom
        numCables                   % Number of cables
        cableLengths                % Vector of cable lengths
        cableLengthsDot             % Vector of cable length derivatives
        
        L                           % cable to joint Jacobian matrix L = VW
        V                           % Cable V matrix
        W                           % Body W matrix, W = PS
        S                           % Body S matrix
        S_dot                       % Body S_dot matrix
        P                           % Body P matrix
        
        q                           % Generalised coordinates state vector
        q_dot                       % Generalised coordinates derivative state vector
        q_ddot                      % Generalised coordinates double derivative state vector
    end
    
    methods (Static)
        function k = LoadXmlObj(body_xmlobj, cable_xmlobj)
            k = SystemKinematics();
            k.bodyKinematics = SystemKinematicsBodies.LoadXmlObj(body_xmlobj);
            k.cableKinematics = SystemKinematicsCables.LoadXmlObj(cable_xmlobj, k.numLinks);
            k.update(zeros(k.numDofs,1), zeros(k.numDofs,1), zeros(k.numDofs,1));
        end
    end
    
    methods
        function k = SystemKinematics()            
%             (bkConstructor, ckConstructor)
%             % Constructor from the bodies and cables properties
%             b.bodyKinematics = bkConstructor();
%             b.cableKinematics = ckConstructor();
        end
        
        
        function update(obj, q, q_dot, q_ddot)
            % Assign the system states q, q_dot, q_ddot
            % Calls set state for BodyKinematics and CableKinematics, and
            % sets the system Jacobian matrix
%             obj.q = q;
%             obj.q_dot = q_dot;
%             obj.q_ddot = q_ddot;
            
            obj.bodyKinematics.update(q, q_dot, q_ddot);
            obj.cableKinematics.update(obj.bodyKinematics);
            %obj.L = obj.cableKinematics.V*obj.bodyKinematics.W;
        end
                
        function value = get.numLinks(obj)
            value = obj.bodyKinematics.numLinks;
        end
        
        function value = get.numDofs(obj)
            value = obj.bodyKinematics.numDofs;
        end
        
        function value = get.numCables(obj)
            value = obj.cableKinematics.numCables;
        end
        
        function value = get.cableLengths(obj)
            value = zeros(obj.numCables,1);
            for i = 1:obj.numCables
                value(i) = obj.cableKinematics.cables{i}.length;
            end
        end
        
        function value = get.cableLengthsDot(obj)
            value = obj.L*obj.q_dot;
        end
        
        function value = get.q(obj)
            value = obj.bodyKinematics.q;
        end
        
        function value = get.q_dot(obj)
            value = obj.bodyKinematics.q_dot;
        end
        
        function value = get.q_ddot(obj)
            value = obj.bodyKinematics.q_ddot;
        end
        
        function value = get.V(obj)
            value = obj.cableKinematics.V;
        end
        
        function value = get.W(obj)
            value = obj.bodyKinematics.W;
        end
        
        function value = get.S(obj)
            value = obj.bodyKinematics.S;
        end
        
        function value = get.S_dot(obj)
            value = obj.bodyKinematics.S_dot;
        end
        
        function value = get.P(obj)
            value = obj.bodyKinematics.P;
        end
        
        function value = get.L(obj)
            value = obj.V*obj.W;
        end
    end
    
end

