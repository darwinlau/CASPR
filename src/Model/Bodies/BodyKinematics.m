classdef (Abstract) BodyKinematics < handle
    %BODYKINEMATICS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        joint                   % Joint object
        
        r_G                     % Position vector from joint to COG
        r_P                     % Position vector from joint to end of link
        
        r_Parent                % Position from joint of parent link to this joint
        parentLinkId            % Link ID of the parent
        
        parentLink = []         % Parent link of type BodyKinematics
        childLinks = {}         % Cell array of child links
        
        R_0k = eye(3);          % Rotation matrix ^0_kR where k is current link
        r_OG = zeros(3,1);      % Absolute position vector to centre of gravity in {k}
        r_OP = zeros(3,1);      % Absolute position vector to joint location in (k)
        r_OPe = zeros(3,1);     % Absolute position vector to end of link in {k}
        
        v_OG = zeros(3,1);      % Absolute velocity vector to centre of gravity in {k}
        w = zeros(3,1);         % Absolute angular velocity vector in {k}
        
        a_OG = zeros(3,1);      % Absolute linear acceleration vector to centre of gravity in {k}
        w_dot = zeros(3,1);     % Absolute angular acceleration vector in {k}
    end
    
    properties (SetAccess = private)
        id                      % Body ID
        name
    end
    
    properties (Dependent)
        numDofs
        numDofVars
    end
   
    methods
        function bk = BodyKinematics(id, name, jointType)
            bk.id = id;
            bk.name = name;
            bk.joint = Joint.CreateJoint(jointType);
        end
        
        function addParent(obj, parent, r_parent_loc)
            % Note that link is child
            obj.r_Parent = r_parent_loc;
            if (~isempty(parent))
                parent.childLinks{length(parent.childLinks)+1} = obj;
                obj.parentLink = parent;
            end
        end        
        
        function dofs = get.numDofs(obj)   
            dofs = obj.joint.numDofs;
        end
        
        function dofs = get.numDofVars(obj)   
            dofs = obj.joint.numVars;
        end
        
        function update(obj, q, q_dot, q_ddot)
            obj.joint.update(q, q_dot, q_ddot);
        end
    end
end

