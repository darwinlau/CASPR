% System kinematics and dynamics of an individual body
%
% Author        : Darwin LAU
% Created       : 2011
% Description    :
classdef (Abstract) BodyModel < handle
    properties
        % Absolute reference positions
        R_0k = eye(3);          % Rotation matrix ^0_kR where k is current link
        r_OG = zeros(3,1);      % Absolute position vector to centre of gravity in {k}
        r_OP = zeros(3,1);      % Absolute position vector to joint location in (k)
        r_OPe = zeros(3,1);     % Absolute position vector to end of link in {k}
        r_Oy = zeros(3,1);      % Absolute position vector to the OP space reference point in {k}

        % Absolute velocities
        v_OG = zeros(3,1);      % Absolute velocity vector to centre of gravity in {k}
        w = zeros(3,1);         % Absolute angular velocity vector in {k}

        % Absolution Accelerations
        a_OG = zeros(3,1);      % Absolute linear acceleration vector to centre of gravity in {k}
        w_dot = zeros(3,1);     % Absolute angular acceleration vector in {k}        
    end
    
    properties (SetAccess = protected)
        % Relative positions
        r_G                     % Position vector from joint to COG
        r_Pe                    % Position vector from joint to end point (only for display purpose)
        r_y = zeros(3,1)        % Position vector from joint to OP space reference point

        % Parent information
        r_Parent                % Position from joint of parent link to this joint
        parentLinkId            % Link ID of the parent
        parentLink = [];        % Parent link of type BodyKinematics
        childLinks = {};        % Cell array of child links
        
        % Inertia
        m                       % Mass of body
        I_G                     % Inertia of body about its centre of mass
    end

    properties (SetAccess = private)
        % Objects
        joint                   % Joint object
        op_space                % Operation Space object
        % Identification
        id                      % Body ID
        name
    end

    properties (Dependent)
        numDofs         % The number of degrees of freedom
        numDofVars      % The number of degrees of freedom variables
        numOPDofs       % The number of operational space degrees of freedom
        isJointActuated % Whether the body is joint actuated
    end

    methods
        % Updates the joint models
        function update(obj, q, q_dot, q_ddot)
            obj.joint.update(q, q_dot, q_ddot);
        end
        
        % Constructor for the body model class
        function bk = BodyModel(id, name, joint)
            bk.id = id;
            bk.name = name;
            bk.joint = joint;
            % Operation Space creation
            bk.op_space = [];
        end

        % Add the parent for the body
        function addParent(obj, parent, r_parent_loc)
            % Note that link is child
            obj.r_Parent = r_parent_loc;
            if (~isempty(parent))
                parent.childLinks{length(parent.childLinks)+1} = obj;
                obj.parentLink = parent;
            end
        end
        
        % Attach the operational space rigid body
        function attachOPSpace(obj,op_space)
            obj.op_space = op_space;
            obj.r_y = op_space.offset;
        end

        % -------
        % Getters
        % -------
        function dofs = get.numDofs(obj)
            dofs = obj.joint.numDofs;
        end

        function dofs = get.numDofVars(obj)
            dofs = obj.joint.numVars;
        end
        
        function dofs = get.numOPDofs(obj)
            if(~isempty(obj.op_space))
                dofs = obj.op_space.numOPDofs;
            else
                dofs = 0;
            end
        end
        
        function val = get.isJointActuated(obj)
            val = obj.joint.isActuated;
        end
    end
end
