% System kinematics of the entire cable robot System
%
% Please cite the following paper when using this for multilink cable
% robots:
% D. Lau, D. Oetomo, and S. K. Halgamuge, "Generalized Modeling of
% Multilink Cable-Driven Manipulators with Arbitrary Routing Using the
% Cable-Routing Matrix," IEEE Trans. Robot., vol. 29, no. 5, pp. 1102-1113,
% Oct. 2013.
%
% Author        : Darwin LAU
% Created       : 2011
% Description    :
%    Data structure that represents the kinematics of both the manipulator
% structure and the cables. The manipulator and cable kinematics are stored
% within the SystemKinematicsBodies and SystemKinematicsCables objects,
% respectively.
%    This class also provides direct access to a range of data (such as the
% generalised coordinates, number of links, DoFs and cables) and matrices
% (such as all of the Jacobian and mapping matrices L, V and W, etc.).
classdef SystemKinematics < handle

    properties (SetAccess = protected)
        bodyKinematics          % SystemKinematicsBodies object
        cableKinematics         % SystemKinematicsCables object
    end

    properties (Dependent)
        numLinks                % Number of links
        numDofs                 % Number of degrees of freedom
        numDofVars              % Number of variables for the DoFs
        numCables               % Number of cables
        cableLengths            % Vector of cable lengths
        cableLengthsDot         % Vector of cable length derivatives

        L                       % cable to joint Jacobian matrix L = VW
        V                       % Cable V matrix
        W                       % Body W matrix, W = PS
        S                       % Body S matrix
        S_dot                   % Body S_dot matrix
        P                       % Body P matrix

        q                       % Generalised coordinates state vector
        q_dot                   % Generalised coordinates derivative
        q_ddot                  % Generalised coordinates double derivative
    end

    methods (Static)
        function k = LoadXmlObj(body_xmlobj, cables_xmlobj)
            k = SystemKinematics();
            k.bodyKinematics = SystemKinematicsBodies.LoadXmlObj(body_xmlobj);
            k.cableKinematics = SystemKinematicsCables.LoadXmlObj(cables_xmlobj, k.bodyKinematics);
            k.update(k.bodyKinematics.q_default, k.bodyKinematics.q_dot_default, k.bodyKinematics.q_ddot_default);
        end
    end

    methods
        % Function updates the kinematics of the bodies and cables of the
        % system with the joint state (q, q_dot and q_ddot)
        function update(obj, q, q_dot, q_ddot)
            obj.bodyKinematics.update(q, q_dot, q_ddot);
            obj.cableKinematics.update(obj.bodyKinematics);
        end

        % The following functions get the dependent variable values
        function value = get.numLinks(obj)
            value = obj.bodyKinematics.numLinks;
        end

        function value = get.numDofs(obj)
            value = obj.bodyKinematics.numDofs;
        end

        function value = get.numDofVars(obj)
            value = obj.bodyKinematics.numDofVars;
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
