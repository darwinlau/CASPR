% System kinematics and dynamics of the entire cable robot system
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
%    Data structure that represents the both the kinematics and dynamics of
% the cable robot system. This object inherits from SystemKinematics, the
% kinematics and dynamics are stored within SystemKinematicsBodies,
% SystemKinematicsCables, SystemDynamicsBodies and SystemDynamicsCables.
%   This class also provides direct access to a range of data and matrices.
% In addition to that from SystemKinematics, the matrices and vectors for
% the equations of motion (M, C, G etc.), the interaction wrenches and its
% magnitudes are available.
classdef SystemModel < handle
    properties (SetAccess = protected)
        bodyModel               % SystemModelBodies object
        cableModel              % SystemModelCables object
        modelMode               % The mode of the model
        filesCompiled           % Flag indicating whether files are compiled
    end

    properties (Constant)
        GRAVITY_CONSTANT = 9.81;
    end

    properties (Dependent)
        % Number of variables
        numLinks                % Number of links
        numDofs                 % Number of degrees of freedom
        numDofVars              % Number of variables for the DoFs
        numDofsJointActuated    % Number of DoFs that are actuated at the joint
        numOperationalDofs      % Number of operational space degrees of freedom
        numCables               % Number of cables
        numCablesActive         % Number of active cables
        numCablesPassive        % Number of active cables
        numActuators            % Number of actuators for the entire system (both cables and joint DoFs)
        numActuatorsActive      % Number of actuators for the entire system that are active
        
        % Generalised coordinates
        q                       % Generalised coordinates state vector
        q_deriv                 % Generalised coordinates time derivative (for special cases q_dot does not equal q_deriv)
        q_dot                   % Generalised coordinates derivative
        q_ddot                  % Generalised coordinates double derivative
        jointTau                % The joint actuator forces for active joints
        
        % Actuator forces
        actuationForces             % vector of actuation forces including both cable forces and actuator forces
        actuationForcesMin          % vector of min active actuation forces
        actuationForcesMax          % vector of max active actuation forces
        ACTUATION_ACTIVE_INVALID    % vector of invalid actuator commands (from both cables and joint actuators)
        
        % Cable lengths and forces
        cableLengths            % Vector of cable lengths
        cableLengthsDot         % Vector of cable length derivatives
        cableForces             % Vector of all cable forces (active and passive)
        cableForcesActive       % Vector of cable forces for active cables
        cableForcesPassive      % Vector of cable forces for passive cables
        
        % Jacobian matrices
        L                       % cable to joint Jacobian matrix L = VW
        L_active                % active part of the Jacobian matrix
        L_passive               % passive part of the Jacobian matrix
        J                       % joint to operational space Jacobian matrix
        J_dot                   % derivative of J
        K                       % The stiffness matrix
                
        % Since S^T w_p = 0 and S^T P^T = W^T
        % W^T ( M_b * q_ddot + C_b ) = W^T ( G_b - V^T f )
        % The equations of motion can then be expressed in the form
        % M * q_ddot + C + G + W_e = - L^T f + A tau
        M
        C
        G
        W_e
        A
        % Instead of q_ddot being passed as input to the system, it could
        % also be determined from:
        % M * q_ddot + C + G + W_e = - L^T f + tau
        q_ddot_dynamics
        
        % Hessians
        L_grad                  % The gradient of the jacobian L
        L_grad_active           % active part of the Jacobian matrix gradient
        L_grad_passive          % active part of the Jacobian matrix gradient
        
        % Linearisation Matrices
        A_lin                   % The state matrix for the system linearisation
        B_lin                   % The input matrix for the system linearisation
                
        % The equations of motion with the interaction wrench w_p
        % P^T ( M_b * q_ddot + C_b ) = P^T ( G_b - V^T f ) + w_p
        interactionWrench               % Joint interaction wrenches (w_p)
        interactionForceMagnitudes      % Magnitudes of the interaction force at each joint
        interactionMomentMagnitudes     % Magnitudes of the interaction moments at each joint
                
        % Operational space coordinates
        y                       % Operational space coordinate vector
        y_dot                   % Operational space coordinate derivative
        y_ddot                  % Operational space coordinate double derivative
    end

    methods (Static)
        % Load the xml objects for bodies and cable sets.
        function b = LoadXmlObj(body_xmlobj, cable_xmlobj, model_mode)
            if((nargin == 2)||(model_mode == ModelModeType.DEFAULT))
                b               =   SystemModel();
                b.modelMode     =   ModelModeType.DEFAULT;
                b.bodyModel     =   SystemModelBodies.LoadXmlObj(body_xmlobj,b.modelMode);                
                b.cableModel    =   SystemModelCables.LoadXmlObj(cable_xmlobj, b.bodyModel,b.modelMode);                
                b.update(b.bodyModel.q_initial, b.bodyModel.q_dot_default, b.bodyModel.q_ddot_default, zeros(b.numDofs,1));
            elseif(model_mode == ModelModeType.SYMBOLIC )
                b               =   SystemModel();
                b.modelMode     =   ModelModeType.SYMBOLIC;
                b.bodyModel     =   SystemModelBodies.LoadXmlObj(body_xmlobj,b.modelMode);                
                b.cableModel    =   SystemModelCables.LoadXmlObj(cable_xmlobj, b.bodyModel,b.modelMode);                
                % Create symbolic variables
                q = sym('q',[b.numDofs,1]); q_d = sym('q_d',[b.numDofs,1]);
                q_dd = sym('q_dd',[b.numDofs,1]); w_ext = sym('w_ext',[b.numDofs,1]);                               
                b.update(q, q_d, q_dd, w_ext);  
            elseif(model_mode == ModelModeType.COMPILED)
                b               =   SystemModel();
                b.modelMode     =   ModelModeType.COMPILED;
                b.bodyModel     =   SystemModelBodies.LoadXmlObj(body_xmlobj,b.modelMode);                
                b.cableModel    =   SystemModelCables.LoadXmlObj(cable_xmlobj, b.bodyModel,b.modelMode);                
                % Create symbolic variables
                q = sym('q',[b.numDofs,1]); q_d = sym('q_d',[b.numDofs,1]);
                q_dd = sym('q_dd',[b.numDofs,1]); w_ext = sym('w_ext',[b.numDofs,1]);                               
                b.setFilesCompiled(false); 
                b.setModelMode(model_mode);
                b.update(q, q_d, q_dd, w_ext);                
            end
        end
    end

    methods
        % Constructor
        function b = SystemModel()
        end

        % Function updates the kinematics and dynamics of the bodies and
        % cables of the system with the joint state (q, q_dot and q_ddot)
        function update(obj, q, q_dot, q_ddot, w_ext)
            % Assert that the body model uses the correct             
            CASPR_log.Assert(length(q) == obj.bodyModel.numDofVars && length(q_dot) == obj.bodyModel.numDofs ...
                && length(q_ddot) == obj.bodyModel.numDofs && length(w_ext) == obj.bodyModel.numDofs,'Incorrect input to update function');
            obj.bodyModel.update(q, q_dot, q_ddot, w_ext);              
            obj.cableModel.update(obj.bodyModel);   
        end
        
        % Function that updates the inertia parameters
        function updateInertiaProperties(obj,m,r_G,I_G,update_flag)
            obj.bodyModel.updateInertiaProperties(m,r_G,I_G);
            if(update_flag)
                obj.update(obj.q,obj.q_dot,obj.q_ddot,obj.W_e);
            end
        end
        
        % Get the linearisation terms for the system.
        function [A,B] = getLinearisedModel(obj)
            % This function assumes that the state input pair for
            % linearisation is given by ([q,qdot],cableForces) as stored by
            % the system.
            
            % First compute L_grad (this also sets the hessian flag to
            % true)
            L_grad_temp = obj.L_grad;
            
            % Had the linearisation been computed previously
            if(isempty(obj.bodyModel.Minv_grad))
                obj.bodyModel.occupied.linearisation = true;
                obj.bodyModel.updateLinearisation();
            end
            
            % Determine the A matrix using gradient matrices
            % Initialise the A matrix
            A = zeros(2*obj.numDofs);
            % Top left block is zero
            % Top right block is I
            A(1:obj.numDofs,obj.numDofs+1:2*obj.numDofs) = eye(obj.numDofs);
            % Bottom left corner
            A(obj.numDofs+1:2*obj.numDofs,1:obj.numDofs) =  -obj.M\(obj.bodyModel.G_grad + obj.bodyModel.C_grad_q + TensorOperations.VectorProduct(L_grad_temp,obj.cableForces,1,isa(obj.q,'symbolic'))) ...
                                                            - TensorOperations.VectorProduct(obj.bodyModel.Minv_grad,(obj.G + obj.C + obj.L.'*obj.cableForces),2,isa(obj.q,'symbolic'));
            % Bottom right corner
            A(obj.numDofs+1:2*obj.numDofs,obj.numDofs+1:2*obj.numDofs) = -obj.M\obj.bodyModel.C_grad_qdot;
            
            % The B matrix
            B = [zeros(obj.numDofs,obj.numCables);-obj.M\obj.L_active'];
        end
        
        % Load the operational space xml object
        function loadOperationalXmlObj(obj,operational_space_xmlobj)
            obj.bodyModel.loadOperationalXmlObj(operational_space_xmlobj);
        end
        
        % -------
        % Getters
        % -------        
        function value = get.numLinks(obj)
            value = obj.bodyModel.numLinks;
        end

        function value = get.numDofs(obj)
            value = obj.bodyModel.numDofs;
        end
        
        function value = get.numOperationalDofs(obj)
            value = obj.bodyModel.numOperationalDofs;
        end

        function value = get.numDofVars(obj)
            value = obj.bodyModel.numDofVars;
        end
        
        function value = get.numDofsJointActuated(obj)
            value = obj.bodyModel.numDofsActuated;
        end

        function value = get.numCables(obj)
            value = obj.cableModel.numCables;
        end
        
        function value = get.numActuators(obj)
            value = obj.numCables + obj.numDofsJointActuated;
        end
        
        function value = get.numActuatorsActive(obj)
            value = obj.numCablesActive + obj.numDofsJointActuated;
        end
            
        function value = get.numCablesActive(obj)
            value = obj.cableModel.numCablesActive;
        end
        
        function value = get.numCablesPassive(obj)
            value = obj.cableModel.numCablesPassive;
        end

        function value = get.cableLengths(obj)
            value = obj.cableModel.lengths;            
        end

        function value = get.cableLengthsDot(obj)
            value = obj.L*obj.q_dot;
        end

        function value = get.q(obj)
            value = obj.bodyModel.q;
        end
        
        function value = get.q_deriv(obj)
            value = obj.bodyModel.q_deriv;
        end

        function value = get.q_dot(obj)
            value = obj.bodyModel.q_dot;
        end

        function value = get.q_ddot(obj)
            value = obj.bodyModel.q_ddot;
        end

        function value = get.L(obj)            
            value = obj.cableModel.V*obj.bodyModel.W;                      
        end
        
        function value = get.L_active(obj)
            value = obj.cableModel.V_active*obj.bodyModel.W;
        end
        
        function value = get.L_passive(obj)
            value = obj.cableModel.V_passive*obj.bodyModel.W;
        end
        
        function value = get.L_grad(obj)
            if(isempty(obj.bodyModel.W_grad))
                if(~obj.bodyModel.occupied.hessian)
                    obj.bodyModel.occupied.hessian = true;
                    obj.bodyModel.updateHessian();
                    obj.cableModel.updateHessian(obj.bodyModel);
                end
            end
            is_symbolic = obj.modelMode == ModelModeType.SYMBOLIC;
            value = TensorOperations.LeftMatrixProduct(obj.cableModel.V,obj.bodyModel.W_grad,is_symbolic) + TensorOperations.RightMatrixProduct(obj.cableModel.V_grad,obj.bodyModel.W,is_symbolic);
        end
        
        function value = get.L_grad_active(obj)
            value = obj.L_grad(obj.cableModel.cableIndicesActive, :, :);
        end
        
        function value = get.L_grad_passive(obj)
            value = obj.L_grad(obj.cableModel.cableIndicesPassive, :, :);
        end
        
        function value = get.K(obj)
            is_symbolic = obj.modelMode == ModelModeType.SYMBOLIC;
            value = obj.L.'*obj.cableModel.K*obj.L + TensorOperations.VectorProduct(obj.L_grad,obj.cableForces,1,is_symbolic);
        end
        
        function value = get.J(obj)
            value = obj.bodyModel.J;
        end
        
        function value = get.J_dot(obj)
            value = obj.bodyModel.J_dot;
        end
        
        function value = get.y(obj)
            value = obj.bodyModel.y;
        end
        
        function value = get.y_dot(obj)
            value = obj.bodyModel.y_dot;
        end
        
        function value = get.y_ddot(obj)
            value = obj.bodyModel.y_ddot;
        end
        
        % Function computes the interaction wrench between the joint of the
        % links.
        % M_b*q_ddot + C_b = G_b +
        function value = get.interactionWrench(obj)
            value = obj.bodyModel.P.'*(obj.cableModel.V.'*obj.cableModel.forces + obj.bodyModel.M_b*obj.q_ddot + obj.bodyModel.C_b - obj.bodyModel.G_b);
        end

        function value = get.interactionForceMagnitudes(obj)
            vector = obj.interactionWrench;
            mag = zeros(obj.numLinks,1);
            for k = 1:obj.numLinks
                mag(k) = norm(vector(6*k-5:6*k-3),2);
            end
            value = mag;
        end

        function value = get.interactionMomentMagnitudes(obj)
            vector = obj.interactionWrench;
            mag = zeros(obj.numLinks,1);
            for k = 1:obj.numLinks
                mag(k) = norm(vector(6*k-2:6*k),2);
            end
            value = mag;
        end

%         function value = get.jointForceAngles(obj)
%             vector = obj.jointWrenches;
%             angle = zeros(obj.numLinks,1);
%             for k = 1:obj.numLinks
%                 angle(k) = atan(norm(vector(6*k-5:6*k-4),2)/abs(vector(6*k-3)))*180/pi;
%             end
%             value = angle;
%         end

        function value = get.q_ddot_dynamics(obj)
            value = obj.M\(-obj.L.'*obj.cableForces + obj.A*obj.jointTau - obj.C - obj.G - obj.W_e);
            % Should we have a function that updates the variable too??
%             obj.bodyModel.q_ddot = obj.M\(-obj.L.'*obj.cableForces + obj.A*obj.jointTau - obj.C - obj.G - obj.W_e);
%             value = obj.q_ddot;
        end

        function value = get.M(obj)
            value = obj.bodyModel.M;
        end

        function value = get.C(obj)
            value = obj.bodyModel.C;
        end

        function value = get.G(obj)
            value = obj.bodyModel.G;
        end
        
        function value = get.W_e(obj)
            value = obj.bodyModel.W_e;
        end
        
        function value = get.A(obj)
            value = obj.bodyModel.A;
        end
        
        function set.actuationForces(obj, f)
            obj.cableModel.forces = f(1:obj.numCablesActive);
            if (obj.numDofsJointActuated > 0)
                obj.bodyModel.tau = f(obj.numCablesActive+1:length(f));
            end
        end
        
        function value = get.actuationForces(obj)
            value = [obj.cableForcesActive; obj.jointTau];
        end
                
        function value = get.cableForces(obj)
            value = obj.cableModel.forces;
        end
        
        function value = get.cableForcesActive(obj)
            value = obj.cableModel.forcesActive;
        end
        
        function value = get.cableForcesPassive(obj)
            value = obj.cableModel.forcesPassive;
        end
                
        function value = get.jointTau(obj)
            value = obj.bodyModel.tau;
        end
        
        function value = get.actuationForcesMin(obj)
            value = [obj.cableModel.forcesActiveMin; obj.bodyModel.tauMin];
        end
        
        function value = get.actuationForcesMax(obj)
            value = [obj.cableModel.forcesActiveMax; obj.bodyModel.tauMax];
        end
        
        function value = get.ACTUATION_ACTIVE_INVALID(obj)
            value = [obj.cableModel.FORCES_ACTIVE_INVALID; obj.bodyModel.TAU_INVALID];
        end
        
        % Uncertainties
        function addInertiaUncertainty(obj,m_bounds,I_bounds)
            CASPR_log.Assert((size(m_bounds,1)==1)&&(length(m_bounds) == 2*obj.numLinks),'Mass uncertainty must be of size 2 * number of links');
            if(nargin == 2)
                obj.bodyModel.addInertiaUncertainty(m_bounds,[]);
            else
                CASPR_log.Assert(((size(I_bounds,1)==1)&&(length(I_bounds) == 12*obj.numLinks)),'Inertia must be of size 12 * number of lins');
                obj.bodyModel.addInertiaUncertainty(m_bounds,I_bounds);
            end
        end
        
        % Model Mode Related Functions %        
        
        % set model mode
        function setModelMode(obj, model_mode)
            obj.modelMode = model_mode;
            obj.bodyModel.setModelMode(model_mode);
            obj.cableModel.setModelMode(model_mode);
        end       
        
        % set filesCompiled Flag
        function setFilesCompiled(obj, value)
            obj.filesCompiled = value;
            obj.bodyModel.setFilesCompiled(value);
            obj.cableModel.setFilesCompiled(value);
        end
        
        % System Compiling Procedure in COMPILED mode
        % - New folders are created in model_config to store the compiled
        % body and cable .m files
        function compile(obj)           
            % Define the paths for storing the compiled files
            model_config_path = CASPR_configuration.LoadModelConfigPath();
            tmp_path = [model_config_path, '/tmp_compilations'];
            tmp_body_path = [tmp_path, '/Bodies'];
            tmp_cable_path = [tmp_path, '/Cables'];
           
            % Remove all the existing compiled files to ensure all compiled
            % files are up-to-date
            if exist(tmp_path)
                warning('off','all');  
                try
                    rmdir(tmp_path, 's');
                    CASPR_log.Info('Previously compiled files are removed.');
                catch
                    CASPR_log.Warn('You might not have the permission to remove previously compiled files');
                    CASPR_log.Warn('Please check your permission before running COMPILED mode.');
                    return;
                end
                warning('on','all');
            else
                % Make folders
                mkdir(tmp_path);  
            end            
            CASPR_log.Info('New files will be compiled.');
                      
            if ~exist(tmp_body_path)
                mkdir(tmp_body_path);              
            end
            if ~exist(tmp_cable_path)
                mkdir(tmp_cable_path);              
            end                        
            CASPR_log.Info('Created Folders.');
            
            % Body Variables Compilations
            CASPR_log.Info('Start Body Compilations...');
            obj.bodyModel.compile(tmp_body_path);
            CASPR_log.Info('Finished Body Compilations.');
            % Cable Variables Compilations
            CASPR_log.Info('Start Cable Compilations...');
            obj.cableModel.compile(tmp_cable_path, obj.bodyModel);
            CASPR_log.Info('Finished Cable Compilations.');
            % System Variables Compilations
            CASPR_log.Info('Start System Compilation...');  
            % Currently only L is needed.
            % If more system variables are needed in the future, a separate
            % function handling these variables might be prefered.
            CASPR_log.Info('- Compiling L...');            
            matlabFunction(obj.L, 'File', strcat(tmp_path, '/compile_L'), 'Vars', {obj.q, obj.q_dot, obj.q_ddot, obj.W_e});
            CASPR_log.Info('Finished L Compilation.');  
            
            % Add the compiled files to the path            
            addpath(genpath(tmp_path));
            
            % Perform an update under compiled mode to initialise
            obj.setFilesCompiled(true);
            obj.setModelMode(ModelModeType.COMPILED);
            obj.update(obj.bodyModel.q_initial, obj.bodyModel.q_dot_default, obj.bodyModel.q_ddot_default, zeros(obj.numDofs,1));
           
            CASPR_log.Info('Finished All Compilations.\n');
        end
    end
end
