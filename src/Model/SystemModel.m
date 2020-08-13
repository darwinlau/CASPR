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
        robotName               % Name of the robot
        cableSetName            % Name of cable set
        operationalSpaceName    % Name of the operational space
        bodyModel               % SystemModelBodies object
        cableModel              % SystemModelCables object
        modelMode               % The mode of the model
        modelOptions            % Model options
    end

    properties (Constant)
        GRAVITY_CONSTANT = 9.81;
    end
    
    properties (Access = private)
        compiled_lib_name       % Library name for compiled mode
        compiled_L_fn           % Function handle of getting L in compiled mode
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
        
        % Wrench set
        availWrenchSet
        availStaticWrenchSet
        availStaticAccelerationSet
                
        % Operational space coordinates
        y                       % Operational space coordinate vector
        y_dot                   % Operational space coordinate derivative
        y_ddot                  % Operational space coordinate double derivative
        
        isSymbolic
    end

    methods (Static)
        % Load the xml objects for bodies and cable sets.
        function b = LoadXmlObj(robot_name, body_xmlobj, cable_set_id, cable_xmlobj, op_space_id, op_space_set_xmlobj, model_mode, model_options)
            if (nargin < 7)
                model_mode = ModelModeType.DEFAULT;
            elseif (nargin < 8)
                model_options = ModelOptions();
            end
            if (model_mode == ModelModeType.COMPILED)
                [bodies_lib_name, cableset_lib_name, opset_lib_name] = ModelConfigBase.ConstructCompiledLibraryNames(robot_name, cable_set_id, op_space_id);
            else 
                bodies_lib_name = '';
                cableset_lib_name = '';
                opset_lib_name = '';
            end
            
            b               =   SystemModel(robot_name, cable_set_id, op_space_id, model_mode, model_options, cableset_lib_name);
            b.bodyModel     =   SystemModelBodies.LoadXmlObj(body_xmlobj, op_space_set_xmlobj, b.modelMode, model_options, bodies_lib_name, opset_lib_name);
            b.cableModel    =   SystemModelCables.LoadXmlObj(cable_xmlobj, b.bodyModel, b.modelMode, model_options, cableset_lib_name);
                
            if ((model_mode == ModelModeType.DEFAULT) || (model_mode == ModelModeType.COMPILED))
                b.update(b.bodyModel.q_initial, b.bodyModel.q_dot_default, b.bodyModel.q_ddot_default, zeros(b.numDofs,1));
            elseif (model_mode == ModelModeType.SYMBOLIC)
                % Create symbolic variables
                q = sym('q', [b.numDofs,1], 'real'); 
                q_d = sym('q_d', [b.numDofs,1], 'real');
                q_dd = sym('q_dd', [b.numDofs,1], 'real'); 
                w_ext = sym('w_ext', [b.numDofs,1], 'real');                               
                b.update(q, q_d, q_dd, w_ext);
            end
        end
    end

    methods
        % Constructor
        function b = SystemModel(robot_name, cable_set_id, op_space_id, mode, model_options, compiled_lib_name)
            if (mode == ModelModeType.COMPILED)
                if (nargin < 6 || isempty(compiled_lib_name))
                    CASPR_log.Error('Library name must be supplied with COMPILED mode');
                else 
                    b.compiled_lib_name = compiled_lib_name;
                    b.compiled_L_fn = str2func([b.compiled_lib_name, '_compiled_L']);
                end
            end
            b.robotName = robot_name;
            if nargin < 4
                mode = ModelModeType.DEFAULT;
            end
            b.modelMode = mode;
            b.modelOptions = model_options;
            b.cableSetName = cable_set_id;
            b.operationalSpaceName = op_space_id;
        end

        % Function updates the kinematics and dynamics of the bodies and
        % cables of the system with the joint state (q, q_dot and q_ddot)
        function update(obj, q, q_dot, q_ddot, w_ext)
            % Assert that the body model uses the correct             
            CASPR_log.Assert(length(q) == obj.bodyModel.numDofVars && length(q_dot) == obj.bodyModel.numDofs ...
                && length(q_ddot) == obj.bodyModel.numDofs && length(w_ext) == obj.bodyModel.numDofs, 'Incorrect input to update function');
            obj.bodyModel.update(q, q_dot, q_ddot, w_ext);              
            obj.cableModel.update(obj.bodyModel);   
        end
        
        % Function that updates the inertia parameters
        function updateInertiaProperties(obj, m, r_G, I_G)
            obj.bodyModel.updateInertiaProperties(m, r_G, I_G);
            obj.update(obj.q, obj.q_dot, obj.q_ddot, obj.W_e);
        end
        
        % Get the linearisation terms for the system.
        function [A,B] = getLinearisedModel(obj)
            % This function assumes that the state input pair for
            % linearisation is given by ([q,qdot],cableForces) as stored by
            % the system.
            
            % First compute L_grad (this also sets the hessian flag to
            % true)
            L_g = obj.L_grad;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % considers the active joints
            % Determine the A matrix using gradient matrices
            % Initialise the A matrix
            A = zeros(2*obj.numDofs);
            % Top left block is zero
            % Top right block is I
            A(1:obj.numDofs,obj.numDofs+1:2*obj.numDofs) = eye(obj.numDofs);
            % Bottom left corner
            A(obj.numDofs+1:2*obj.numDofs,1:obj.numDofs) =  -obj.M\(obj.bodyModel.G_grad + obj.bodyModel.C_grad_q + TensorOperations.VectorProduct(L_g, obj.cableForces,1,isa(obj.q,'symbolic'))) ...
                                                            - TensorOperations.VectorProduct(obj.bodyModel.Minv_grad,(obj.G + obj.C + [obj.L.',-obj.A]*obj.actuationForces),2,isa(obj.q,'symbolic'));
            % Bottom right corner
            A(obj.numDofs+1:2*obj.numDofs,obj.numDofs+1:2*obj.numDofs) = -obj.M\obj.bodyModel.C_grad_qdot;
            
            % The B matrix
            B = [zeros(obj.numDofs,obj.numActuatorsActive);[-obj.M\obj.L_active',obj.M\obj.A]];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
%             if (obj.modelMode == ModelModeType.COMPILED)
%                 value = obj.compiled_L_fn(obj.bodyModel.q,obj.bodyModel.q_dot,obj.bodyModel.q_ddot,obj.bodyModel.W_e);                      
%             else
%                 value = obj.cableModel.V*obj.bodyModel.W;                      
%             end
            value = obj.cableModel.V*obj.bodyModel.W;        
        end
        
        function value = get.L_active(obj)
            value = obj.cableModel.V_active*obj.bodyModel.W;
        end
        
        function value = get.L_passive(obj)
            value = obj.cableModel.V_passive*obj.bodyModel.W;
        end
        
        function value = get.L_grad(obj)
            if obj.modelMode==ModelModeType.COMPILED || ~obj.modelOptions.isComputeHessian
                %CASPR_log.Warn('L_grad is not be computed under compiled mode or if hessian computation is off');
                value = [];
            else
                value = TensorOperations.LeftMatrixProduct(obj.cableModel.V, obj.bodyModel.W_grad, obj.isSymbolic) + TensorOperations.RightMatrixProduct(obj.cableModel.V_grad,obj.bodyModel.W, obj.isSymbolic);
            end
        end
        
        function value = get.L_grad_active(obj)
            if obj.modelMode==ModelModeType.COMPILED || ~obj.modelOptions.isComputeHessian
                %CASPR_log.Warn('L_grad is not be computed under compiled mode or if hessian computation is off');
                value = [];
            else
                value = obj.L_grad(obj.cableModel.cableIndicesActive, :, :);
            end
        end
        
        function value = get.L_grad_passive(obj)
            if obj.modelMode==ModelModeType.COMPILED || ~obj.modelOptions.isComputeHessian
                %CASPR_log.Warn('L_grad is not computed under compiled mode or if hessian computation is off');
                value = [];
            else
                value = obj.L_grad(obj.cableModel.cableIndicesPassive, :, :);
            end
        end
        
        function value = get.K(obj)          
            if(obj.modelMode == ModelModeType.COMPILED  || ~obj.modelOptions.isComputeHessian)
                %CASPR_log.Warn('Stiffness matrix K is not computed under compiled mode or if hessian computation is off');
                value = [];
            else
                value = obj.L.'*obj.cableModel.K*obj.L + TensorOperations.VectorProduct(obj.L_grad, obj.cableForces, 1, obj.isSymbolic);
            end
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
            if isempty(obj.A)
                value = obj.M\(-obj.L.'*obj.cableForces - obj.C - obj.G - obj.W_e);
            else
                value = obj.M\(-obj.L.'*obj.cableForces + obj.A*obj.jointTau - obj.C - obj.G - obj.W_e);
            end
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
        
        function value = get.availWrenchSet(obj)
            value = Zonotope(-obj.L_active', obj.actuationForcesMax, obj.actuationForcesMin, -obj.L_passive'*obj.cableForcesPassive);
        end
        
        function value = get.availStaticWrenchSet(obj)
            value = Zonotope(-obj.L_active', obj.actuationForcesMax, obj.actuationForcesMin, -obj.L_passive'*obj.cableForcesPassive - obj.M'\obj.G);
        end
        
        function value = get.availStaticAccelerationSet(obj)  
            value = Zonotope(-obj.M\obj.L_active', obj.actuationForcesMax, obj.actuationForcesMin, -obj.M\obj.L_passive'*obj.cableForcesPassive - obj.M'\obj.G);
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
                
        function value = get.isSymbolic(obj)
            value = (obj.modelMode == ModelModeType.SYMBOLIC);
        end
                
        % System Compiling Procedure in COMPILED mode
        % This function performs the following steps:
        % 1) Checks if the SystemModelBodies needs to be compiled (if so,
        %   the entire model will compiled)
        % 2) Checks if the SystemModelCables needs to be compiled (if so,
        %   just the cable set will be compiled)
        % 3) Checks if the operational space needs to be compiled (if so,
        %   just the operational space set will be compiled) 
        function compile(obj, base_compile_folder, model_config)
            CASPR_log.Assert(obj.modelMode == ModelModeType.SYMBOLIC, 'Compile function only works when in symbolic mode');
            
            % Function should return bodies folder, cableset folder, and
            % operational set folder; and also corresponding library names?
            
            [b_lib_name, c_lib_name, op_lib_name, cpp_lib_name, b_folder, c_folder, op_folder, cpp_folder, m_folder, cables_base_folder, opset_base_folder] = ModelConfigBase.ConstructCompiledLibraryNames(obj.robotName, obj.cableSetName, obj.operationalSpaceName, base_compile_folder);
            
            [compile_bodies, compile_cables, compile_opspaces] = model_config.checkRequireCompile(obj.cableSetName, obj.operationalSpaceName, base_compile_folder);
            
            if (compile_bodies || compile_cables || compile_opspaces)
                if (exist(base_compile_folder, 'dir'))
                    rmpath(genpath(base_compile_folder));
                end

                if (compile_bodies)
                    if (exist(m_folder, 'dir'))
                        rmdir(m_folder, 's');
                    end
                    mkdir(b_folder);
                    CASPR_log.Info('Start Body Compilations...');
                    obj.bodyModel.compileBodies(b_folder, b_lib_name);                
                    % Record the timestamp file
                    ModelConfigBase.WriteCompileRecordFile(base_compile_folder, ModelConfigBase.COMPILE_RECORD_BODIES_FILENAME);
                end

                if (compile_cables)
                    if (exist(c_folder, 'dir'))
                        rmdir(cables_base_folder, 's');
                    end
                    mkdir(c_folder);                
                    CASPR_log.Info('Start Body Compilations...');
                    obj.cableModel.compile(c_folder, obj.bodyModel, c_lib_name);                
                    % Record the timestamp file
                    ModelConfigBase.WriteCompileRecordFile(base_compile_folder, ModelConfigBase.COMPILE_RECORD_CABLES_FILENAME);
                end

                if (compile_opspaces)
                    if (exist(op_folder, 'dir'))
                        rmdir(opset_base_folder, 's');
                    end
                    mkdir(op_folder);                
                    CASPR_log.Info('Start Body Compilations...');
                    obj.bodyModel.compileOperationalSpace(op_folder, op_lib_name);                
                    % Record the timestamp file
                    ModelConfigBase.WriteCompileRecordFile(base_compile_folder, ModelConfigBase.COMPILE_RECORD_OPERATIONAL_SPACES_FILENAME);
                end
                
%                 % Compile CPP files (START)
%                 CASPR_log.Info('Compiling CPP files...');
%                 if (exist(cpp_folder, 'dir'))
%                     rmdir(cpp_folder, 's');
%                 end
%                 mkdir(cpp_folder);
%                                
%                 % Do the .cpp compile
%                 files_list = [dir([b_folder, '/**/', '*.m']); dir([c_folder, '/**/', '*.m'])];
%                 
%                 if (compile_opspaces)
%                     files_list = [files_list; dir([op_folder, '/**/', '*.m'])];
%                 end
%                 
%                 source_files = cell(1, length(files_list));
%                 for i = 1:length(files_list)
%                     source_files{i} = [files_list(i).folder, '\', files_list(i).name];
%                 end
%                 % Set the input data type for the cpp functions
%                 input_data = {zeros(obj.numDofs,1), zeros(obj.numDofs,1), zeros(obj.numDofs,1), zeros(obj.numDofs,1)};
%                 % Setup the CPP compile config
%                 cpp_code_config = coder.config('dll');
%                 %code_config.IncludeTerminateFcn = false;
%                 cpp_code_config.SupportNonFinite = false;
%                 cpp_code_config.SaturateOnIntegerOverflow = false;
%                 cpp_code_config.GenerateExampleMain = 'DoNotGenerate';
%                 cpp_code_config.TargetLang = 'C++';
%                 cpp_code_config.FilePartitionMethod = 'SingleFile';
%                 % Code generation
%                 str = ['codegen -d ', cpp_folder, ' -o ', cpp_lib_name, ' -config cpp_code_config '];
%                 for i = 1:length(source_files)
%                     str = [str, source_files{i},' -args input_data '];
%                 end
%                 eval(str)
%                 CASPR_log.Info('Finished compiling cpp files');
%              
%                 CASPR_log.Info('Cleaning up cpp compile...');
%                 % Remove unnecessary stuff
%                 rmdir([cpp_folder, '/examples']);
%                 delete([cpp_folder, '/*.mat']);
%                 ModelConfigBase.WriteCompileRecordFile(cpp_folder, ModelConfigBase.COMPILE_RECORD_FILENAME);
%                 % Compile CPP files (END)
                
                % Add the compiled files to the path
                addpath(genpath(base_compile_folder));
            end
        end
    end
end
