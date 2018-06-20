% Contains options for controller simulator
%
% Author        : Chen SONG
% Created       : 2017
% Description   : 
%   This class has all the parameters needed to configure a controller
%   simulator. 
%   The syntex for the parameters are:
%       SimulationFrequencyRatio    ->  sim_freq_ratio
%       ObserverFrequencyRatio      ->  ob_freq_ratio
%       UseAbsoluteEncoder          ->  use_absolute_encoder
%       UseDisturbanceEstimation    ->  use_ob_disturbance_estimation
%       UseStateEstimation          ->  use_ob_state_estimation
%       UseFKInController           ->  use_FK_in_controller
%       UseFKInObserver             ->  use_FK_in_observer
%       EnableFKSolver              ->  enable_FK_solver
%       EnableObserver              ->  enable_observer
%       FKDebugging                 ->  forward_kinematics_debugging
%       IsOpSpaceControl            ->  is_operational_space_control
%       IsSilentMode                ->  is_silent_mode
%   The syntex for constructor call:
%       opt = ControllerSimulatorOptions()
%           -->  use the default options
%       opt = ControllerSimulatorOptions('OptionName1', 'option1', ...)
%           -->  intialize with input upon the default options
%       opt = ControllerSimulatorOptions(opt_ori, 'OptionName1', 'option1', ...)
%           -->  intialize with input upon the existing options (represented by opt_ori here)
classdef ControllerSimulatorOptions < handle
    properties
                                            % assume that the controller frequency is
                                            % defined in the old way (in trajectories)
        sim_freq_ratio                      % frequency ratio between simulation freq and controller freq
        ob_freq_ratio                       % frequency ratio between observer freq and controller freq
        use_absolute_encoder                % if true absolute encoders are used, otherwise relative encoders are used
        use_ob_disturbance_estimation       % if true the estimated disturbance will be used in the controller
        use_ob_state_estimation             % if true the estimated state will be used in the controller
        use_FK_in_controller                % if true FK solver will be used to generate joint feedback for the controller
        use_FK_in_observer                  % if true FK solver will be used to generate joint feedback for the observer
        enable_FK_solver                    % if true FK solver is assumed to be passed to the simulator and enabled
        enable_observer                     % if true observer is assumed to be passed to the simulator and enabled
        forward_kinematics_debugging        % if true, simulator will save all available FK data for future debugging
        is_operational_space_control        % if true the simulator will be cofigured to perform operational space control
        is_silent_mode                   % if true nothing will be printed out when this flag is true (e.g. Compeletion Percentage)
    end
    methods
        % constructors
        function ctrlsimopt = ControllerSimulatorOptions(varargin)
            if (nargin < 1)
                disp('Default options will be applied.');
                ctrlsimopt.defaultOptions();
            elseif (isa(varargin{1}, 'ControllerSimulatorOptions'))
                ctrlsimopt = varargin{1};
                if (mod(nargin-1, 2))
                    disp('Incomplete controller simulator option input, will throw away the broken part.');
                end
                for i=1:floor((nargin - 1)/2)
                    index = 1 + 2*(i - 1) + 1;
                    ctrlsimopt.inputInterpretation(varargin{index}, varargin{index+1});
                end
            else
                ctrlsimopt.defaultOptions();
                if (mod(nargin, 2))
                    disp('Incomplete controller simulator option input, will throw away the broken part.');
                end
                for i=1:floor(nargin/2)
                    index = 1 + 2*(i - 1);
                    ctrlsimopt.inputInterpretation(varargin{index}, varargin{index+1});
                end
            end
        end
        % assign default options
        function defaultOptions(obj)
            obj.sim_freq_ratio                  =   1;
            obj.ob_freq_ratio                   =   1;
            obj.use_absolute_encoder            =   true;
            obj.use_ob_disturbance_estimation   =   false;
            obj.use_ob_state_estimation         =   false;
            obj.use_FK_in_controller            =   false;
            obj.use_FK_in_observer              =   false;
            obj.enable_FK_solver                =   false;
            obj.enable_observer                 =   false;
            obj.forward_kinematics_debugging    =   false;
            obj.is_operational_space_control    =   false;
            obj.is_silent_mode                  =   false;
        end
        
        % process option input
        function inputInterpretation(obj, input_category, input_value)
            if (strcmp(input_category, 'SimulationFrequencyRatio'))
                obj.sim_freq_ratio   =   input_value;
            end
            if (strcmp(input_category, 'ObserverFrequencyRatio'))
                obj.ob_freq_ratio    =   input_value;
            end
            if (strcmp(input_category, 'UseAbsoluteEncoder'))
                if (strcmp(input_value, 'true'))
                    obj.use_absolute_encoder     =   true;
                else
                    obj.use_absolute_encoder     =   false;
                end
            end
            if (strcmp(input_category, 'UseDisturbanceEstimation'))
                if (strcmp(input_value, 'true'))
                    obj.use_ob_disturbance_estimation     =   true;
                else
                    obj.use_ob_disturbance_estimation     =   false;
                end
            end
            if (strcmp(input_category, 'UseStateEstimation'))
                if (strcmp(input_value, 'true'))
                    obj.use_ob_state_estimation     =   true;
                else
                    obj.use_ob_state_estimation     =   false;
                end
            end
            if (strcmp(input_category, 'UseFKInController'))
                if (strcmp(input_value, 'true'))
                    obj.use_FK_in_controller     =   true;
                else
                    obj.use_FK_in_controller     =   false;
                end
            end
            if (strcmp(input_category, 'UseFKInObserver'))
                if (strcmp(input_value, 'true'))
                    obj.use_FK_in_observer     =   true;
                else
                    obj.use_FK_in_observer     =   false;
                end
            end
            if (strcmp(input_category, 'EnableFKSolver'))
                if (strcmp(input_value, 'true'))
                    obj.enable_FK_solver     =   true;
                else
                    obj.enable_FK_solver     =   false;
                end
            end
            if (strcmp(input_category, 'EnableObserver'))
                if (strcmp(input_value, 'true'))
                    obj.enable_observer     =   true;
                else
                    obj.enable_observer     =   false;
                end
            end
            if (strcmp(input_category, 'FKDebugging'))
                if (strcmp(input_value, 'true'))
                    obj.forward_kinematics_debugging     =   true;
                else
                    obj.forward_kinematics_debugging     =   false;
                end
            end
            if (strcmp(input_category, 'IsOpSpaceControl'))
                if (strcmp(input_value, 'true'))
                    obj.is_operational_space_control     =   true;
                else
                    obj.is_operational_space_control     =   false;
                end
            end
            if (strcmp(input_category, 'IsSilentMode'))
                if (strcmp(input_value, 'true'))
                    obj.is_silent_mode      =   true;
                else
                    obj.is_silent_mode      =   false;
                end
            end
        end
    end
end