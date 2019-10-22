% The main GUI file for CASPR
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
%    Creates the main GUI window for users to use CASPR using a more
%    friendly interface. This is the main interface that connects to the
%    other GUI windows that perform more specific analyses.

%--------------------------------------------------------------------------
%% Constructor
%--------------------------------------------------------------------------
function varargout = CASPR_GUI(varargin)
    % CASPR_GUI MATLAB code for CASPR_GUI.fig
    %      CASPR_GUI, by itself, creates a new CASPR_GUI or raises the existing
    %      singleton*.
    %
    %      H = CASPR_GUI returns the handle to a new CASPR_GUI or the handle to
    %      the existing singleton*.
    %
    %      CASPR_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
    %      function named CALLBACK in CASPR_GUI.M with the given input arguments.
    %
    %      CASPR_GUI('Property','Value',...) creates a new CASPR_GUI or raises the
    %      existing singleton*.  Starting from the left, property value pairs are
    %      applied to the GUI before CASPR_GUI_OpeningFcn gets called.  An
    %      unrecognized property name or invalid value makes property application
    %      stop.  All inputs are passed to CASPR_GUI_OpeningFcn via varargin.
    %
    %      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
    %      instance to run (singleton)".
    %
    % See also: GUIDE, GUIDATA, GUIHANDLES

    % Edit the above text to modify the response to help CASPR_GUI

    % Last Modified by GUIDE v2.5 13-Nov-2018 13:01:30

    % Begin initialization code - DO NOT EDIT
    warning('off','MATLAB:uitabgroup:OldVersion')
    gui_Singleton = 1;
    gui_State = struct('gui_Name',       mfilename, ...
                       'gui_Singleton',  gui_Singleton, ...
                       'gui_OpeningFcn', @CASPR_GUI_OpeningFcn, ...
                       'gui_OutputFcn',  @CASPR_GUI_OutputFcn, ...
                       'gui_LayoutFcn',  [] , ...
                       'gui_Callback',   []);
    if nargin && ischar(varargin{1})
        gui_State.gui_Callback = str2func(varargin{1});
    end

    if nargout
        [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
    else
        gui_mainfcn(gui_State, varargin{:});
    end
    % End initialization code - DO NOT EDIT
end

%--------------------------------------------------------------------------
%% GUI Setup Functions
%--------------------------------------------------------------------------
% --- Executes just before CASPR_GUI is made visible.
function CASPR_GUI_OpeningFcn(hObject, ~, handles, varargin)
    % This function has no output args, see OutputFcn.
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % varargin   command line arguments to CASPR_GUI (see VARARGIN)

    % Choose default command line output for CASPR_GUI
    handles.output = hObject;

    % Update handles structure
    guidata(hObject, handles);
    
%     Load previous information
    axes(handles.plot_axis)
    plot(rand(5)); % Hack to fix size.  Ideally removed at some point.
    loadState(handles);
    
    % UIWAIT makes CASPR_GUI wait for user response (see UIRESUME)
    % uiwait(handles.figure1);
end

% --- Outputs from this function are returned to the command line.
function varargout = CASPR_GUI_OutputFcn(~, ~, handles)
    % varargout  cell array for returning output args (see VARARGOUT);
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Get default command line output from handles structure
    varargout{1} = handles.output;
end

% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, ~, handles) %#ok<DEFNU>
    % hObject    handle to figure1 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hint: delete(hObject) closes the figure
    saveState(handles);
    delete(hObject);
end

% --- Executes when figure1 is resized.
function figure1_ResizeFcn(~, ~, ~) %#ok<DEFNU>
    % hObject    handle to figure1 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
%     rect = get(0,'ScreenSize');
%     set(hObject,'Position',rect);
end

%--------------------------------------------------------------------------
%% Menu Functions
%--------------------------------------------------------------------------
% --------------------------------------------------------------------
function FileMenu_Callback(~, ~, ~) %#ok<DEFNU>
    % hObject    handle to FileMenu (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
end

% --------------------------------------------------------------------
function OpenMenuItem_Callback(~, ~, ~)  %#ok<DEFNU>
    % hObject    handle to OpenMenuItem (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    file = uigetfile('*.fig');
    if ~isequal(file, 0)
        open(file);
    end
end 
    
% --------------------------------------------------------------------
function PrintMenuItem_Callback(~, ~, handles) %#ok<DEFNU>
    % hObject    handle to PrintMenuItem (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    printdlg(handles.figure1)
end

% --------------------------------------------------------------------
function CloseMenuItem_Callback(~, ~, handles) %#ok<DEFNU>
    % hObject    handle to CloseMenuItem (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    selection = questdlg(['Close ' get(handles.figure1,'Name') '?'],...
                         ['Close ' get(handles.figure1,'Name') '...'],...
                         'Yes','No','Yes');
    if strcmp(selection,'No')
        return;
    end
    delete(handles.figure1)
end

%--------------------------------------------------------------------------
%% Popups
%--------------------------------------------------------------------------
% Model Popup
% --- Executes on selection change in model_popup.
function model_popup_Callback(~, ~, handles) %#ok<DEFNU>
    % hObject    handle to model_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = get(hObject,'String') returns model_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from model_popup
    cable_popup_update(handles);
end

% --- Executes during object creation, after setting all properties.
function model_popup_CreateFcn(hObject, ~, ~) %#ok<DEFNU>
    % hObject    handle to model_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
    set(hObject, 'String', {' '});
end

% Cable Popup
% --- Executes on selection change in cable_popup.
function cable_popup_Callback(~, ~, handles) %#ok<DEFNU>
    % hObject    handle to cable_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns cable_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from cable_popup
    generate_model_object(handles);
end

% --- Executes during object creation, after setting all properties.
function cable_popup_CreateFcn(hObject, ~, ~) %#ok<DEFNU>
    % hObject    handle to cable_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
    set(hObject, 'String', {'Choose a Model'});
end

%--------------------------------------------------------------------------
%% Push Buttons
%--------------------------------------------------------------------------
% Dynamics 
% --- Executes on button press in dynamics_button.
function dynamics_button_Callback(~, ~, handles) %#ok<DEFNU>
    % hObject    handle to dynamics_button (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    saveState(handles);
    dynamics_GUI;
end

% Kinematics
% --- Executes on button press in kinematics_button.
function kinematics_button_Callback(~, ~, handles) %#ok<DEFNU>
    % hObject    handle to kinematics_button (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    saveState(handles);
    kinematics_GUI;
end

% Workspace
% --- Executes on button press in workspace_button.
function workspace_button_Callback(~, ~, handles) %#ok<DEFNU>
    % hObject    handle to workspace_button (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    saveState(handles);
    workspace_GUI;
end

% --- Executes on button press in update_button.
function update_button_Callback(~, ~, handles) %#ok<DEFNU>
% hObject    handle to update_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    % modObj = getappdata(handles.cable_popup,'modObj');
    %
    q_data = get(handles.qtable,'Data');
    generate_model_object(handles);
    modObj = getappdata(handles.cable_popup,'modObj');
    set(handles.qtable,'Data',q_data);
    modObj.update(q_data',zeros(modObj.numDofVars,1),zeros(modObj.numDofVars,1),zeros(modObj.numDofVars,1));
    cla;
    axis_range = getappdata(handles.cable_popup,'axis_range');
    view_angle = getappdata(handles.cable_popup,'view_angle');
    MotionSimulatorBase.PlotFrame(modObj, axis_range, view_angle, handles.figure1, handles.plot_axis);
end

% --- Executes on button press in control_button.
function control_button_Callback(~, ~, handles) %#ok<DEFNU>
    % hObject    handle to control_button (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    saveState(handles);
    control_GUI;
end

% --- Executes on button press in console_pushbutton.
function console_pushbutton_Callback(~, ~, handles) %#ok<DEFNU>
    % hObject    handle to console_pushbutton (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % Load the model 
    modObj = getappdata(handles.cable_popup,'modObj');
    % Assign the model to the base workspace
    assignin('base','model_object',modObj);
end

% Model update button
% --- Executes on button press in model_update_button.
function missing_term_error = model_update_button_Callback(~, ~, handles) %#ok<DEFNU>
    % hObject    handle to model_update_button (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    try 
        model_popup_update(handles, get(handles.model_popup,'Value'));
    catch
        model_popup_update(handles);
    end
    % Update the cable set list
    % Generate the model_config object
    contents = cellstr(get(handles.model_popup,'String'));
    try
        model_type = contents{get(handles.model_popup,'Value')};
    catch 
        CASPR_log.Warn('Previous model state does not exist anymore. Default to first element.');
        model_type = contents{1};
    end
    if(CASPR_configuration.LoadDevModelConfig())
        model_config = DevModelConfig(model_type);
    else
    	model_config = ModelConfig(model_type);
    end
    contents = cellstr(get(handles.cable_popup,'String'));
    try
        cable_type = contents{get(handles.cable_popup,'Value')}; %#ok<NASGU>
    catch 
        CASPR_log.Warn('Previous cable does not exist anymore. Default to first element.');
        cable_type = contents{1}; %#ok<NASGU>
    end
    cableset_str = model_config.getCableSetList();
    set(handles.cable_popup, 'String', cableset_str);
    % Generate the new model
    generate_model_object(handles);
end

%--------------------------------------------------------------------------
% Additional Functions
%--------------------------------------------------------------------------
function generate_model_object(handles)
    % Generate the dynamics object
    contents = cellstr(get(handles.model_popup,'String'));
    try
        model_type = contents{get(handles.model_popup,'Value')};
    catch 
        CASPR_log.Warn('Previous model state does not exist anymore. Default to first element.');
        model_type = contents{1};
    end
    if(CASPR_configuration.LoadDevModelConfig())
        model_config = DevModelConfig(model_type);
    else
    	model_config = ModelConfig(model_type);
    end
    contents = cellstr(get(handles.cable_popup,'String'));
    try
        cable_set_id = contents{get(handles.cable_popup,'Value')};
    catch 
        CASPR_log.Warn('Previous cable set state does not exist anymore. Default to first element.');
        cable_set_id = contents{1};
    end
    modObj = model_config.getModel(cable_set_id);
    cla;
    display_range = model_config.displayRange;
    view_angle = model_config.viewAngle;
    MotionSimulatorBase.PlotFrame(modObj, display_range, view_angle, handles.figure1, handles.plot_axis);
    % Store the dynamics object
    setappdata(handles.cable_popup,'modObj',modObj);
    setappdata(handles.cable_popup,'axis_range',display_range);
    setappdata(handles.cable_popup,'view_angle',view_angle);
    set(handles.model_label_text,'String',model_type);
    format_q_table(modObj.numDofs,handles.qtable,modObj.q');
end

function model_popup_update(handles, model_popup_value)
    % Determine the state of the toggle
    if(CASPR_configuration.LoadDevModelConfig())
        e_list_str      =   sort(ModelConfigManager.GetDevModelConfigListNames());
    else
    	e_list_str      =   sort(ModelConfigManager.GetModelConfigListNames());
    end
    if nargin > 1
        set(handles.model_popup, 'Value', model_popup_value);
    else
        set(handles.model_popup, 'Value', 1);
    end
    set(handles.model_popup, 'String', e_list_str);
end

function cable_popup_update(handles)
    % Generate the model_config object
    contents = cellstr(get(handles.model_popup,'String'));
    try
        model_type = contents{get(handles.model_popup,'Value')};
    catch 
        CASPR_log.Warn('Previous model state does not exist anymore. Default to first element.');
        model_type = contents{1};
    end
    if(CASPR_configuration.LoadDevModelConfig())
        model_config = DevModelConfig(model_type);
    else
    	model_config = ModelConfig(model_type);
    end
    cableset_str = model_config.getCableSetList();
    set(handles.cable_popup, 'Value', 1);
    set(handles.cable_popup, 'String', cableset_str);
    generate_model_object(handles);
end

function saveState(handles)
    % Save all of the settings
    state.model_popup_value     =   get(handles.model_popup,'value');
    state.cable_popup_value     =   get(handles.cable_popup,'value');
    contents                    =   get(handles.model_popup,'String');
    state.model_text            =   contents{state.model_popup_value};
    contents                    =   get(handles.cable_popup,'String');
    state.cable_text            =   contents{state.cable_popup_value};
    modObj                      =   getappdata(handles.cable_popup,'modObj');
    state.modObj                =   modObj;
    path_string = CASPR_configuration.LoadHomePath();
    % Check if the log folder exists
    if(exist([path_string,'/GUI/config'],'dir')~=7)
        mkdir([path_string,'/GUI/config']);        
    end
    save([path_string,'/GUI/config/caspr_gui_state.mat'],'state');
end

function loadState(handles)
    % load all of the settings and initialise the values to match
    file_name = [CASPR_configuration.LoadHomePath(),'/GUI/config/caspr_gui_state.mat'];
    if(exist(file_name,'file'))
        load(file_name);
        model_popup_update(handles);
        % Determine if model parameter still exists
        set(handles.model_popup,'value',state.model_popup_value);
        try
            contents = cellstr(get(handles.model_popup,'String'));
            contents{get(handles.model_popup,'Value')};
        catch
            CASPR_log.Warn('Previous model state does not exist anymore. Default to first element.');
            set(handles.model_popup,'value',1);
        end
        cable_popup_update(handles);
        % Determine if model parameter still exists
        set(handles.cable_popup,'value',state.cable_popup_value);
        try
            contents = cellstr(get(handles.cable_popup,'String'));
            contents{get(handles.cable_popup,'Value')};
        catch
            CASPR_log.Warn('Previous model state does not exist anymore. Default to first element.');
            set(handles.cable_popup,'value',1);
        end
        generate_model_object(handles);
    else
        model_popup_update(handles);
        set(handles.model_popup,'value',1);
        cable_popup_update(handles);
    end
end

function format_q_table(numDofs,qtable,q_data)
    set(qtable,'Data',q_data);
    set(qtable,'ColumnWidth',{30});
    set(qtable,'ColumnEditable',true(1,numDofs));
    column_name = cell(1,numDofs);
    for i = 1:numDofs
        column_name{i} = ['q',num2str(i)];
    end
    set(qtable,'ColumnName',column_name);
end

% --- Executes on button press in Rviz_pushbutton.
function Rviz_pushbutton_Callback(~,~, handles) %#ok<DEFNU>
    modObj = getappdata(handles.cable_popup,'modObj');    
    try 
        load('CARDSFlowConfig.mat');
        interface = CARDSFlowInterface();        
    catch 
        interface = CASPRRVizInterface();        
    end    
    % Set robot name rosparam
    rosparam('set','/robot_name',modObj.robotName);
    rosparam('set','/deleteall',1);
    start_tic = tic;
    while toc(start_tic) < 5       
        interface.visualize(modObj);
    end
end

% --- Executes on button press in model_manager_button.
function model_manager_button_Callback(~, ~, ~) %#ok<DEFNU>
    CASPR_Model_Manager;
end
