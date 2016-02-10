%--------------------------------------------------------------------------
%% Constructor
%--------------------------------------------------------------------------
function varargout = workspaceGUI(varargin)
    % WORKSPACEGUI MATLAB code for workspaceGUI.fig
    %      WORKSPACEGUI, by itself, creates a new WORKSPACEGUI or raises the existing
    %      singleton*.
    %
    %      H = WORKSPACEGUI returns the handle to a new WORKSPACEGUI or the handle to
    %      the existing singleton*.
    %
    %      WORKSPACEGUI('CALLBACK',hObject,eventData,handles,...) calls the local
    %      function named CALLBACK in WORKSPACEGUI.M with the given input arguments.
    %
    %      WORKSPACEGUI('Property','Value',...) creates a new WORKSPACEGUI or raises the
    %      existing singleton*.  Starting from the left, property value pairs are
    %      applied to the GUI before workspaceGUI_OpeningFcn gets called.  An
    %      unrecognized property name or invalid value makes property application
    %      stop.  All inputs are passed to workspaceGUI_OpeningFcn via varargin.
    %
    %      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
    %      instance to run (singleton)".
    %
    % See also: GUIDE, GUIDATA, GUIHANDLES

    % Edit the above text to modify the response to help workspaceGUI

    % Last Modified by GUIDE v2.5 14-Jan-2016 14:25:24

    % Begin initialization code - DO NOT EDIT
    gui_Singleton = 1;
    gui_State = struct('gui_Name',       mfilename, ...
                       'gui_Singleton',  gui_Singleton, ...
                       'gui_OpeningFcn', @workspaceGUI_OpeningFcn, ...
                       'gui_OutputFcn',  @workspaceGUI_OutputFcn, ...
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
% --- Executes just before workspaceGUI is made visible.
function workspaceGUI_OpeningFcn(hObject, ~, handles, varargin)
    % This function has no output args, see OutputFcn.
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % varargin   command line arguments to workspaceGUI (see VARARGIN)

    % Choose default command line output for workspaceGUI    
    handles.output = hObject;
    
    % Update handles structure
    guidata(hObject, handles);
    loadState(handles);
    % UIWAIT makes workspaceGUI wait for user response (see UIRESUME)
    % uiwait(handles.figure1);
end

% --- Outputs from this function are returned to the command line.
function varargout = workspaceGUI_OutputFcn(~, ~, handles) 
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


%--------------------------------------------------------------------------
%% Popups
%--------------------------------------------------------------------------
% Workspace Condition
% --- Executes on selection change in workspace_condition_popup.
function workspace_condition_popup_Callback(~, ~, handles) %#ok<DEFNU>
    % hObject    handle to workspace_condition_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns workspace_condition_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from workspace_condition_popup
    workspace_generation_popup_Update(handles.workspace_generation_popup,handles);
end

% --- Executes during object creation, after setting all properties.
function workspace_condition_popup_CreateFcn(hObject, ~, ~) %#ok<DEFNU>
    % hObject    handle to workspace_condition_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
    path_string = fileparts(mfilename('fullpath'));
    path_string = path_string(1:strfind(path_string, 'GUI')-2);
    settingsXMLObj =  XmlOperations.XmlReadRemoveIndents([path_string,'\GUI\XML\workspaceXML.xml']); 
    setappdata(hObject,'settings',settingsXMLObj);
    workspacesObj = settingsXMLObj.getElementsByTagName('simulator').item(0).getElementsByTagName('workspace_condition');
    workspace_str = cell(1,workspacesObj.getLength);
    % Extract the identifies from the cable sets
    for i =1:workspacesObj.getLength
        workspaceObj = workspacesObj.item(i-1);
        workspace_str{i} = char(workspaceObj.getAttribute('id'));
    end
    set(hObject, 'String', workspace_str);
end

% Workspace Generation
% --- Executes on selection change in workspace_generation_popup.
function workspace_generation_popup_Callback(~, ~, ~) %#ok<DEFNU>
    % hObject    handle to workspace_generation_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns workspace_generation_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from workspace_generation_popup
end

function workspace_generation_popup_Update(hObject,handles)
    contents = cellstr(get(handles.workspace_condition_popup,'String'));
    workspace_condition_id = contents{get(handles.workspace_condition_popup,'Value')};
    settings = getappdata(handles.workspace_condition_popup,'settings');
    workspaceObj = settings.getElementById(workspace_condition_id);
    enum_file = workspaceObj.getElementsByTagName('generation_method_enum').item(0).getFirstChild.getData;
    workspace_generation_list = eval([char(enum_file),'.workspace_method_list()']);
    set(hObject,'Value',1);
    set(hObject, 'String', workspace_generation_list);
end

% --- Executes during object creation, after setting all properties.
function workspace_generation_popup_CreateFcn(hObject, ~, ~) %#ok<DEFNU>
    % hObject    handle to workspace_generation_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end

% Workspace Metric
% --- Executes on selection change in workspace_metric_popup.
function workspace_metric_popup_Callback(~, ~, ~) %#ok<DEFNU>
    % hObject    handle to workspace_metric_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns workspace_metric_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from workspace_metric_popup
end

% --- Executes during object creation, after setting all properties.
function workspace_metric_popup_CreateFcn(hObject, ~, ~) %#ok<DEFNU>
    % hObject    handle to workspace_metric_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
    path_string = fileparts(mfilename('fullpath'));
    path_string = path_string(1:strfind(path_string, 'GUI')-2);
    settingsXMLObj =  XmlOperations.XmlReadRemoveIndents([path_string,'\GUI\XML\workspaceXML.xml']);
    workspacesObj = settingsXMLObj.getElementsByTagName('simulator').item(0).getElementsByTagName('workspace_metrics').item(0).getElementsByTagName('workspace_metric');
    workspace_str = cell(1,workspacesObj.getLength+1);
    % Extract the identifies from the cable sets
    workspace_str{1} = ' ';
    for i =1:workspacesObj.getLength
        workspaceObj = workspacesObj.item(i-1);
        workspace_str{i+1} = char(workspaceObj.getFirstChild.getData);
    end
    set(hObject, 'String', workspace_str);
end

% --- Executes on selection change in grid_popup.
function grid_popup_Callback(hObject, ~, handles) %#ok<INUSD,DEFNU>
    % hObject    handle to grid_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns grid_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from grid_popup
end

% --- Executes during object creation, after setting all properties.
function grid_popup_CreateFcn(hObject, ~, ~) %#ok<DEFNU>
    % hObject    handle to grid_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
    path_string = fileparts(mfilename('fullpath'));
    path_string = path_string(1:strfind(path_string, 'GUI')-2);
    settingsXMLObj =  XmlOperations.XmlReadRemoveIndents([path_string,'\GUI\XML\workspaceXML.xml']);
    workspacesObj = settingsXMLObj.getElementsByTagName('simulator').item(0).getElementsByTagName('grid_types').item(0).getElementsByTagName('grid_type');
    workspace_str = cell(1,workspacesObj.getLength);
    % Extract the identifies from the cable sets
    for i =1:workspacesObj.getLength
        workspaceObj = workspacesObj.item(i-1);
        workspace_str{i} = char(workspaceObj.getFirstChild.getData);
    end
    set(hObject, 'String', workspace_str);
end

%--------------------------------------------------------------------------
%% Push Buttons
%--------------------------------------------------------------------------
% --- Executes on button press in save_button.
function save_button_Callback(~, ~, handles) %#ok<DEFNU>
    % hObject    handle to save_button (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    path_string = fileparts(mfilename('fullpath'));
    path_string = path_string(1:strfind(path_string, 'GUI')-2);
    file_name = [path_string,'\logs\*.mat'];
    [file,path] = uiputfile(file_name,'Save file name');
    saveState(handles,[path,file]);
end

% --- Executes on button press in load_button.
function load_button_Callback(~, ~, handles) %#ok<DEFNU>
    % hObject    handle to load_button (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    path_string = fileparts(mfilename('fullpath'));
    path_string = path_string(1:strfind(path_string, 'GUI')-2);
    file_name = [path_string,'\logs\*.mat'];
    settings = uigetfile(file_name);
    load(settings)
    mp_text = get(handles.model_text,'String');
    cs_text = get(handles.cable_text,'String');
    if(strcmp(mp_text,state.model_text)&&strcmp(cs_text,state.cable_text))
        set(handles.workspace_condition_popup,'value',state.workspace_condition_popup_value);
        workspace_generation_popup_Update(handles.workspace_generation_popup,handles);
        set(handles.workspace_generation_popup,'value',state.workspace_generation_popup_value);
        set(handles.workspace_metric_popup,'value',state.workspace_metric_popup_value);
        set(handles.qtable,'Data',state.workspace_table);
    else
        warning('Incorrect Model Type');
    end
end

% --- Executes on button press in Generate_Button.
function Generate_Button_Callback(~, ~, handles) %#ok<DEFNU>
    % hObject    handle to Generate_Button (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    %% Clear data
    clc; warning off; %#ok<WNOFF> %close all;
    %% Model setup
    dynObj = getappdata(handles.cable_text,'dynObj');
    %% Workspace Setup
    % First the condition
    contents = cellstr(get(handles.workspace_condition_popup,'String'));
    cfh = str2func(contents{get(handles.workspace_condition_popup,'Value')});
    contents = cellstr(get(handles.workspace_generation_popup,'String'));
    wcondition  = cfh(contents{get(handles.workspace_generation_popup,'Value')});
%     wcondition  = cfh(contents{get(handles.workspace_generation_popup,'Value')},[-1,1,0,0;0,0,-1,1]);
    % Then the metric
    contents = cellstr(get(handles.workspace_metric_popup,'String'));
    if(strcmp(contents{get(handles.workspace_metric_popup,'Value')},' '))
        metric = NullMetric();
    else
        mfh = str2func(contents{get(handles.workspace_metric_popup,'Value')});
        metric = mfh();
    end
    %% Now initialise the simulation
    disp('Start Setup Simulation');
    start_tic       =   tic;
    wsim            =   WorkspaceSimulator(dynObj,wcondition,metric);
%     q_step          =   pi/18;
    q_info = get(handles.qtable,'Data');
    uGrid           =   UniformGrid(q_info(:,1),q_info(:,2),q_info(:,3));
    %% Now set up the grid information
    time_elapsed    =   toc(start_tic);
    fprintf('End Setup Simulation : %f seconds\n', time_elapsed);
    
    disp('Start Running Simulation');
    start_tic       =   tic;
    wsim.run(uGrid);
    time_elapsed    =   toc(start_tic);
    fprintf('End Running Simulation : %f seconds\n', time_elapsed);
    
    disp('Start Plotting Simulation');
    start_tic = tic;
    axes(handles.workspace_axes);
    cla;
    wsim.plotWorkspace();
    % wsim.plotWorkspaceHigherDimension()
    time_elapsed = toc(start_tic);
    fprintf('End Plotting Simulation : %f seconds\n', time_elapsed);
    assignin('base','wsim',wsim);
end

% --- Executes on button press in plot_button.
function plot_button_Callback(~, ~, ~) %#ok<DEFNU>
    % hObject    handle to plot_button (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
end

%--------------------------------------------------------------------------
%% Text Functions
%--------------------------------------------------------------------------
function model_text_CreateFcn(hObject, ~, ~) %#ok<DEFNU>
    % hObject    handle to grid_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end

function cable_text_CreateFcn(hObject, ~, ~) %#ok<DEFNU>
    % hObject    handle to grid_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end

%--------------------------------------------------------------------------
%% Additional Functions
%--------------------------------------------------------------------------
function saveState(handles,file_path)
    % Save all of the settings
    state.model_text                        =   get(handles.model_text,'String');
    state.cable_text                        =   get(handles.cable_text,'String');
    state.workspace_condition_popup_value   =   get(handles.workspace_condition_popup,'value');
    state.workspace_generation_popup_value  =   get(handles.workspace_generation_popup,'value');
    state.workspace_metric_popup_value      =   get(handles.workspace_metric_popup,'value');
    state.workspace_table                   =   get(handles.qtable,'Data');
    if(nargin>1)
        save(file_path,'state');
    else
        path_string                             =   fileparts(mfilename('fullpath'));
        path_string                             =   path_string(1:strfind(path_string, 'GUI')-2);
        save([path_string,'\logs\workspace_gui_state.mat'],'state')
    end
end

function loadState(handles)
    % load all of the settings and initialise the values to match
    path_string = fileparts(mfilename('fullpath'));
    path_string = path_string(1:strfind(path_string, 'GUI')-2);
    file_name = [path_string,'\logs\upcra_gui_state.mat'];
    if(exist(file_name,'file'))
        load(file_name)
        set(handles.model_text,'String',state.model_text);
        set(handles.cable_text,'String',state.cable_text);
        setappdata(handles.cable_text,'dynObj',state.dynObj);
        file_name = [path_string,'\logs\workspace_gui_state.mat'];
        format_q_table(state.dynObj.numDofs,handles.qtable)
        if(exist(file_name,'file'))
            load(file_name);
            mp_text = get(handles.model_text,'String');
            cs_text = get(handles.cable_text,'String');
            if(strcmp(mp_text,state.model_text)&&strcmp(cs_text,state.cable_text))
                set(handles.workspace_condition_popup,'value',state.workspace_condition_popup_value);
                workspace_generation_popup_Update(handles.workspace_generation_popup,handles);
                set(handles.workspace_generation_popup,'value',state.workspace_generation_popup_value);
                set(handles.workspace_metric_popup,'value',state.workspace_metric_popup_value);
                set(handles.qtable,'Data',state.workspace_table);
            else
                workspace_generation_popup_Update(handles.workspace_generation_popup,handles);
            end
        end
    end
end

function format_q_table(numDofs,qtable)
    set(qtable,'Data',zeros(numDofs,3));
    set(qtable,'ColumnWidth',{50});
    set(qtable,'ColumnEditable',true(1,3));
    set(qtable,'ColumnName',{'q_start','q_end','q_step'});
    q_position = get(qtable,'Position');
    if(numDofs>2)
        q_position(1) = 103;
        q_position(3) = 200;
        set(qtable,'Position',q_position);
    else
        q_position(1) = 118;
        q_position(3) = 185;
        set(qtable,'Position',q_position);
    end
end



%% TO BE DONE
% Generate Workspace Plotting Functions
% Add new Methods for computing Wrench Closure
% Determine where best to store settings