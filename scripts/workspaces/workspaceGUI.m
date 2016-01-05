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

    % Last Modified by GUIDE v2.5 05-Jan-2016 09:39:26

    % Begin initialization code - DO NOT EDIT
    path_string = fileparts(mfilename('fullpath'));
    path_string = path_string(1:strfind(path_string, 'scripts')-2);
    addpath(genpath(path_string));
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

% --- Executes just before workspaceGUI is made visible.
function workspaceGUI_OpeningFcn(hObject, eventdata, handles, varargin)
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
function varargout = workspaceGUI_OutputFcn(hObject, eventdata, handles) 
    % varargout  cell array for returning output args (see VARARGOUT);
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Get default command line output from handles structure
    varargout{1} = handles.output;
end


% --- Executes on selection change in model_popup.
function model_popup_Callback(hObject, eventdata, handles)
    % hObject    handle to model_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns model_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from model_popup
    cableset_popup_update(handles);
end


% --- Executes during object creation, after setting all properties.
function model_popup_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to model_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
    e_list      =   enumeration('ModelConfigType');
    e_n         =   length(e_list);
    e_list_str  =   cell(1,e_n);
    for i=1:e_n
        temp_str = char(e_list(i));
        e_list_str{i} = temp_str(3:length(temp_str));
    end
    set(hObject, 'String', e_list_str);
end


% --- Executes on selection change in workspace_type_popup.
function workspace_type_popup_Callback(hObject, eventdata, handles)
    % hObject    handle to workspace_type_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns workspace_type_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from workspace_type_popup    
    workspace_type_index = get(handles.workspace_type_popup, 'Value');
    workspace_condition_popup_Update(handles.workspace_condition_popup,workspace_type_index);
    workspace_generation_popup_Update(handles);
end


% --- Executes during object creation, after setting all properties.
function workspace_type_popup_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to workspace_type_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
    workspace_condition_list = WorkspaceType.workspace_type_list();
    set(hObject, 'String', workspace_condition_list);
end


% --- Executes on selection change in workspace_condition_popup.
function workspace_condition_popup_Callback(hObject, eventdata, handles)
    % hObject    handle to workspace_condition_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns workspace_condition_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from workspace_condition_popup
    workspace_generation_popup_Update(handles);
end


function workspace_condition_popup_Update(hObject,workspace_type_index)
    switch workspace_type_index
        case 1
            subfolder = 'Interference';
        case 2
            subfolder = 'Closure';
        case 3
            subfolder = 'Static';
        case 4
            subfolder = 'Feasible';
    end
    path_string = fileparts(mfilename('fullpath'));
    path_string = path_string(1:strfind(path_string, 'scripts')-2);
    dir_files = dir([path_string,'\src\Analysis\Workspace\Conditions\',subfolder]);
    dir_files_temp = cell(fliplr(size(dir_files))); j =0;
    for i=1:length(dir_files)
        if((~dir_files(i).isdir))
            l_n = length(dir_files(i).name);
            dir_files_temp{j+1} = char(dir_files(i).name(1:l_n-2));
            j = j+1;
        end
    end
    dir_files = dir_files_temp(1:j);
    if(isempty(dir_files))
        set(hObject, 'Value', 1);
        set(hObject, 'String', {'No File'});
    else
        set(hObject, 'Value', 1);
        set(hObject, 'String', dir_files);
    end
end


% --- Executes during object creation, after setting all properties.
function workspace_condition_popup_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to workspace_condition_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
    workspace_condition_popup_Update(hObject,1);
end


% --- Executes on button press in Generate_Button.
function Generate_Button_Callback(hObject, eventdata, handles)
    % hObject    handle to Generate_Button (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    %% Clear data
    clc; warning off; %close all;
    %% Model setup
    dynObj = getappdata(handles.cableset_popup,'dynObj');
    %% Workspace Setup
    % First the condition
    contents = cellstr(get(handles.workspace_condition_popup,'String'));
    cfh = str2func(contents{get(handles.workspace_condition_popup,'Value')});
    contents = cellstr(get(handles.workspace_generation_popup,'String'));
    wcondition  = cfh(contents{get(handles.workspace_generation_popup,'Value')});
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
    q_info = handles.qtable.Data;
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


function cableset_popup_Callback(hObject, eventdata, handles)
    % hObject    handle to cableset_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: get(hObject,'String') returns contents of cableset_popup as text
    %        str2double(get(hObject,'String')) returns contents of cableset_popup as a double
    
    % This block is for the cable set to be written.
    % No callback will be evaluated.
    generate_dynamic_object(handles);
end

function generate_dynamic_object(handles)
    % Generate the dynamics object
    contents = cellstr(get(handles.model_popup,'String'));
    model_type = contents{get(handles.model_popup,'Value')};
    model_config = ModelConfig(ModelConfigType.(['M_',model_type]));
    bodies_xmlobj = model_config.getBodiesProperiesXmlObj();
    contents = cellstr(get(handles.cableset_popup,'String'));
    cable_set_id = contents{get(handles.cableset_popup,'Value')};
    cableset_xmlobj = model_config.getCableSetXmlObj(cable_set_id);
    dynObj = SystemKinematicsDynamics.LoadXmlObj(bodies_xmlobj, cableset_xmlobj);
    % Change the grid
    handles.qtable.Data = zeros(dynObj.numDofs,3);
    handles.qtable.ColumnWidth = {50};
    handles.qtable.ColumnEditable = [true,true,true];
    handles.qtable.ColumnName = {'q_start','q_end','q_step'};
    handles.qtable.Position(1) = 25;  handles.qtable.Position(2) = 2;
    if(dynObj.numDofs >2 )
        handles.qtable.Position(3) = (14/13)*handles.qtable.Extent(3); handles.qtable.Position(4) = 3*handles.qtable.Extent(4)/(dynObj.numDofs+1);
    else
        handles.qtable.Position(3) = handles.qtable.Extent(3); handles.qtable.Position(4) = 3*handles.qtable.Extent(4)/(dynObj.numDofs+1);
    end
    % Store the dynamics object
    setappdata(handles.cableset_popup,'dynObj',dynObj);
end


function cableset_popup_update(handles)
    % Generate the model_config object
    contents = cellstr(get(handles.model_popup,'String'));
    model_type = contents{get(handles.model_popup,'Value')};
    model_config = ModelConfig(ModelConfigType.(['M_',model_type]));
    % Determine the cable sets
    cablesetsObj = model_config.cablesXmlObj.getElementsByTagName('cables').item(0).getElementsByTagName('cable_set');
    cableset_str = cell(1,cablesetsObj.length);
    % Extract the identifies from the cable sets
    for i =1 :cablesetsObj.length
        cablesetObj = cablesetsObj.item(i-1);
        cableset_str{i} = char(cablesetObj.getAttribute('id'));
    end
    set(handles.cableset_popup, 'Value', 1);
    set(handles.cableset_popup, 'String', cableset_str);
    generate_dynamic_object(handles);
end

% --- Executes during object creation, after setting all properties.
function cableset_popup_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to cableset_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
    set(hObject, 'String', {'Choose a Model'});
end


% --- Executes on selection change in workspace_generation_popup.
function workspace_generation_popup_Callback(hObject, eventdata, handles)
    % hObject    handle to workspace_generation_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns workspace_generation_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from workspace_generation_popup
end

function workspace_generation_popup_Update(handles)
    contents = cellstr(get(handles.workspace_condition_popup,'String'));
    if(strcmp(contents,'No File'))
        set(handles.workspace_generation_popup, 'Value', 1);
        set(handles.workspace_generation_popup, 'String', {'No File'});
    else
        fh = str2func([contents{get(handles.workspace_condition_popup,'Value')},'Methods.workspace_method_list']);
        list = fh();
        set(handles.workspace_generation_popup, 'Value', 1);
        set(handles.workspace_generation_popup, 'String', list);
    end
end


% --- Executes during object creation, after setting all properties.
function workspace_generation_popup_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to workspace_generation_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end


% --- Executes on selection change in workspace_metric_popup.
function workspace_metric_popup_Callback(hObject, eventdata, handles)
    % hObject    handle to workspace_metric_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns workspace_metric_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from workspace_metric_popup
end

% --- Executes during object creation, after setting all properties.
function workspace_metric_popup_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to workspace_metric_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
    path_string = fileparts(mfilename('fullpath'));
    path_string = path_string(1:strfind(path_string, 'scripts')-2);
    str_dir = [path_string,'\src\Analysis\Workspace\Metrics'];
    dirinfo = dir(str_dir);
    % Remove m files
    dirinfo(~[dirinfo.isdir])=[];
    % Go through each subdirectory and determine all m files
    dir_files = cell(1,20); k = 1;
    dir_files{1} = ' ';
    for i = 1:length(dirinfo)
        if(strcmp(dirinfo(i).name(1),'.'))
        else
            thisdir = [str_dir,'\',dirinfo(i).name];
            subdir = dir(thisdir);
            for j=1:length(subdir)
                if(~subdir(j).isdir)
                    l_n = length(subdir(j).name);
                    dir_files{k+1} = subdir(j).name(1:l_n-2);
                    k = k+1;
                end
            end
        end
    end
    dir_files = dir_files(1:k);
    set(hObject, 'String', dir_files);
end


% --- Executes on selection change in grid_popup.
function grid_popup_Callback(hObject, eventdata, handles)
    % hObject    handle to grid_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns grid_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from grid_popup
end


% --- Executes during object creation, after setting all properties.
function grid_popup_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to grid_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
    % For the moment
%     set(hObject, 'String', {'Uniform Grid'});
end




% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
saveState(handles)
delete(hObject);
end

function saveState(handles)
    % Save all of the settings
    state.model_popup_value                 =   get(handles.model_popup,'value');
    state.cable_set_value                   =   get(handles.cableset_popup,'value');
    state.workspace_type_popup_value        =   get(handles.workspace_type_popup,'value');
    state.workspace_condition_popup_value   =   get(handles.workspace_condition_popup,'value');
    state.workspace_generation_popup_value  =   get(handles.workspace_generation_popup,'value');
    state.workspace_metric_popup_value      =   get(handles.workspace_metric_popup,'value');
    state.workspace_table                   =   handles.qtable.Data;
    path_string = fileparts(mfilename('fullpath'));
    path_string = path_string(1:strfind(path_string, 'scripts')-2);
    save([path_string,'\logs\gui_state.mat'],'state')
end

function loadState(handles)
    % load all of the settings and initialise the values to match
    path_string = fileparts(mfilename('fullpath'));
    path_string = path_string(1:strfind(path_string, 'scripts')-2);
    file_name = [path_string,'\logs\gui_state.mat'];
    if(exist(file_name,'file'))
        load(file_name);
        set(handles.model_popup,'value',state.model_popup_value);
        cableset_popup_update(handles);
        set(handles.cableset_popup,'value',state.cable_set_value);
        set(handles.workspace_type_popup,'value',state.workspace_type_popup_value);
        workspace_condition_popup_Update(handles.workspace_condition_popup,state.workspace_type_popup_value);
        set(handles.workspace_condition_popup,'value',state.workspace_condition_popup_value);
        workspace_generation_popup_Update(handles);
        set(handles.workspace_generation_popup,'value',state.workspace_generation_popup_value);
        set(handles.workspace_metric_popup,'value',state.workspace_metric_popup_value);
        handles.qtable.Data = state.workspace_table;        
    end
end
