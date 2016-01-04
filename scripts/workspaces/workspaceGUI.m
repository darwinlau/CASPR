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

    % Last Modified by GUIDE v2.5 03-Jan-2016 18:54:54

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

% --- Executes just before workspaceGUI is made visible.
function workspaceGUI_OpeningFcn(hObject, eventdata, handles, varargin)
    % This function has no output args, see OutputFcn.
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % varargin   command line arguments to workspaceGUI (see VARARGIN)

    % Choose default command line output for workspaceGUI
    handles.output = hObject;

    path_string = fileparts(mfilename('fullpath'));
    path_string = path_string(1:strfind(path_string, 'scripts')-2);
    addpath(genpath(path_string));
    
    % Update handles structure
    guidata(hObject, handles);

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


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
    % hObject    handle to popupmenu1 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from popupmenu1
end


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to popupmenu1 (see GCBO)
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


% --- Executes on selection change in popupmenu2.
function popupmenu2_Callback(hObject, eventdata, handles)
    % hObject    handle to popupmenu2 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns popupmenu2 contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from popupmenu2    
    workspace_type_index = get(handles.popupmenu2, 'Value');
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
    popupmenu3_Update(handles.popupmenu3,subfolder);
    popupmenu4_Update(handles);
end


% --- Executes during object creation, after setting all properties.
function popupmenu2_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to popupmenu2 (see GCBO)
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


% --- Executes on selection change in popupmenu3.
function popupmenu3_Callback(hObject, eventdata, handles)
    % hObject    handle to popupmenu3 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns popupmenu3 contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from popupmenu3
    popupmenu4_Update(handles);
end


function popupmenu3_Update(hObject,subfolder)
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
        set(hObject, 'String', {'No File'});
    else
        set(hObject, 'String', dir_files);
    end
end


% --- Executes during object creation, after setting all properties.
function popupmenu3_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to popupmenu3 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
%     subfolder = 'Interference';
%     popupmenu3_Update(hObject,subfolder);
end


% --- Executes on button press in Generate_Button.
function Generate_Button_Callback(hObject, eventdata, handles)
    % hObject    handle to Generate_Button (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    %% Clear data
    clc; warning off; %close all;
    %% Model setup
    contents = cellstr(get(handles.popupmenu1,'String'));
    model_type = contents{get(handles.popupmenu1,'Value')};
    model_config = ModelConfig(ModelConfigType.(['M_',model_type]));
    cable_set_id = get(handles.edit1,'String');
    bodies_xmlobj = model_config.getBodiesProperiesXmlObj();
    cableset_xmlobj = model_config.getCableSetXmlObj(cable_set_id);
    dynObj = SystemKinematicsDynamics.LoadXmlObj(bodies_xmlobj, cableset_xmlobj);
    %% Workspace Setup
    % First the condition
    contents = cellstr(get(handles.popupmenu3,'String'));
    cfh = str2func(contents{get(handles.popupmenu3,'Value')});
    contents = cellstr(get(handles.popupmenu4,'String'));
    wcondition  = cfh(contents{get(handles.popupmenu4,'Value')});
    % Then the metric
    contents = cellstr(get(handles.popupmenu5,'String'));
    mfh = str2func(contents{get(handles.popupmenu5,'Value')});
    metric = mfh();
    %% Now initialise the simulation
    disp('Start Setup Simulation');
    start_tic       =   tic;
    wsim            =   WorkspaceSimulator(dynObj,wcondition,metric);
    q_step          =   pi/18;
    n_dim           =   2;
    uGrid           =   UniformGrid(-pi*ones(n_dim,1),(pi-q_step)*ones(n_dim,1),q_step*ones(n_dim,1));
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
    wsim.plotWorkspace();
    % wsim.plotWorkspaceHigherDimension()
    time_elapsed = toc(start_tic);
    fprintf('End Plotting Simulation : %f seconds\n', time_elapsed);
end



function edit1_Callback(hObject, eventdata, handles)
    % hObject    handle to edit1 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: get(hObject,'String') returns contents of edit1 as text
    %        str2double(get(hObject,'String')) returns contents of edit1 as a double
    
    % This block is for the cable set to be written.
    % No callback will be evaluated.
end


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to edit1 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end


% --- Executes on selection change in popupmenu4.
function popupmenu4_Callback(hObject, eventdata, handles)
    % hObject    handle to popupmenu4 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns popupmenu4 contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from popupmenu4
end

function popupmenu4_Update(handles)
    contents = cellstr(get(handles.popupmenu3,'String'));
    if(strcmp(contents,'No File'))
        set(handles.popupmenu4, 'String', {'No File'});
    else
        fh = str2func([contents{get(handles.popupmenu3,'Value')},'Methods.workspace_method_list']);
        list = fh();
        set(handles.popupmenu4, 'String', list);
    end
end


% --- Executes during object creation, after setting all properties.
function popupmenu4_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to popupmenu4 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end


% --- Executes on selection change in popupmenu5.
function popupmenu5_Callback(hObject, eventdata, handles)
    % hObject    handle to popupmenu5 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns popupmenu5 contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from popupmenu5
end

% --- Executes during object creation, after setting all properties.
function popupmenu5_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to popupmenu5 (see GCBO)
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
    dir_files = cell(1,20); k = 0;
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


% --- Executes on selection change in popupmenu6.
function popupmenu6_Callback(hObject, eventdata, handles)
    % hObject    handle to popupmenu6 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns popupmenu6 contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from popupmenu6
end


% --- Executes during object creation, after setting all properties.
function popupmenu6_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to popupmenu6 (see GCBO)
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



function edit3_Callback(hObject, eventdata, handles)
    % hObject    handle to edit3 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: get(hObject,'String') returns contents of edit3 as text
    %        str2double(get(hObject,'String')) returns contents of edit3 as a double
end


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to edit3 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end


function edit4_Callback(hObject, eventdata, handles)
    % hObject    handle to edit4 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: get(hObject,'String') returns contents of edit4 as text
    %        str2double(get(hObject,'String')) returns contents of edit4 as a double
end


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to edit4 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end



function edit5_Callback(hObject, eventdata, handles)
    % hObject    handle to edit5 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: get(hObject,'String') returns contents of edit5 as text
    %        str2double(get(hObject,'String')) returns contents of edit5 as a double
end


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to edit5 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end
