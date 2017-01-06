% The GUI window for performing workspace analysis
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :

%--------------------------------------------------------------------------
%% Constructor
%--------------------------------------------------------------------------
function varargout = workspace_GUI(varargin)
    % WORKSPACE_GUI MATLAB code for workspace_GUI.fig
    %      WORKSPACE_GUI, by itself, creates a new WORKSPACE_GUI or raises the existing
    %      singleton*.
    %
    %      H = WORKSPACE_GUI returns the handle to a new WORKSPACE_GUI or the handle to
    %      the existing singleton*.
    %
    %      WORKSPACE_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
    %      function named CALLBACK in WORKSPACE_GUI.M with the given input arguments.
    %
    %      WORKSPACE_GUI('Property','Value',...) creates a new WORKSPACE_GUI or raises the
    %      existing singleton*.  Starting from the left, property value pairs are
    %      applied to the GUI before workspace_GUI_OpeningFcn gets called.  An
    %      unrecognized property name or invalid value makes property application
    %      stop.  All inputs are passed to workspace_GUI_OpeningFcn via varargin.
    %
    %      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
    %      instance to run (singleton)".
    %
    % See also: GUIDE, GUIDATA, GUIHANDLES

    % Edit the above text to modify the response to help workspace_GUI

    % Last Modified by GUIDE v2.5 24-Feb-2016 19:07:40

    % Begin initialization code - DO NOT EDIT
    gui_Singleton = 1;
    gui_State = struct('gui_Name',       mfilename, ...
                       'gui_Singleton',  gui_Singleton, ...
                       'gui_OpeningFcn', @workspace_GUI_OpeningFcn, ...
                       'gui_OutputFcn',  @workspace_GUI_OutputFcn, ...
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
% --- Executes just before workspace_GUI is made visible.
function workspace_GUI_OpeningFcn(hObject, ~, handles, varargin)
    % This function has no output args, see OutputFcn.
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % varargin   command line arguments to workspace_GUI (see VARARGIN)

    % Choose default command line output for workspace_GUI    
    handles.output = hObject;
    
    % Update handles structure
    guidata(hObject, handles);
    loadState(handles);
    GUIOperations.CreateTabGroup(handles);
    % UIWAIT makes workspace_GUI wait for user response (see UIRESUME)
    % uiwait(handles.figure1);
end

% --- Outputs from this function are returned to the command line.
function varargout = workspace_GUI_OutputFcn(~, ~, handles) 
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
    settingsXMLObj =  GUIOperations.GetSettings('/GUI/XML/workspaceXML.xml');
    setappdata(hObject,'settings',settingsXMLObj);
    workspace_str = GUIOperations.XmlObj2StringCellArray(settingsXMLObj.getElementsByTagName('simulator').item(0).getElementsByTagName('workspace_condition')...
                    ,'id');
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
    e_list      =   enumeration(char(enum_file));
    e_n         =   length(e_list);
    e_list_str  =   cell(1,e_n);
    for i=1:e_n
        temp_str = char(e_list(i));
        e_list_str{i} = temp_str(3:length(temp_str));
    end
    set(hObject,'Value',1);
    set(hObject, 'String', e_list_str);
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
    settingsXMLObj = GUIOperations.GetSettings('/GUI/XML/workspaceXML.xml');
    workspace_str = GUIOperations.XmlObj2StringCellArray(settingsXMLObj.getElementsByTagName('simulator').item(0).getElementsByTagName('workspace_metrics').item(0).getElementsByTagName('workspace_metric')...
        ,[]);
    workspace_str = [{' '},workspace_str];
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
    settingsXMLObj = GUIOperations.GetSettings('/GUI/XML/workspaceXML.xml');
    workspace_str = GUIOperations.XmlObj2StringCellArray(settingsXMLObj.getElementsByTagName('simulator').item(0).getElementsByTagName('grid_types').item(0).getElementsByTagName('grid_type')...
        ,[]);
    set(hObject, 'String', workspace_str);
end

% --- Executes on selection change in plot_type_popup.
function plot_type_popup_Callback(hObject, ~, handles) %#ok<DEFNU>
    % hObject    handle to plot_type_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns plot_type_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from plot_type_popup
    settingsXMLObj = getappdata(handles.workspace_condition_popup,'settings');
    plotsObj = settingsXMLObj.getElementsByTagName('simulator').item(0).getElementsByTagName('plot_functions').item(0).getElementsByTagName('plot_function');
    contents = get(hObject,'Value');
    plotObj = plotsObj.item(contents-1);
    setappdata(hObject,'num_plots',plotObj.getElementsByTagName('figure_quantity').item(0).getFirstChild.getData);
end

% --- Executes during object creation, after setting all properties.
function plot_type_popup_CreateFcn(hObject, ~, ~) %#ok<DEFNU>
    % hObject    handle to plot_type_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
    settingsXMLObj = GUIOperations.GetSettings('/GUI/XML/workspaceXML.xml');
    plotsObj = settingsXMLObj.getElementsByTagName('simulator').item(0).getElementsByTagName('plot_functions').item(0).getElementsByTagName('plot_function');
    plot_str = cell(1,plotsObj.getLength);
    % Extract the identifies from the cable sets
    for i =1:plotsObj.getLength
        plotObj = plotsObj.item(i-1);
        plot_str{i} = char(plotObj.getAttribute('type'));
    end
    set(hObject,'Value',1);
    set(hObject, 'String', plot_str);
    setappdata(hObject,'num_plots',1);
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
    file_name = [path_string,'/GUI/config/*.mat'];
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
    file_name = [path_string,'/GUI/config/*.mat'];
    settings = uigetfile(file_name);
    load(settings)
    mp_text = get(handles.model_text,'String');
    cs_text = get(handles.cable_text,'String');
    if(strcmp(state.simulator,'workspace'))
        if(strcmp(mp_text,state.model_text)&&strcmp(cs_text,state.cable_text))
            set(handles.workspace_condition_popup,'value',state.workspace_condition_popup_value);
            workspace_generation_popup_Update(handles.workspace_generation_popup,handles);
            set(handles.workspace_generation_popup,'value',state.workspace_generation_popup_value);
            set(handles.workspace_metric_popup,'value',state.workspace_metric_popup_value);
            set(handles.qtable,'Data',state.workspace_table);
        else
            warning('Incorrect Model Type');
        end
    else
        warning('File is not the correct file type'); %#ok<WNTAG>
    end
end

% --- Executes on button press in generate_button.
function generate_button_Callback(~, ~, handles) %#ok<DEFNU>
    % hObject    handle to generate_button (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    %% Clear data
    clc; warning off; %#ok<WNOFF> %close all;
    %% Model setup
    modObj = getappdata(handles.cable_text,'modObj');
    %% Workspace Setup
    % First the condition
    contents = cellstr(get(handles.workspace_condition_popup,'String'));
    wc_string = contents{get(handles.workspace_condition_popup,'Value')};
    contents = cellstr(get(handles.workspace_generation_popup,'String'));
    wcm_string = contents{get(handles.workspace_generation_popup,'Value')};    
    settings = getappdata(handles.workspace_condition_popup,'settings');
    workspaceObj = settings.getElementById(wc_string);
    enum_file = workspaceObj.getElementsByTagName('generation_method_enum').item(0).getFirstChild.getData;
    wcondition = {WorkspaceConditionBase.CreateWorkspaceCondition(eval(['WorkspaceConditionType.',wc_string]),eval([char(enum_file),'.M_',wcm_string]),[])};
%     wcondition  = cfh(contents{get(handles.workspace_generation_popup,'Value')},[-1,1,0,0;0,0,-1,1]);
    % Then the metric
    contents = cellstr(get(handles.workspace_metric_popup,'String'));
    if(strcmp(contents{get(handles.workspace_metric_popup,'Value')},' '))
        metric = {};
    else
        wmetric = contents{get(handles.workspace_metric_popup,'Value')};
        metric = {WorkspaceMetricBase.CreateWorkspaceMetric(eval(['WorkspaceMetricType.',wmetric]),[])};
    end
    %% Now initialise the simulation
    disp('Start Setup Simulation');
    set(handles.status_text,'String','Setting up simulation');
    drawnow;
    start_tic       =   tic;
    q_info = get(handles.qtable,'Data');
    uGrid           =   UniformGrid(q_info(:,1),q_info(:,2),q_info(:,3));
    contents = cellstr(get(handles.plot_type_popup,'String'));
    plot_type = contents{get(handles.plot_type_popup,'Value')};
    opt = WorkspaceSimulatorOptions(true); % This should be made into an object
    wsim            =   WorkspaceSimulator(modObj,uGrid,opt);
    %% Now set up the grid information
    time_elapsed    =   toc(start_tic);
    fprintf('End Setup Simulation : %f seconds\n', time_elapsed);
    
    disp('Start Running Simulation');
    set(handles.status_text,'String','Simulation running');
    drawnow;
    start_tic       =   tic;
    wsim.run(wcondition,metric); 
    time_elapsed    =   toc(start_tic);
    fprintf('End Running Simulation : %f seconds\n', time_elapsed);
    
    disp('Start Plotting Simulation');
%     set(handles.status_text,'String','Simulation plotting');
%     drawnow;
%     start_tic = tic;
%     GUIOperations.GUIPlot(plot_type,wsim,handles,str2double(getappdata(handles.plot_type_popup,'num_plots')),get(handles.undock_box,'Value'));
%     time_elapsed = toc(start_tic);
    disp('Workspace plotting is not currently supported in the GUI.  Please use the plotting functions for the Simulator Classes')
    fprintf('End Plotting Simulation : %f seconds\n', time_elapsed);
    set(handles.status_text,'String','No simulation running');
    setappdata(handles.figure1,'sim',wsim);
    assignin('base','wsim',wsim);
end

% --- Executes on button press in plot_button.
function plot_button_Callback(~, ~, handles) %#ok<DEFNU>
    % hObject    handle to plot_button (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    sim = getappdata(handles.figure1,'sim');
    if(isempty(sim))
        warning('No simulator has been generated. Please press run first'); %#ok<WNTAG>
    else
        contents = cellstr(get(handles.plot_type_popup,'String'));
        plot_type = contents{get(handles.plot_type_popup,'Value')};
        GUIOperations.GUIPlot(plot_type,sim,handles,str2double(getappdata(handles.plot_type_popup,'num_plots')),get(handles.undock_box,'Value'))
    end
end

%--------------------------------------------------------------------------
%% Toolbar buttons
%--------------------------------------------------------------------------
% --------------------------------------------------------------------
function save_figure_tool_ClickedCallback(~, ~, handles) %#ok<DEFNU>
    % hObject    handle to save_figure_tool (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    tabgp = getappdata(handles.figure1,'tabgp');
    s_tab = get(tabgp,'SelectedTab');
    ax = get(s_tab,'Children');
    f1 = figure; % Open a new figure with handle f1
    copyobj(ax,f1); % Copy axes object h into figure f1
    set(gca,'ActivePositionProperty','outerposition')
    set(gca,'Units','normalized')
    set(gca,'OuterPosition',[0 0 1 1])
    set(gca,'position',[0.1300 0.1100 0.7750 0.8150])
    [file,path] = uiputfile({'*.fig';'*.bmp';'*.eps';'*.emf';'*.jpg';'*.pcx';...
        '*.pbm';'*.pdf';'*.pgm';'*.png';'*.ppm';'*.svg';'*.tif'},'Save file name');
    if(path ~= 0)
        saveas(gcf,[path,file]);
    end
    close(f1);
end

% --------------------------------------------------------------------
function undock_figure_tool_ClickedCallback(~, ~, handles) %#ok<DEFNU>
    % hObject    handle to undock_figure_tool (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    tabgp = getappdata(handles.figure1,'tabgp');
    s_tab = get(tabgp,'SelectedTab');
    ax = get(s_tab,'Children');
    f1 = figure; % Open a new figure with handle f1
    copyobj(ax,f1); % Copy axes object h into figure f1
    set(gca,'ActivePositionProperty','outerposition')
    set(gca,'Units','normalized')
    set(gca,'OuterPosition',[0 0 1 1])
    set(gca,'position',[0.1300 0.1100 0.7750 0.8150])
end   


% --------------------------------------------------------------------
function delete_figure_tool_ClickedCallback(~, ~, handles) %#ok<DEFNU>
    % hObject    handle to delete_figure_tool (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    tabgp = getappdata(handles.figure1,'tabgp');
    s_tab = get(tabgp,'SelectedTab');
    if(strcmp('0',get(s_tab,'Title')))
        % Do nothing
    else
        delete(s_tab);
    end
end

%--------------------------------------------------------------------------
%% Toggle Buttons
%--------------------------------------------------------------------------
% --- Executes on button press in undock_box.
function undock_box_Callback(~, ~, ~) %#ok<DEFNU>
    % hObject    handle to undock_box (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hint: get(hObject,'Value') returns toggle state of undock_box
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
    state.simulator                         =   'workspace';
    state.model_text                        =   get(handles.model_text,'String');
    state.cable_text                        =   get(handles.cable_text,'String');
    state.workspace_condition_popup_value   =   get(handles.workspace_condition_popup,'value');
    state.workspace_generation_popup_value  =   get(handles.workspace_generation_popup,'value');
    state.workspace_metric_popup_value      =   get(handles.workspace_metric_popup,'value');
    state.workspace_table                   =   get(handles.qtable,'Data');
    state.plot_type_popup                   =   get(handles.plot_type_popup,'value');
    if(nargin>1)
        save(file_path,'state');
    else
        path_string                             =   fileparts(mfilename('fullpath'));
        path_string                             =   path_string(1:strfind(path_string, 'GUI')-2);
        save([path_string,'/GUI/config/workspace_gui_state.mat'],'state')
    end
end

function loadState(handles)
    % load all of the settings and initialise the values to match
    path_string = fileparts(mfilename('fullpath'));
    path_string = path_string(1:strfind(path_string, 'GUI')-2);
    file_name = [path_string,'/GUI/config/caspr_gui_state.mat'];
    if(exist(file_name,'file'))
        load(file_name)
        set(handles.model_text,'String',state.model_text);
        set(handles.cable_text,'String',state.cable_text);
        % This is to ensure that we are starting fresh
        state.modObj.bodyModel.occupied.reset();
        setappdata(handles.cable_text,'modObj',state.modObj);
        file_name = [path_string,'/GUI/config/workspace_gui_state.mat'];
        format_q_table(state.modObj.numDofs,handles.qtable)
        if(exist(file_name,'file'))
            load(file_name)
            mp_text = get(handles.model_text,'String');
            cs_text = get(handles.cable_text,'String');
            if(strcmp(mp_text,state.model_text)&&strcmp(cs_text,state.cable_text))
                set(handles.workspace_condition_popup,'value',state.workspace_condition_popup_value);
                workspace_generation_popup_Update(handles.workspace_generation_popup,handles);
                set(handles.workspace_generation_popup,'value',state.workspace_generation_popup_value);
                set(handles.workspace_metric_popup,'value',state.workspace_metric_popup_value);
                set(handles.qtable,'Data',state.workspace_table);
                set(handles.plot_type_popup,'value',state.plot_type_popup);
                plot_type_popup_Callback(handles.plot_type_popup,[],handles);
            else
                workspace_generation_popup_Update(handles.workspace_generation_popup,handles);
                plot_type_popup_Callback(handles.plot_type_popup,[],handles);
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
    q_position(1) = 0.713;
    q_position(3) = 0.235;
    set(qtable,'Position',q_position);
end

%% TO BE DONE
% Generate Workspace Plotting Functions
% Add new Methods for computing Wrench Closure
% Determine where best to store settings
