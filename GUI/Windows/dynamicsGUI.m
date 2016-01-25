%--------------------------------------------------------------------------
%% Constructor
%--------------------------------------------------------------------------
function varargout = dynamicsGUI(varargin)
    % DYNAMICSGUI MATLAB code for dynamicsGUI.fig
    %      DYNAMICSGUI, by itself, creates a new DYNAMICSGUI or raises the existing
    %      singleton*.
    %
    %      H = DYNAMICSGUI returns the handle to a new DYNAMICSGUI or the handle to
    %      the existing singleton*.
    %
    %      DYNAMICSGUI('CALLBACK',hObject,eventData,handles,...) calls the local
    %      function named CALLBACK in DYNAMICSGUI.M with the given input arguments.
    %
    %      DYNAMICSGUI('Property','Value',...) creates a new DYNAMICSGUI or raises the
    %      existing singleton*.  Starting from the left, property value pairs are
    %      applied to the GUI before dynamicsGUI_OpeningFcn gets called.  An
    %      unrecognized property name or invalid value makes property application
    %      stop.  All inputs are passed to dynamicsGUI_OpeningFcn via varargin.
    %
    %      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
    %      instance to run (singleton)".
    %
    % See also: GUIDE, GUIDATA, GUIHANDLES

    % Edit the above text to modify the response to help dynamicsGUI

    % Last Modified by GUIDE v2.5 14-Jan-2016 15:58:41

    % Begin initialization code - DO NOT EDIT
    gui_Singleton = 1;
    gui_State = struct('gui_Name',       mfilename, ...
                       'gui_Singleton',  gui_Singleton, ...
                       'gui_OpeningFcn', @dynamicsGUI_OpeningFcn, ...
                       'gui_OutputFcn',  @dynamicsGUI_OutputFcn, ...
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
% --- Executes just before dynamicsGUI is made visible.
function dynamicsGUI_OpeningFcn(hObject, eventdata, handles, varargin)
    % This function has no output args, see OutputFcn.
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % varargin   command line arguments to dynamicsGUI (see VARARGIN)

    % Choose default command line output for dynamicsGUI
    handles.output = hObject;

    % Update handles structure
    guidata(hObject, handles);
    
    % Load the state
    loadState(handles);
    trajectory_popup_Update(hObject, eventdata, handles);

    % UIWAIT makes dynamicsGUI wait for user response (see UIRESUME)
    % uiwait(handles.figure1);
end

% --- Outputs from this function are returned to the command line.
function varargout = dynamicsGUI_OutputFcn(~, ~, handles)
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
%% Menu Functions
%--------------------------------------------------------------------------
% --------------------------------------------------------------------
function FileMenu_Callback(~, ~, ~) %#ok<DEFNU>
    % hObject    handle to FileMenu (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
end

% --------------------------------------------------------------------
function OpenMenuItem_Callback(~, ~, ~) %#ok<DEFNU>
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
% --- Executes on selection change in popupmenu1.
% --- Executes on selection change in trajectory_popup.
function trajectory_popup_Callback(~, ~, ~) %#ok<DEFNU>
    % hObject    handle to trajectory_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns trajectory_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from trajectory_popup
    
end

function trajectory_popup_Update(~, ~, handles)
    contents = cellstr(get(handles.model_text,'String'));
    model_type = contents{1};
    model_config = ModelConfig(ModelConfigType.(['M_',model_type]));
    setappdata(handles.trajectory_popup,'model_config',model_config);
    % Determine the cable sets
    trajectoriesObj = model_config.trajectoriesXmlObj.getElementsByTagName('trajectories').item(0).getElementsByTagName('trajectory');
    trajectories_str = cell(1,trajectoriesObj.getLength);
    % Extract the identifies from the cable sets
    for i =1:trajectoriesObj.getLength
        trajectoryObj = trajectoriesObj.item(i-1);
        trajectories_str{i} = char(trajectoryObj.getAttribute('id'));
    end
    set(handles.trajectory_popup, 'Value', 1);
    set(handles.trajectory_popup, 'String', trajectories_str);
end

% --- Executes during object creation, after setting all properties.
function trajectory_popup_CreateFcn(hObject, ~, ~) %#ok<DEFNU>
    % hObject    handle to trajectory_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
    
end

% --- Executes on selection change in dynamics_popup.
function dynamics_popup_Callback(~, ~, ~) %#ok<DEFNU>
    % hObject    handle to dynamics_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns dynamics_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from dynamics_popup
end


% --- Executes during object creation, after setting all properties.
function dynamics_popup_CreateFcn(hObject, ~, ~) %#ok<DEFNU>
    % hObject    handle to dynamics_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
    dynamics_str = {'Forward Dynamics','Inverse Dynamics'};
    set(hObject, 'String', dynamics_str);
end

% --- Executes on selection change in solver_class_popup.
function solver_class_popup_Callback(~, ~, handles) %#ok<DEFNU>
    % hObject    handle to solver_class_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns solver_class_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from solver_class_popup
    solver_type_popup_update(handles.solver_type_popup,handles);
    objective_popup_update(handles.objective_popup,handles);
    constraint_popup_update(handles.constraint_popup,handles);
end

% --- Executes during object creation, after setting all properties.
function solver_class_popup_CreateFcn(hObject, ~, ~) %#ok<DEFNU>
    % hObject    handle to solver_class_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
    path_string = fileparts(mfilename('fullpath'));
    path_string = path_string(1:strfind(path_string, 'GUI')-2);
    settingsXMLObj =  XmlOperations.XmlReadRemoveIndents([path_string,'\GUI\XML\dynamicsXML.xml']);
    setappdata(hObject,'settings',settingsXMLObj);
    solversObj = settingsXMLObj.getElementsByTagName('simulator').item(0).getElementsByTagName('solver_class');
    solver_str = cell(1,solversObj.getLength);
    % Extract the identifies from the cable sets
    for i =1:solversObj.getLength
        solverObj = solversObj.item(i-1);
        solver_str{i} = char(solverObj.getAttribute('id'));
    end
    set(hObject, 'String', solver_str);
end


% --- Executes on selection change in objective_popup.
function objective_popup_Callback(~, ~, ~) %#ok<DEFNU>
    % hObject    handle to objective_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns objective_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from objective_popup
end

function objective_popup_update(hObject,handles)
    contents = cellstr(get(handles.solver_class_popup,'String'));
    solver_class_id = contents{get(handles.solver_class_popup,'Value')};
    settings = getappdata(handles.solver_class_popup,'settings');
    solverObj = settings.getElementById(solver_class_id);
    objectivesUnfiltered = solverObj.getElementsByTagName('objectives').item(0);
    if(isempty(objectivesUnfiltered))
        set(hObject,'Value',1);
        set(hObject,'String',{' '});
    else
        objectivesObj = objectivesUnfiltered.getElementsByTagName('objective');
        objective_str = cell(1,objectivesObj.getLength);
        % Extract the identifies from the cable sets
        for i =1:objectivesObj.getLength
            objectiveObj = objectivesObj.item(i-1);
            objective_str{i} = char(objectiveObj.getFirstChild.getData);
        end
        set(hObject,'Value',1);
        set(hObject, 'String', objective_str);
    end
end

% --- Executes during object creation, after setting all properties.
function objective_popup_CreateFcn(hObject, ~, ~) %#ok<DEFNU>
    % hObject    handle to objective_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end

% --- Executes on selection change in solver_type_popup.
function solver_type_popup_Callback(~, ~, ~) %#ok<DEFNU>
    % hObject    handle to solver_type_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns solver_type_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from solver_type_popup
end

function solver_type_popup_update(hObject,handles)
    contents = cellstr(get(handles.solver_class_popup,'String'));
    solver_class_id = contents{get(handles.solver_class_popup,'Value')};
    settings = getappdata(handles.solver_class_popup,'settings');
    solverObj = settings.getElementById(solver_class_id);
    enum_file = solverObj.getElementsByTagName('solver_type_enum').item(0).getFirstChild.getData;
    e_list = enumeration(char(enum_file));
    e_n         =   length(e_list);
    e_list_str  =   cell(1,e_n);
    for i=1:e_n
        temp_str = char(e_list(i));
        e_list_str{i} = temp_str(1:length(temp_str));
    end
    set(hObject,'Value',1);
    set(hObject, 'String', e_list_str);
end

% --- Executes during object creation, after setting all properties.
function solver_type_popup_CreateFcn(hObject, ~, ~) %#ok<DEFNU>
    % hObject    handle to solver_type_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end

% --- Executes on selection change in plot_type_popup.
function plot_type_popup_Callback(~, ~, ~) %#ok<DEFNU>
    % hObject    handle to plot_type_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns plot_type_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from plot_type_popup
    
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
    path_string = fileparts(mfilename('fullpath'));
    path_string = path_string(1:strfind(path_string, 'GUI')-2);
    settingsXMLObj =  XmlOperations.XmlReadRemoveIndents([path_string,'\GUI\XML\dynamicsXML.xml']);
    plotsObj = settingsXMLObj.getElementsByTagName('simulator').item(0).getElementsByTagName('plot_functions').item(0).getElementsByTagName('plot_type');
    plot_str = cell(1,plotsObj.getLength);
    % Extract the identifies from the cable sets
    for i =1:plotsObj.getLength
        plotObj = plotsObj.item(i-1);
        plot_str{i} = char(plotObj.getFirstChild.getData);
    end
    set(hObject,'Value',1);
    set(hObject, 'String', plot_str);
end

% --- Executes on selection change in constraint_popup.
function constraint_popup_Callback(~, ~, ~) %#ok<DEFNU>
    % hObject    handle to constraint_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns constraint_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from constraint_popup
end

function constraint_popup_update(hObject,handles)
    contents = cellstr(get(handles.solver_class_popup,'String'));
    solver_class_id = contents{get(handles.solver_class_popup,'Value')};
    settings = getappdata(handles.solver_class_popup,'settings');
    solverObj = settings.getElementById(solver_class_id);
    constraintsUnfiltered = solverObj.getElementsByTagName('constraints').item(0);
    if(isempty(constraintsUnfiltered))
        set(hObject,'Value',1);
        set(hObject,'String',{' '});
    else
        constraintsObj = solverObj.getElementsByTagName('constraints').item(0).getElementsByTagName('constraint');
        constraint_str = cell(1,constraintsObj.getLength+1);
        % Extract the identifies from the cable sets
        constraint_str{1} = ' ';
        for i =1:constraintsObj.getLength
            constraintObj = constraintsObj.item(i-1);
            constraint_str{i+1} = char(constraintObj.getFirstChild.getData);
        end
        set(hObject,'Value',1);
        set(hObject, 'String', constraint_str);
    end
end


% --- Executes during object creation, after setting all properties.
function constraint_popup_CreateFcn(hObject, ~, ~) %#ok<DEFNU>
    % hObject    handle to constraint_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end 


%--------------------------------------------------------------------------
%% Push Buttons
%--------------------------------------------------------------------------
% --- Executes on button press in run_button.
function run_button_Callback(~, ~, handles) %#ok<DEFNU>
    % hObject    handle to run_button (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % First read the trajecory information
    clc; 
    contents = cellstr(get(handles.trajectory_popup,'String'));
    trajectory_id = contents{get(handles.trajectory_popup,'Value')};
    model_config = getappdata(handles.trajectory_popup,'model_config');
    trajectory_xmlobj = model_config.getTrajectoryXmlObj(trajectory_id);
    % Then read the form of dynamics
    contents = cellstr(get(handles.dynamics_popup,'String'));
    dynamics_id = contents{get(handles.dynamics_popup,'Value')};
    dynObj = getappdata(handles.cable_text,'dynObj');
    if(strcmp(dynamics_id,'Inverse Dynamics'))
        run_inverse_dynamics(handles,dynObj,trajectory_xmlobj);
    else
        run_forward_dynamics(handles,dynObj,trajectory_xmlobj);
    end
end

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
    load(file_name);
    mp_text = get(handles.model_text,'String');
    cs_text = get(handles.cable_text,'String');
    if(strcmp(mp_text,state.model_text)&&strcmp(cs_text,state.cable_text))
        set(handles.trajectory_popup,'value',state.trajectory_popup_vale);
        set(handles.dynamics_popup,'value',state.dynamics_popup_vale);
        set(handles.solver_class_popup,'value',state.solver_class_popup);
        solver_type_popup_update(handles.solver_type_popup,handles);
        objective_popup_update(handles.objective_popup,handles);
        constraint_popup_update(handles.constraint_popup,handles);
        set(handles.solver_type_popup,'value',state.solver_type_popup);
        set(handles.objective_popup,'value',state.objective_popup);
        set(handles.constraint_popup,'value',state.constraint_popup);
        set(handles.plot_type_popup,'value',state.plot_type_popup);
        set(handles.radiobutton1,'value',state.r_value);
        set(handles.QTable,'Data',state.weight_table);
    else
        warning('Incorrect Model Type');
    end
end


% --- Executes on button press in plot_button.
function plot_button_Callback(~, ~, handles) %#ok<DEFNU>
    % hObject    handle to plot_button (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)  
    sim = getappdata(handles.figure1,'sim');
    if(isempty(sim))
        warning('No simulator has been generated. Please press run first');
    else
        contents = cellstr(get(handles.plot_type_popup,'String'));
        plot_type = contents{get(handles.plot_type_popup,'Value')};
        plot_function = str2func(plot_type);
        figure_handles = handles.figure1;
        plot_function(sim,[],figure_handles);    
    end
end


%--------------------------------------------------------------------------
%% Radio Buttons
%--------------------------------------------------------------------------
% --- Executes on button press in radiobutton1.
function radiobutton1_Callback(hObject, ~, handles) %#ok<DEFNU>
    % hObject    handle to radiobutton1 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hint: get(hObject,'Value') returns toggle state of radiobutton1
    r_value = get(hObject,'Value');
    Q_editable = get(handles.QTable,'ColumnEditable');
    Q_n = length(Q_editable);
    if(r_value)
        set(handles.QTable,'Data',ones(1,Q_n));
        set(handles.QTable,'ColumnEditable',false(1,Q_n));
    else
        
        set(handles.QTable,'ColumnEditable',true(1,Q_n));
    end
end

%--------------------------------------------------------------------------
% Additional Functions
%--------------------------------------------------------------------------
function run_inverse_dynamics(handles,dynObj,trajectory_xmlobj)
    % First read the solver form from the GUI
    contents = cellstr(get(handles.solver_class_popup,'String'));
    solver_class = contents{get(handles.solver_class_popup,'Value')};
    contents = cellstr(get(handles.solver_type_popup,'String'));
    solver_type = contents{get(handles.solver_type_popup,'Value')};
    contents = cellstr(get(handles.objective_popup,'String'));
    objective = contents{get(handles.objective_popup,'Value')};
    settings = getappdata(handles.solver_class_popup,'settings');
    if(~strcmp(objective,' '))
        objective_function = str2func(objective);
        q_data = get(handles.QTable,'Data');
        id_objective = objective_function(q_data');
        solver_function = str2func(solver_class);
        solverObj = settings.getElementById(solver_class);
        enum_file = solverObj.getElementsByTagName('solver_type_enum').item(0).getFirstChild.getData;
        id_solver = solver_function(id_objective,eval([char(enum_file),'.',char(solver_type)]));
    else
        solver_function = str2func(solver_class);
        solverObj = settings.getElementById(solver_class);
        enum_file = solverObj.getElementsByTagName('solver_type_enum').item(0).getFirstChild.getData;
        id_solver = solver_function(eval([char(enum_file),'.',char(solver_type)]));
    end
    contents = cellstr(get(handles.plot_type_popup,'String'));
    plot_type = contents{get(handles.plot_type_popup,'Value')};
    
    
    % Setup the inverse dynamics simulator with the SystemKinematicsDynamics
    % object and the inverse dynamics solver
    disp('Start Setup Simulation');
    start_tic = tic;
    idsim = InverseDynamicsSimulator(dynObj, id_solver);
    trajectory = JointTrajectory.LoadXmlObj(trajectory_xmlobj, dynObj);
    time_elapsed = toc(start_tic);
    fprintf('End Setup Simulation : %f seconds\n', time_elapsed);

    % Run the solver on the desired trajectory
    disp('Start Running Simulation');
    start_tic = tic;
    idsim.run(trajectory);
    time_elapsed = toc(start_tic);
    fprintf('End Running Simulation : %f seconds\n', time_elapsed);

    % Display information from the inverse dynamics simulator
    fprintf('Optimisation computational time, mean : %f seconds, std dev : %f seconds, total: %f seconds\n', mean(idsim.compTime), std(idsim.compTime), sum(idsim.compTime));

    % After running the simulator the data can be plotted
    % Refer to the simulator classes to see what can be plotted.

    % The neck model has many cables/muscles, so it is possible to plot a
    % subset of them only

    % % Right muscles
    % idsim.PlotCableForces(1:38); xlabel('time [s]'); ylabel('cable forces [N]'); title('Forces (right)');
    % % Left muscles
    % idsim.PlotCableForces(39:76); xlabel('time [s]'); ylabel('cable forces [N]'); title('Forces (left)');

    % Otherwise here is some simple example
    disp('Start Plotting Simulation');
    start_tic = tic;
    plot_function = str2func(plot_type);
    figure_handles = handles.figure1;
    plot_function(idsim,[],figure_handles);    
    % Store the simutor
    setappdata(handles.figure1,'sim',idsim);
    time_elapsed = toc(start_tic);
    fprintf('End Plotting Simulation : %f seconds\n', time_elapsed);
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
        file_name = [path_string,'\logs\dynamics_gui_state.mat'];
        format_Q_table(state.dynObj.numCables,handles.QTable);
        if(exist(file_name,'file'))
            load(file_name);
            mp_text = get(handles.model_text,'String');
            cs_text = get(handles.cable_text,'String');
            if(strcmp(mp_text,state.model_text)&&strcmp(cs_text,state.cable_text))
                set(handles.trajectory_popup,'value',state.trajectory_popup_vale);
                set(handles.dynamics_popup,'value',state.dynamics_popup_vale);
                set(handles.solver_class_popup,'value',state.solver_class_popup);
                solver_type_popup_update(handles.solver_type_popup,handles);
                objective_popup_update(handles.objective_popup,handles);
                constraint_popup_update(handles.constraint_popup,handles);
                set(handles.solver_type_popup,'value',state.solver_type_popup);
                set(handles.objective_popup,'value',state.objective_popup);
                set(handles.constraint_popup,'value',state.constraint_popup);
                set(handles.plot_type_popup,'value',state.plot_type_popup);
                set(handles.radiobutton1,'value',state.r_value);
                set(handles.QTable,'Data',state.weight_table);
            end
        end
    end
end

function saveState(handles,file_path)
    % Save all of the settings
    state.model_text                        =   get(handles.model_text,'String');
    state.cable_text                        =   get(handles.cable_text,'String');
    state.trajectory_popup_vale             =   get(handles.trajectory_popup,'value');
    state.dynamics_popup_vale               =   get(handles.dynamics_popup,'value');
    state.solver_class_popup                =   get(handles.solver_class_popup,'value');
    state.solver_type_popup                 =   get(handles.solver_type_popup,'value');
    state.objective_popup                   =   get(handles.objective_popup,'value');
    state.constraint_popup                  =   get(handles.constraint_popup,'value');
    state.plot_type_popup                   =   get(handles.plot_type_popup,'value');
    state.r_value                           =   get(handles.radiobutton1,'value');
    state.weight_table                      =   get(handles.QTable,'Data');
    if(nargin>1)
        save(file_path,'state');
    else
        path_string                             =   fileparts(mfilename('fullpath'));
        path_string                             = path_string(1:strfind(path_string, 'GUI')-2);
        save([path_string,'\logs\dynamics_gui_state.mat'],'state')
    end
end

function run_forward_dynamics(handles,dynObj,trajectory_xmlobj)
    % This will be added once script_FD has been fixed
end

function format_Q_table(numCables,QTable)
    set(QTable,'Data',ones(1,numCables));
    set(QTable,'ColumnWidth',{30});
    set(QTable,'ColumnEditable',true(1,numCables));
    q_position = get(QTable,'Position');
    if(numCables>4)
        q_position(2) = 23;
        q_position(4) = 57;
        set(QTable,'Position',q_position);
    else
        q_position(2) = 40;
        q_position(4) = 40;
        set(QTable,'Position',q_position);
    end
end

%% TO BE DONE
% Add more plotting functions
% Integrate with forward dynamics
% Determine where best to store settings
