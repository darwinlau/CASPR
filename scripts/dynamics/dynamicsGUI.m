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

    % Last Modified by GUIDE v2.5 11-Jan-2016 15:59:14

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

    % This sets up the initial plot - only do when we are invisible
    % so window can get raised using dynamicsGUI.
    if strcmp(get(hObject,'Visible'),'off')
        plot(rand(5));
    end
    loadState(handles);
    trajectory_popup_Update(hObject, eventdata, handles);

    % UIWAIT makes dynamicsGUI wait for user response (see UIRESUME)
    % uiwait(handles.figure1);
end

% --- Outputs from this function are returned to the command line.
function varargout = dynamicsGUI_OutputFcn(hObject, eventdata, handles)
    % varargout  cell array for returning output args (see VARARGOUT);
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Get default command line output from handles structure
    varargout{1} = handles.output;
end

% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
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
function FileMenu_Callback(hObject, eventdata, handles)
    % hObject    handle to FileMenu (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
end

% --------------------------------------------------------------------
function OpenMenuItem_Callback(hObject, eventdata, handles)
    % hObject    handle to OpenMenuItem (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    file = uigetfile('*.fig');
    if ~isequal(file, 0)
        open(file);
    end
end

% --------------------------------------------------------------------
function PrintMenuItem_Callback(hObject, eventdata, handles)
    % hObject    handle to PrintMenuItem (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    printdlg(handles.figure1)
end

% --------------------------------------------------------------------
function CloseMenuItem_Callback(hObject, eventdata, handles)
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
function trajectory_popup_Callback(hObject, eventdata, handles)
    % hObject    handle to trajectory_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns trajectory_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from trajectory_popup
    
end

function trajectory_popup_Update(hObject, eventdata, handles)
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
function trajectory_popup_CreateFcn(hObject, eventdata, handles)
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
function dynamics_popup_Callback(hObject, eventdata, handles)
    % hObject    handle to dynamics_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns dynamics_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from dynamics_popup
end


% --- Executes during object creation, after setting all properties.
function dynamics_popup_CreateFcn(hObject, eventdata, handles)
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
function solver_class_popup_Callback(hObject, eventdata, handles)
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
function solver_class_popup_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to solver_class_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
    path_string = fileparts(mfilename('fullpath'));
    path_string = path_string(1:strfind(path_string, 'scripts')-2);
    dir_files = dir([path_string,'\src\Analysis\InverseDynamics\Solvers']);
    dir_files_temp = cell(length(dir_files)-3,1); j =0;
    for i=1:length(dir_files)
        if((~dir_files(i).isdir)&&(~strcmp(dir_files(i).name,'IDSolverExitType.m')))
            l_n = length(dir_files(i).name);
            dir_files_temp{j+1} = char(dir_files(i).name(9:l_n-2));
            j = j+1;
        end
    end
    dir_files = dir_files_temp(1:j);
    set(hObject, 'String', dir_files);
    set(hObject, 'Value', 1);
end


% --- Executes on selection change in objective_popup.
function objective_popup_Callback(hObject, eventdata, handles)
    % hObject    handle to objective_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns objective_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from objective_popup
end

function objective_popup_update(hObject,handles)
    if(nargin == 1)
        subfolder = 'Linear';
    else
        contents = cellstr(get(handles.solver_class_popup,'String'));
        solver_class = contents{get(handles.solver_class_popup,'Value')};
        if(strcmp(solver_class,'LinProg'))
            subfolder = 'Linear';
        else
            subfolder = 'Quadratic';
        end
    end
    path_string = fileparts(mfilename('fullpath'));
    path_string = path_string(1:strfind(path_string, 'scripts')-2);
    dir_files = dir([path_string,'\src\Analysis\InverseDynamics\Formulation\Objectives\',subfolder]);
    dir_files_temp = cell(length(dir_files)-2,1); j =0;
    for i=1:length(dir_files)
        if((~dir_files(i).isdir))
            l_n = length(dir_files(i).name);
            dir_files_temp{j+1} = char(dir_files(i).name(12:l_n-2));
            j = j+1;
        end
    end
    dir_files = dir_files_temp(1:j);
    set(hObject, 'Value', 1);
    set(hObject, 'String', dir_files);
end

% --- Executes during object creation, after setting all properties.
function objective_popup_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to objective_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
    objective_popup_update(hObject);
end

% --- Executes on selection change in solver_type_popup.
function solver_type_popup_Callback(hObject, eventdata, handles)
    % hObject    handle to solver_type_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns solver_type_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from solver_type_popup
end

function solver_type_popup_update(hObject,handles)
    if(nargin == 1)
        e_list      =   enumeration('ID_LP_SolverType');
    else
        contents = cellstr(get(handles.solver_class_popup,'String'));
        solver_class = contents{get(handles.solver_class_popup,'Value')};
        if(strcmp(solver_class,'LinProg'))
            e_list      =   enumeration('ID_LP_SolverType');
        else
            e_list      =   enumeration('ID_QP_SolverType');
        end
    end
    e_n         =   length(e_list);
    e_list_str  =   cell(1,e_n);
    for i=1:e_n
        temp_str = char(e_list(i));
        e_list_str{i} = temp_str(1:length(temp_str));
    end
    set(hObject, 'String', e_list_str);
end

% --- Executes during object creation, after setting all properties.
function solver_type_popup_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to solver_type_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
    solver_type_popup_update(hObject)
end

% --- Executes on selection change in plot_type_popup.
function plot_type_popup_Callback(hObject, eventdata, handles)
    % hObject    handle to plot_type_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns plot_type_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from plot_type_popup
end


% --- Executes during object creation, after setting all properties.
function plot_type_popup_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to plot_type_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
    % AT A LATER TIME THIS WILL BE CHANGED TO INCLUDE OTHER PLOTS
    m = methods('InverseDynamicsSimulator');
    plot_list = cell(length(m),1);j = 0;
    for i =1:length(m)
        if(~isempty(strfind(m{i},'plot')))
            plot_list{j+1} = m{i};
            j = j+1;
        end
    end
    plot_list = plot_list(1:j);
    set(hObject, 'String', plot_list);
end

% --- Executes on selection change in constraint_popup.
function constraint_popup_Callback(hObject, eventdata, handles)
    % hObject    handle to constraint_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns constraint_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from constraint_popup
end

function constraint_popup_update(hObject,handles)
    % This will be modified later to be more intelligent
    subfolder = 'Linear';
    path_string = fileparts(mfilename('fullpath'));
    path_string = path_string(1:strfind(path_string, 'scripts')-2);
    dir_files = dir([path_string,'\src\Analysis\InverseDynamics\Formulation\Constraints\',subfolder]);
    dir_files_temp = cell(length(dir_files)-3,1); j =1;
    dir_files_temp(1) = {' '};
    for i=1:length(dir_files)
        if((~dir_files(i).isdir))
            l_n = length(dir_files(i).name);
            dir_files_temp{j+1} = char(dir_files(i).name(13:l_n-2));
            j = j+1;
        end
    end
    dir_files = dir_files_temp(1:j);
    set(hObject, 'Value', 1);
    set(hObject, 'String', dir_files);
end


% --- Executes during object creation, after setting all properties.
function constraint_popup_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to constraint_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
    constraint_popup_update(hObject)
end 


%--------------------------------------------------------------------------
%% Push Buttons
%--------------------------------------------------------------------------
% --- Executes on button press in run_button.
function run_button_Callback(hObject, eventdata, handles)
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
function save_button_Callback(hObject, eventdata, handles)
    % hObject    handle to save_button (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    path_string = fileparts(mfilename('fullpath'));
    path_string = path_string(1:strfind(path_string, 'scripts')-2);
    file_name = [path_string,'\logs\*.mat'];
    [file,path] = uiputfile(file_name,'Save file name');
    saveState(handles,[path,file]);
end

% --- Executes on button press in load_button.
function load_button_Callback(hObject, eventdata, handles)
    % hObject    handle to load_button (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    path_string = fileparts(mfilename('fullpath'));
    path_string = path_string(1:strfind(path_string, 'scripts')-2);
    file_name = [path_string,'\logs\*.mat'];
    load(settings)
    mp_text = get(handles.model_text,'String');
    cs_text = get(handles.cable_text,'String');
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
    else
        warning('Incorrect Model Type')
    end
end


% --- Executes on button press in plot_button.
function plot_button_Callback(hObject, eventdata, handles)
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
function radiobutton1_Callback(hObject, eventdata, handles)
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
    objective_function = str2func(['IDObjective',objective]);
    q_data = get(handles.QTable,'Data');
    id_objective = objective_function(q_data');
    solver_function = str2func(['IDSolver',solver_class]);
    if(strcmp(solver_class,'LinProg'))
        id_solver = solver_function(id_objective,ID_LP_SolverType.(solver_type));
    else
        id_solver = solver_function(id_objective,ID_QP_SolverType.(solver_type));
    end
    contents = cellstr(get(handles.plot_type_popup,'String'));
    plot_type = contents{get(handles.plot_type_popup,'Value')};
%     id_objective = IDObjectiveMinQuadCableForce(ones(dynObj.numCables,1));
%     id_solver = IDSolverQuadProg(id_objective, ID_QP_SolverType.OPTITOOLBOX_OOQP);
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
    %plot_axis = [0 1 0 1 -0.1 0.1];
    plot_axis = [-0.2 0.2 -0.2 0.2 -0.1 0.3];
    %idsim.plotMovie(plot_axis, [fileparts(mfilename('fullpath')), '\test.avi'], 2, 500, 640);
    % idsim.plotJointSpace();
    % idsim.plotAngularAcceleration();
    % idsim.plotCableLengths();
    % idsim.plotBodyCOG();
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
    path_string = path_string(1:strfind(path_string, 'scripts')-2);
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
    path_string                             =   fileparts(mfilename('fullpath'));
    path_string                             = path_string(1:strfind(path_string, 'scripts')-2);
    if(nargin>1)
        save(file_path,'state');
    else
        save([path_string,'\logs\dynamics_gui_state.mat'],'state')
    end
end

function run_forward_dynamics(handles,dynObj,trajectory_xmlobj)
    % This will be added once script_FD has been fixed
end

function format_Q_table(numCables,QTable)
    set(QTable,'Data',ones(1,numCables));
    set(QTable,'ColumnWidth',{20});
    set(QTable,'ColumnEditable',true(1,numCables));
    Q_extent = get(QTable,'Extent');
    Q_position = zeros(1,4);
    Q_position(1) = 35;  Q_position(2) = 1;
    if(numCables>2)
        Q_position(3) = 7*Q_extent(3)/(numCables+1);
    else
        Q_position(3) = Q_extent(3); 
    end
    Q_position(4) = 2.6*Q_extent(4);
    set(QTable,'Position',Q_position);
end


