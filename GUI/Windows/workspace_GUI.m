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

    % Last Modified by GUIDE v2.5 09-Feb-2019 07:24:01

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
function workspace_condition_popup_Callback(~, ~, ~) %#ok<DEFNU>
    % hObject    handle to workspace_condition_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns workspace_condition_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from workspace_condition_popup
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
    workspace_str = lower(workspace_str);
    workspace_str = strrep(workspace_str,'_',' ');
    set(hObject, 'String', workspace_str);
end

% --- Executes on selection change in workspace_type_popup.
function workspace_type_popup_Callback(~, ~, ~) %#ok<DEFNU>
    % hObject    handle to workspace_type_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns workspace_type_popup contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from workspace_type_popup
end

% --- Executes during object creation, after setting all properties.
function workspace_type_popup_CreateFcn(hObject, ~, ~)%#ok<DEFNU>
    % hObject    handle to workspace_type_popup (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
    settingsXMLObj =  GUIOperations.GetSettings('/GUI/XML/workspaceXML.xml');
    setappdata(hObject,'settings',settingsXMLObj);
    workspace_str = GUIOperations.XmlObj2StringCellArray(settingsXMLObj.getElementsByTagName('simulator').item(0).getElementsByTagName('workspace_type')...
                    ,'id');
    workspace_str = lower(workspace_str);
    workspace_str = strrep(workspace_str,'_',' ');
    set(hObject, 'String', workspace_str);
end

% % Workspace Metric
% % --- Executes on selection change in workspace_metric_popup.
% function workspace_metric_popup_Callback(~, ~, ~) %#ok<DEFNU>
%     % hObject    handle to workspace_metric_popup (see GCBO)
%     % eventdata  reserved - to be defined in a future version of MATLAB
%     % handles    structure with handles and user data (see GUIDATA)
% 
%     % Hints: contents = cellstr(get(hObject,'String')) returns workspace_metric_popup contents as cell array
%     %        contents{get(hObject,'Value')} returns selected item from workspace_metric_popup
% end
% 
% % --- Executes during object creation, after setting all properties.
% function workspace_metric_popup_CreateFcn(hObject, ~, ~) %#ok<DEFNU>
%     % hObject    handle to workspace_metric_popup (see GCBO)
%     % eventdata  reserved - to be defined in a future version of MATLAB
%     % handles    empty - handles not created until after all CreateFcns called
% 
%     % Hint: popupmenu controls usually have a white background on Windows.
%     %       See ISPC and COMPUTER.
%     if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
%         set(hObject,'BackgroundColor','white');
%     end
%     settingsXMLObj = GUIOperations.GetSettings('/GUI/XML/workspaceXML.xml');
%     workspace_str = GUIOperations.XmlObj2StringCellArray(settingsXMLObj.getElementsByTagName('simulator').item(0).getElementsByTagName('workspace_metrics').item(0).getElementsByTagName('workspace_metric')...
%         ,[]);
%     workspace_str = [{' '},workspace_str];
%     set(hObject, 'String', workspace_str);
% end

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
% --- Executes on button press in generate_button.
function generate_button_Callback(~, ~, handles) %#ok<DEFNU>
    % hObject    handle to generate_button (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    %% Clear data
    clc; warning off; %#ok<WNOFF> %close all;
    %% Model setup
    modObj = getappdata(handles.cable_text,'modObj');
    % Set up the workspace simulator
    % First the grid
    q_begin         =   modObj.bodyModel.q_min; q_end = modObj.bodyModel.q_max;
    q_step          =   (modObj.bodyModel.q_max - modObj.bodyModel.q_min)/5;
    uGrid           =   UniformGrid(q_begin,q_end,q_step,'step_size');
    % First determine if ray or point
    contents = cellstr(get(handles.workspace_type_popup,'String'));
    wt_string = contents{get(handles.workspace_type_popup,'Value')};
    if(strcmp(wt_string,'point'))
        % First the condition
        contents = cellstr(get(handles.workspace_condition_popup,'String'));
        wc_string = contents{get(handles.workspace_condition_popup,'Value')};
        settings = getappdata(handles.workspace_condition_popup,'settings');
        if(~strcmp(wc_string,'wrench feasible'))
            wc_string = upper(wc_string);
            wc_string = strrep(wc_string,' ','_');
            w_condition = {WorkspaceConditionBase.CreateWorkspaceCondition(eval(['WorkspaceConditionType.',wc_string]),[])};
        else
            wc_string = 'WRENCH_FEASIBLE';
            i_max     = 2^(modObj.numDofs)-1;
            w_set     = zeros(modObj.numDofs,i_max+1);
            flag_vec  = zeros(modObj.numDofs,1);
            min_vec     = -ones(modObj.numDofs,1);
            max_vec     = ones(modObj.numDofs,1);
            for k = 0:i_max
                flag_set = dec2bin(k,modObj.numDofs);
                flag_vec(:) = str2num(flag_set(:)); %#ok<ST2NM>
                w_set(:,k+1) = min_vec.*(~flag_vec) + max_vec.*flag_vec;
            end 
            w_condition = {WrenchFeasibleCondition(w_set, [])};
        end
        % Then the metric
        %w_metric = {WorkspaceMetricBase.CreateWorkspaceMetric(WorkspaceMetricType.CONDITION_NUMBER,[])};
        % FOR THE MOMENT NO OPTIONS ON CONNECTIVITY
        w_connectivity  =   WorkspaceConnectivityBase.CreateWorkspaceConnectivityCondition(WorkspaceConnectivityType.GRID,uGrid);
        %% Now initialise the simulation
        CASPR_log.Info('Start Setup Simulation');
        set(handles.status_text,'String','Setting up simulation');
        drawnow;
        start_tic       =   tic;
        wsim            =   PointWorkspaceSimulator(modObj, uGrid, w_condition, {}, w_connectivity);
        time_elapsed    =   toc(start_tic);
        CASPR_log.Info(sprintf('End Setup Simulation : %f seconds', time_elapsed));

        CASPR_log.Info('Start Running Simulation');
        set(handles.status_text,'String','Simulation running');
        drawnow;
        start_tic       =   tic;
        wsim.run(); 
        time_elapsed    =   toc(start_tic);
        CASPR_log.Info(sprintf('End Running Simulation : %f seconds', time_elapsed));
        CASPR_log.Info('Start Plotting Simulation');
        drawnow;
        start_tic       =   tic;
        %         figure
        plot_axis = 1:size(wsim.grid.delta_q',2);
        fixed_variables = wsim.grid.q_begin' + wsim.grid.delta_q' .* size(wsim.grid.delta_q',2);
        cartesian_workspace_plot = wsim.workspace.plotWorkspace(plot_axis, w_condition, [], fixed_variables);
        time_elapsed    =   toc(start_tic);
        CASPR_log.Info(sprintf('End Plotting Simulation : %f seconds\n', time_elapsed));
        set(handles.status_text,'String','No simulation running');
    else
        min_segment_percentage = 20;
        contents = cellstr(get(handles.workspace_condition_popup,'String'));
        wc_string = contents{get(handles.workspace_condition_popup,'Value')};
        wc_string = upper(wc_string);
        wc_string = strrep(wc_string,' ','_');
        w_condition     =   {WorkspaceRayConditionBase.CreateWorkspaceRayCondition(eval(['WorkspaceRayConditionType.',wc_string]),min_segment_percentage,modObj)};
%         w_metrics       =   {WorkspaceMetricBase.CreateWorkspaceMetric(WorkspaceMetricType.TENSION_FACTOR)};
        opt             =   RayWorkspaceSimulatorOptions(false,false);
        
        % Start the simulation
        CASPR_log.Info('Start Setup Simulation');
        set(handles.status_text,'String','Setting up simulation');
        drawnow;
        start_tic       =   tic;
        wsim            =   RayWorkspaceSimulator(modObj,uGrid,opt);
        time_elapsed    =   toc(start_tic);
        CASPR_log.Info(sprintf('End Setup Simulation : %f seconds', time_elapsed));
        
        % Run the simulation
        CASPR_log.Info('Start Running Simulation');
        set(handles.status_text,'String','Simulation running');
        drawnow;
        start_tic       =   tic;
        wsim.run(w_condition,[])
        time_elapsed    =   toc(start_tic);
        CASPR_log.Info(sprintf('End Running Simulation : %f seconds', time_elapsed));
        
        % Plot the simulation
        CASPR_log.Info('Start Plotting Simulation');
        drawnow;
        start_tic       =   tic;
        wsim.plotGraph();
        time_elapsed    =   toc(start_tic);
        CASPR_log.Info(sprintf('End Plotting Simulation : %f seconds\n', time_elapsed));
        set(handles.status_text,'String','No simulation running');
    end
    setappdata(handles.figure1,'sim',wsim);
    assignin('base','workspace_sim',wsim); 
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
        
    end
end

% --- Executes on button press in generate_script_button.
function generate_script_button_Callback(~, ~, handles) %#ok<DEFNU>
% hObject    handle to generate_script_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    base_folder = CASPR_configuration.LoadHomePath();
    contents = cellstr(get(handles.workspace_condition_popup,'String'));
    model_str = cellstr(get(handles.model_text,'String'));
    cable_str = cellstr(get(handles.cable_text,'String'));
    wc_string = contents{get(handles.workspace_condition_popup,'Value')};
    contents = cellstr(get(handles.workspace_type_popup,'String'));
    wt_string = contents{get(handles.workspace_type_popup,'Value')};
    if(strcmp(wt_string,'point'))
        if(strcmp(wc_string,'wrench closure'))
            r_string = [base_folder,'/GUI/template_scripts/workspace/script_workspace_condition_wrench_closure_point_template.m'];
        elseif(strcmp(wc_string,'wrench feasible'))
            r_string = [base_folder,'/GUI/template_scripts/workspace/script_workspace_condition_wrench_feasible_point_template.m'];
        elseif(strcmp(wc_string,'static'))
            r_string = [base_folder,'/GUI/template_scripts/workspace/script_workspace_condition_static_point_template.m'];
        elseif(strcmp(wc_string,'interference free'))
            r_string = [base_folder,'/GUI/template_scripts/workspace/script_workspace_condition_interference_free_point_template.m'];
        end
    else
        if(strcmp(wc_string,'wrench closure'))
            r_string = [base_folder,'/GUI/template_scripts/workspace/script_workspace_condition_wrench_closure_ray_template.m'];
        elseif(strcmp(wc_string,'wrench feasible'))
            CASPR_log.Error('Wrench feasible workspace is currently not supported in ray mode')
        elseif(strcmp(wc_string,'static'))
            CASPR_log.Error('Static workspace is currently not supported in ray mode')
        elseif(strcmp(wc_string,'interference free'))
            r_string = [base_folder,'/GUI/template_scripts/workspace/script_workspace_condition_interference_free_ray_template.m'];
        end
    end
    w_string = [base_folder,'/scripts/local/GUI_script_autogenerated.m'];
    r_fid = fopen(r_string,'r');
    w_fid = fopen(w_string,'w');
    while(~feof(r_fid))
        s = fgetl(r_fid);
        % Determine if comment
        new_s = regexprep(s,'%','%%');
        % Replace all references to the model
        new_s = regexprep(new_s,'Example planar XY',model_str);
        new_s = regexprep(new_s,'basic',cable_str);
        if (CASPR_configuration.LoadDevModelConfig())
            new_s = regexprep(new_s, 'ModelConfig', 'DevModelConfig');
        end
        fprintf(w_fid,[new_s,'\n']);
    end
    fclose(r_fid);
    fclose(w_fid);
    edit(w_string)
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
    state.workspace_type_popup_value        =   get(handles.workspace_type_popup,'value');
%     state.workspace_metric_popup_value      =   get(handles.workspace_metric_popup,'value');
    state.plot_type_popup                   =   get(handles.plot_type_popup,'value');
    if(nargin>1)
        save(file_path,'state');
    else
        path_string                             =   CASPR_configuration.LoadHomePath();
        save([path_string,'/GUI/config/workspace_gui_state.mat'],'state')
    end
end

function loadState(handles)
    % load all of the settings and initialise the values to match
    path_string = CASPR_configuration.LoadHomePath();
    file_name = [path_string,'/GUI/config/caspr_gui_state.mat'];
    set(handles.status_text,'String','No simulation running');
    if(exist(file_name,'file'))
        load(file_name)
        set(handles.model_text,'String',state.model_text);
        set(handles.cable_text,'String',state.cable_text);
        setappdata(handles.cable_text,'modObj',state.modObj);
        file_name = [path_string,'/GUI/config/workspace_gui_state.mat'];
        if(exist(file_name,'file'))
            load(file_name)
            mp_text = get(handles.model_text,'String');
            cs_text = get(handles.cable_text,'String');
            if(strcmp(mp_text,state.model_text)&&strcmp(cs_text,state.cable_text))
                set(handles.workspace_condition_popup,'value',state.workspace_condition_popup_value);
                set(handles.workspace_type_popup,'value',state.workspace_type_popup_value);
%                 set(handles.workspace_metric_popup,'value',state.workspace_metric_popup_value);
                set(handles.plot_type_popup,'value',state.plot_type_popup);
                plot_type_popup_Callback(handles.plot_type_popup,[],handles);
            else
                plot_type_popup_Callback(handles.plot_type_popup,[],handles);
            end
        end
    end
end
