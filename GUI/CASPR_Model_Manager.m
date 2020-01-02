function varargout = CASPR_Model_Manager(varargin)
    % CASPR_MODEL_MANAGER MATLAB code for CASPR_Model_Manager.fig
    %      CASPR_MODEL_MANAGER, by itself, creates a new CASPR_MODEL_MANAGER or raises the existing
    %      singleton*.
    %
    %      H = CASPR_MODEL_MANAGER returns the handle to a new CASPR_MODEL_MANAGER or the handle to
    %      the existing singleton*.
    %
    %      CASPR_MODEL_MANAGER('CALLBACK',hObject,eventData,handles,...) calls the local
    %      function named CALLBACK in CASPR_MODEL_MANAGER.M with the given input arguments.
    %
    %      CASPR_MODEL_MANAGER('Property','Value',...) creates a new CASPR_MODEL_MANAGER or raises the
    %      existing singleton*.  Starting from the left, property value pairs are
    %      applied to the GUI before CASPR_Model_Manager_OpeningFcn gets called.  An
    %      unrecognized property name or invalid value makes property application
    %      stop.  All inputs are passed to CASPR_Model_Manager_OpeningFcn via varargin.
    %
    %      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
    %      instance to run (singleton)".
    %
    % See also: GUIDE, GUIDATA, GUIHANDLES

    % Edit the above text to modify the response to help CASPR_Model_Manager

    % Last Modified by GUIDE v2.5 22-Jun-2017 18:18:19

    % Begin initialization code - DO NOT EDIT
    gui_Singleton = 1;
    gui_State = struct('gui_Name',       mfilename, ...
                       'gui_Singleton',  gui_Singleton, ...
                       'gui_OpeningFcn', @CASPR_Model_Manager_OpeningFcn, ...
                       'gui_OutputFcn',  @CASPR_Model_Manager_OutputFcn, ...
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


% --- Executes just before CASPR_Model_Manager is made visible.
function CASPR_Model_Manager_OpeningFcn(hObject, ~, handles, varargin)
    % This function has no output args, see OutputFcn.
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % varargin   command line arguments to CASPR_Model_Manager (see VARARGIN)

    % Choose default command line output for CASPR_Model_Manager
    handles.output = hObject;

    % Update handles structure
    guidata(hObject, handles);
    set_robotlist(handles)
    % UIWAIT makes CASPR_Model_Manager wait for user response (see UIRESUME)
    % uiwait(handles.figure1);
end


% --- Outputs from this function are returned to the command line.
function varargout = CASPR_Model_Manager_OutputFcn(~, ~, handles) 
    % varargout  cell array for returning output args (see VARARGOUT);
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Get default command line output from handles structure
    varargout{1} = handles.output;
end


% --- Executes on button press in add_button.
function add_button_Callback(~, ~, handles) %#ok<DEFNU>
    % hObject    handle to add_button (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    
    % Read the elements from the table
    name = get(handles.name_box,'String');
    if(CASPR_configuration.LoadDevModelConfig())
        model_path = DevModelConfig.MODEL_FOLDER_PATH;
        folder_path = [uigetdir([CASPR_configuration.LoadModelConfigPath(), model_path,'/']),'/'];
        % Change slashes to make OS consistent
        folder_path = strrep(folder_path,'\','/');
        ind = strfind(folder_path,model_path);
        CASPR_log.Assert(~isempty(ind),'Folder must be a subfolder of the default choice');    
        folder_path = folder_path(ind+length(model_path):length(folder_path));
    else
    	model_path = ModelConfig.MODEL_FOLDER_PATH;
        folder_path = [uigetdir([CASPR_configuration.LoadModelConfigPath(), model_path]),'/'];
        % Change slashes to make OS consistent
        folder_path = strrep(folder_path,'\','/');
        ind = strfind(folder_path,model_path);
        CASPR_log.Assert(~isempty(ind),'Folder must be a subfolder of the default choice');    
        folder_path = folder_path(ind+length(model_path):length(folder_path));
    end
    CASPR_log.Assert(sum(~(isstrprop(name,'alphanum')|(name==' ')|(name=='_')|(name=='-')))==0,'Robot Name must contain only numbers and letters');
    if(get(handles.advanced_toggle,'Value'))
        table_data = get(handles.model_table,'Data');
        folder = [folder_path,table_data{1}];
        bodies = table_data{2};
        cables = table_data{3};
        trajectories = table_data{4};
        op_spaces = table_data{5};
    else
        % Add conversions
        name_for_files = strrep(name,' ','_');
        name_for_files = strrep(name_for_files,'-','_');
        name_for_files = strrep(name_for_files,'.','_');
        folder = [folder_path,name_for_files,'/'];
        bodies = [name_for_files,'_bodies.xml'];
        cables = [name_for_files,'_cables.xml'];
        trajectories = [name_for_files,'_trajectories.xml'];
        op_spaces = [name_for_files,'_operational_spaces.xml'];
    end
       
    % Call the appropriate model config operations
    if(CASPR_configuration.LoadDevModelConfig())
        ModelConfigManager.AddDevModelConfig(name,folder,bodies,cables,trajectories,op_spaces);
    else
        ModelConfigManager.AddModelConfig(name,folder,bodies,cables,trajectories,op_spaces);
    end
    set_robotlist(handles)
    addpath(genpath(CASPR_configuration.LoadModelConfigPath()));
end

% --- Executes on button press in remove_button.
function remove_button_Callback(~, ~, handles) %#ok<DEFNU>
    % hObject    handle to remove_button (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % Read the elements from the table
    name = get(handles.remove_box,'String');
    % Call the appropriate model config operations
    if(CASPR_configuration.LoadDevModelConfig())
        ModelConfigManager.RemoveDevModelConfig(name);
    else
        ModelConfigManager.RemoveModelConfig(name);
    end
    set_robotlist(handles)
    set(handles.remove_box,'string','');
end

% --- Executes during object creation, after setting all properties.
function model_table_CreateFcn(hObject, ~, ~) %#ok<DEFNU>
    % hObject    handle to model_table (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called
    set(hObject,'Data',cell(1,5),'ColumnName',{'Folder','Bodies XML Filename','Cables XML Filename','Trajectories XML Filename','Operational Spaces XML Filename'})
    set(hObject,'Visible','Off');
end

function name_box_Callback(~, ~, ~) %#ok<DEFNU>
    % hObject    handle to name_box (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: get(hObject,'String') returns contents of name_box as text
    %        str2double(get(hObject,'String')) returns contents of name_box as a double
end

% --- Executes during object creation, after setting all properties.
function name_box_CreateFcn(hObject, ~, ~) %#ok<DEFNU>
    % hObject    handle to name_box (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end

% --- Executes on button press in advanced_toggle.
function advanced_toggle_Callback(hObject, ~, handles) %#ok<DEFNU>
    % hObject    handle to advanced_toggle (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hint: get(hObject,'Value') returns toggle state of advanced_toggle
    if(get(hObject,'Value'))
        set(handles.model_table,'Visible','On');
    else
        set(handles.model_table,'Visible','Off');
    end
end


% --- Executes on selection change in robotlist_menu.
function robotlist_menu_Callback(~, ~, ~) %#ok<DEFNU>
    % hObject    handle to robotlist_menu (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns robotlist_menu contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from robotlist_menu
end


% --- Executes during object creation, after setting all properties.
function robotlist_menu_CreateFcn(hObject, ~, ~) %#ok<DEFNU>
    % hObject    handle to robotlist_menu (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
    robotlist_str = ModelConfigManager.GetModelConfigListNames();
    set(hObject, 'Value', 1);
    set(hObject, 'String', robotlist_str);
end

function set_robotlist(handles)
    if(CASPR_configuration.LoadDevModelConfig())
        robotlist_str = ModelConfigManager.GetDevModelConfigListNames();
    else
        robotlist_str = ModelConfigManager.GetModelConfigListNames();
    end
    set(handles.robotlist_menu, 'Value', 1);
    set(handles.robotlist_menu, 'String', robotlist_str);
end


function remove_box_Callback(~, ~, ~) %#ok<DEFNU>
    % hObject    handle to remove_box (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: get(hObject,'String') returns contents of remove_box as text
    %        str2double(get(hObject,'String')) returns contents of remove_box as a double
end


% --- Executes during object creation, after setting all properties.
function remove_box_CreateFcn(hObject, ~, ~) %#ok<DEFNU>
    % hObject    handle to remove_box (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end


% --- Executes during object creation, after setting all properties.
function add_instructions_text_CreateFcn(hobject, ~, ~) %#ok<DEFNU>
% hObject    handle to add_instructions_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
    text_string = cell(5,1);
    text_string{1} = 'Instructions:';
    text_string{2} = 'Step 1 - Enter the desired robot name (only use letters and numbers).'; 
    text_string{3} = 'Step 2 - Click the add model button and select the base folder to save into';
    if(CASPR_configuration.LoadDevModelConfig())
        text_string{4} = '(this must be a subfolder of /data/model_config/indev_models).' ;
    else
        text_string{4} = '(this must be a subfolder of /data/model_config/models).' ;
    end
    text_string{5} = 'Advanced options allows the folder and file names to be explicitly specified.';
    set(hobject,'String',text_string)
end
