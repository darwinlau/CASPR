function varargout = CASPR_Model_Wizard(varargin)
    % CASPR_MODEL_WIZARD MATLAB code for CASPR_Model_Wizard.fig
    %      CASPR_MODEL_WIZARD, by itself, creates a new CASPR_MODEL_WIZARD or raises the existing
    %      singleton*.
    %
    %      H = CASPR_MODEL_WIZARD returns the handle to a new CASPR_MODEL_WIZARD or the handle to
    %      the existing singleton*.
    %
    %      CASPR_MODEL_WIZARD('CALLBACK',hObject,eventData,handles,...) calls the local
    %      function named CALLBACK in CASPR_MODEL_WIZARD.M with the given input arguments.
    %
    %      CASPR_MODEL_WIZARD('Property','Value',...) creates a new CASPR_MODEL_WIZARD or raises the
    %      existing singleton*.  Starting from the left, property value pairs are
    %      applied to the GUI before CASPR_Model_Wizard_OpeningFcn gets called.  An
    %      unrecognized property name or invalid value makes property application
    %      stop.  All inputs are passed to CASPR_Model_Wizard_OpeningFcn via varargin.
    %
    %      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
    %      instance to run (singleton)".
    %
    % See also: GUIDE, GUIDATA, GUIHANDLES

    % Edit the above text to modify the response to help CASPR_Model_Wizard

    % Last Modified by GUIDE v2.5 05-Apr-2017 18:32:52

    % Begin initialization code - DO NOT EDIT
    gui_Singleton = 1;
    gui_State = struct('gui_Name',       mfilename, ...
                       'gui_Singleton',  gui_Singleton, ...
                       'gui_OpeningFcn', @CASPR_Model_Wizard_OpeningFcn, ...
                       'gui_OutputFcn',  @CASPR_Model_Wizard_OutputFcn, ...
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


% --- Executes just before CASPR_Model_Wizard is made visible.
function CASPR_Model_Wizard_OpeningFcn(hObject, ~, handles, varargin)
    % This function has no output args, see OutputFcn.
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % varargin   command line arguments to CASPR_Model_Wizard (see VARARGIN)

    % Choose default command line output for CASPR_Model_Wizard
    handles.output = hObject;

    % Update handles structure
    guidata(hObject, handles);

    % UIWAIT makes CASPR_Model_Wizard wait for user response (see UIRESUME)
    % uiwait(handles.figure1);
end


% --- Outputs from this function are returned to the command line.
function varargout = CASPR_Model_Wizard_OutputFcn(~, ~, handles) 
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
    table_data = get(handles.model_table,'Data');
    name = table_data{1};
    folder = table_data{2};
    bodies = table_data{3};
    cables = table_data{4};
    trajectories = table_data{5};
    % Call the appropriate model config operations
    ModelConfigManager.AddModelConfig(name,folder,bodies,cables,trajectories);
end



% --- Executes on button press in remove_button.
function remove_button_Callback(~, ~, handles) %#ok<DEFNU>
    % hObject    handle to remove_button (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % Read the elements from the table
    table_data = get(handles.model_table,'Data');
    name = table_data{1};
    % Call the appropriate model config operations
    ModelConfigManager.RemoveModelConfig(name);
end


% --- Executes during object creation, after setting all properties.
function model_table_CreateFcn(hObject, ~, ~) %#ok<DEFNU>
    % hObject    handle to model_table (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called
    set(hObject,'Data',cell(1,5),'ColumnName',{'Robot Name','Folder','Bodies XML Filename','Cables XML Filename','Trajectories XML Filename'})
end