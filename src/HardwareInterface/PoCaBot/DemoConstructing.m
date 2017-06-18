function varargout = DemoConstructing(varargin)
% DEMOCONSTRUCTING MATLAB code for DemoConstructing.fig
%      DEMOCONSTRUCTING, by itself, creates a new DEMOCONSTRUCTING or raises the existing
%      singleton*.
%
%      H = DEMOCONSTRUCTING returns the handle to a new DEMOCONSTRUCTING or the handle to
%      the existing singleton*.
%
%      DEMOCONSTRUCTING('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DEMOCONSTRUCTING.M with the given input arguments.
%
%      DEMOCONSTRUCTING('Property','Value',...) creates a new DEMOCONSTRUCTING or raises
%      the existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before DemoConstructing_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to DemoConstructing_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help DemoConstructing

% Last Modified by GUIDE v2.5 15-Jun-2017 10:08:27

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @DemoConstructing_OpeningFcn, ...
                   'gui_OutputFcn',  @DemoConstructing_OutputFcn, ...
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

% --- Executes just before DemoConstructing is made visible.
function DemoConstructing_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to DemoConstructing (see VARARGIN)

% Choose default command line output for DemoConstructing
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

initialize_gui(hObject, handles, false);

paras.isGetBackToInitPosition = false;


% UIWAIT makes DemoConstructing wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = DemoConstructing_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes when selected object changed in unitgroup.
function unitgroup_SelectionChangedFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in unitgroup 
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if (hObject == handles.english)
    set(handles.text5, 'String', 'cu.in');
    set(handles.text6, 'String', 'lb');
else
    set(handles.text5, 'String', 'cu.m');
    set(handles.text6, 'String', 'kg');
end

% --------------------------------------------------------------------
function initialize_gui(fig_handle, handles, isreset)
loadState(handles);
handles = guidata(handles.figure1);
set(handles.edit_pickupfile, 'String',handles.gui_state.strPickup);
set(handles.edit_placingfile, 'String',handles.gui_state.strPlacing);
set(handles.editNextBrick, 'String',handles.gui_state.nextBrick);
set(handles.editQ0, 'String',mat2str(handles.gui_state.q0));
set(handles.editInitialPosition, 'String',mat2str(handles.gui_state.initMotorPos));

% Update handles structure
guidata(handles.figure1, handles);



% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[filename, pathname] = uigetfile();
filepath = strcat(pathname, filename);
set(handles.edit_pickupfile, 'String', filename);


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[filename, pathname] = uigetfile();
filepath = strcat(pathname, filename);
set(handles.edit_placingfile, 'String', filename);


% --- Executes on button press in btnRUN.
function btnRUN_Callback(hObject, eventdata, handles)
% hObject    handle to btnRUN (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.btnRUN, 'Enable', 'off');

function loadState(handles)
    % load all of the settings and initialise the values to match
    file_name = 'C:\Users\Tristan\Desktop\Constructing Demo\demo_gui_state.mat';
    if(exist(file_name,'file'))
        handles.gui_state = load(file_name,'gui_state');
    else
        handles.gui_state.strPickup = 'xxxx.txt';
        handles.gui_state.strPlacing = 'yyyy.txt';
        handles.gui_state.nextBrick = 1;
        handles.gui_state.q0 = zeros(6,1);
        handles.gui_state.initMotorPos = zeros(8,1);
    end
    guidata(handles.figure1, handles);
    
function saveState(handles)
    % Save all of the settings
    % Check if the log folder exists
    if(exist('C:\Users\Tristan\Desktop\Constructing Demo','dir')~=7)
        mkdir('C:\Users\Tristan\Desktop\Constructing Demo');
    end
    gui_state = handles.gui_state;
    save('C:\Users\Tristan\Desktop\Constructing Demo\demo_gui_state.mat','gui_state');



function editInitialPosition_Callback(hObject, eventdata, handles)
% hObject    handle to editInitialPosition (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editInitialPosition as text
%        str2double(get(hObject,'String')) returns contents of editInitialPosition as a double


% --- Executes during object creation, after setting all properties.
function editInitialPosition_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editInitialPosition (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in cboxGetBack.
function cboxGetBack_Callback(hObject, eventdata, handles)
% hObject    handle to cboxGetBack (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cboxGetBack
paras.isGetBackToInitPosition = get(hObject,'Value');




% --- Executes on selection change in lboxNextBrick.
function lboxNextBrick_Callback(hObject, eventdata, handles)
% hObject    handle to lboxNextBrick (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns lboxNextBrick contents as cell array
%        contents{get(hObject,'Value')} returns selected item from lboxNextBrick
contents = cellstr(get(hObject,'String'));
content = contents{get(hObject,'Value')};

% --- Executes during object creation, after setting all properties.
function lboxNextBrick_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lboxNextBrick (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editQ0_Callback(hObject, eventdata, handles)
% hObject    handle to editQ0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editQ0 as text
%        str2double(get(hObject,'String')) returns contents of editQ0 as a double

handles.gui_state.q0 = str2num(get(hObject,'String'));


% --- Executes during object creation, after setting all properties.
function editQ0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editQ0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editNextBrick_Callback(hObject, eventdata, handles)
% hObject    handle to editNextBrick (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editNextBrick as text
%        str2double(get(hObject,'String')) returns contents of editNextBrick as a double


% --- Executes during object creation, after setting all properties.
function editNextBrick_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editNextBrick (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
