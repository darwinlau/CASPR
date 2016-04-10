% Utilities to operate on the GUI
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   :
classdef GUIOperations
    
    methods (Static)
        % childNodes is a java type and hence is a handle class (no need to
        % return the childNodes again)
        function settings = GetSettings(str)
            path_string = fileparts(mfilename('fullpath'));
            path_string = path_string(1:strfind(path_string, 'src')-2);
            settings =  XmlOperations.XmlReadRemoveIndents([path_string,str]);
        end
        
        function str_cell_array = XmlObj2StringCellArray(xmlObj,str)
            if(isempty(str))
                str_cell_array = cell(1,xmlObj.getLength);
                % Extract the identifies from the cable sets
                for i =1:xmlObj.getLength
                    tempXMLObj = xmlObj.item(i-1);
                    str_cell_array{i} = char(tempXMLObj.getFirstChild.getData);
                end
            else
                str_cell_array = cell(1,xmlObj.getLength);
                % Extract the identifies from the cable sets
                for i =1:xmlObj.getLength
                    tempXMLObj = xmlObj.item(i-1);
                    str_cell_array{i} = char(tempXMLObj.getAttribute(str));
                end
            end
        end
            
        function CreateTabGroup(handles)
            tabgp = uitabgroup(handles.tab_panel,'Position',[0 0 1 1]);
            % A temporary hack to make the figures plot correctly
            tab1 = uitab(tabgp);
            ax = axes;
            set(ax,'Parent',tab1,'OuterPosition',[0,0,1,1])
            delete(tab1);
            setappdata(handles.figure1,'tabgp',tabgp);
        end
        
        function GUIPlot(plot_type,sim,handles,figure_quantity,tab_toggle)
            plot_function = str2func(plot_type);
            if(tab_toggle)
                plot_function(sim,[],[]);
            else
                tabgp = getappdata(handles.figure1,'tabgp');
                for i = 1:figure_quantity
                    tab(i) = uitab(tabgp,'Title',plot_type); %#ok<AGROW>
                    ax(i) = axes; %#ok<AGROW>
                    set(ax(i),'Parent',tab(i),'OuterPosition',[0,0,1,1])
                end
                plot_function(sim,[],ax);
            end
        end
    end
end
