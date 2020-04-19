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
            settings =  XmlOperations.XmlReadRemoveIndents([CASPR_configuration.LoadHomePath(),str]);
        end

        % Converts and xmlobj into a cell array of strings.  This function
        % looks through the xml object and returns all strings with the
        % attribute given by str.
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

        % Create a tab group for the gui
        function CreateTabGroup(handles)
            tabgp = uitabgroup(handles.tab_panel,'Position',[0 0 1 1]);
            % A temporary hack to make the figures plot correctly
            tab1 = uitab(tabgp,'Title','0');
            ax = axes;
            set(ax,'Parent',tab1,'OuterPosition',[0,0,1,1],'Xtick',[],'Ytick',[]);
            t1 = text(0.1,0.8,'CASPR GUI Tab','FontSize',25,'FontUnits','normalized');
            t2 = text(0.1,0.6,'Please enter your desired settings','FontSize',12,'FontUnits','normalized');
            t3 = text(0.1,0.52,'and then press run/generate to','FontSize',12,'FontUnits','normalized');
            t4 = text(0.1,0.44,'start the simulation.','FontSize',12,'FontUnits','normalized');
%             delete(tab1);
            setappdata(handles.figure1,'tabgp',tabgp);
        end

        % Generate a new plot for the GUI using the given plotting
        % function. This new plot will be inserted into a new tab.
        function GUIPlot(plot_type,sim,handles,figure_quantity,tab_toggle)
            plot_function = str2func(plot_type);
            if(tab_toggle)
                plot_function(sim);
            else
                tabgp = getappdata(handles.figure1,'tabgp');
                for i = 1:figure_quantity
                    tab(i) = uitab(tabgp,'Title',plot_type); %#ok<AGROW>
                    ax(i) = axes; %#ok<AGROW>
                    set(ax(i),'Parent',tab(i),'OuterPosition',[0,0,1,1])
                end
                plot_function(sim, ax);
                set(tabgp,'SelectedTab',tab(i));
            end
        end
    end
end
