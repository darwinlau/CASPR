classdef WrenchClosure < WorkspaceCondition
    %IDFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected, GetAccess = protected)
        options                         % The options for the wrench closure
    end
    
    methods
        %% Constructor for wrench closure workspace
        function w = WrenchClosure(method)
            w.options               =   optimset('display','off');
            if(nargin>0)
                if(strcmp(method,'quad_prog'))
                    w.method = WrenchClosureMethods.QP;
                elseif(strcmp(method,'tension_factor'))
                    w.method = WrenchClosureMethods.TF;
                elseif(strcmp(method,'unilateral_dexterity'))
                    w.method = WrenchClosureMethods.UD;
                elseif(strcmp(method,'semi_singular'))
                    w.method = WrenchClosureMethods.SS;
                elseif(strcmp(method,'combinatoric_null_space'))
                    w.method = WrenchClosureMethods.CNS;
                elseif(strcmp(method,'combinatoric_positive_span'))
                    w.method = WrenchClosureMethods.CPS;
                else
                    msg = 'Incorrect wrench method set';
                    error(msg);
                end
            % Translate the method into an enum
            else
                w.method = WrenchClosureMethods.QP;
            end 
        end
        
        %% Evaluate the wrench closure condition return true if satisfied 
        function inWorkspace = evaluate(obj,dynamics)
           if(obj.method == WrenchClosureMethods.QP)
               inWorkspace = wrench_closure_quadprog(dynamics,obj.options);
           elseif(obj.method == WrenchClosureMethods.TF)
               inWorkspace = wrench_closure_tension_factor(dynamics,obj.options);
           elseif(obj.method == WrenchClosureMethods.UD)
               inWorkspace = wrench_closure_unilateral_dexterity(dynamics,obj.options);
           elseif(obj.method == WrenchClosureMethods.SS)
               inWorkspace = wrench_closure_semi_singular(dynamics);
           elseif(obj.method == WrenchClosureMethods.CNS)
               inWorkspace = wrench_closure_combinatoric_null_space(dynamics,obj.options);
           elseif(obj.method == WrenchClosureMethods.CPS)
               inWorkspace = wrench_closure_combinatoric_positive_span(dynamics,obj.options);
           end
        end
        
        function [isConnected] = connected(obj,workspace,i,j,grid)
            % Connectiveness is evaluated using grid connectivity.
            % THIS FILE MAY NEED A DYNAMICS OBJECT ADDED AT A LATER DATE
            tol = 1e-6; l_x = size(workspace,1) - 1;
            isConnected = sum((abs(workspace(1:l_x,i) - workspace(1:l_x,j)) < grid.delta_q+tol)) + sum((abs(workspace(1:l_x,i) - workspace(1:l_x,j)-2*pi) < grid.delta_q+tol)) + sum((abs(workspace(1:l_x,i) - workspace(1:l_x,j)+2*pi) < grid.delta_q+tol)) == grid.n_dimensions;
        end
    end
end

