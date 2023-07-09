classdef ReconfigurationProblemSolver
    
    
    properties
        SolverType
    end
    
    methods
        function obj = ReconfigurationProblemSolver(SolverType)
            obj.SolverType = SolverType;
        end
        
        function [solutions,status,Comp_Time] = solveProblem(obj,Problem,Path,RelaxOrder)
            switch obj.SolverType
                case ReconfigurationSolverType.GloptiPoly
                    scriptFiles = POPGenerator(Problem,Path);
                    clear global;
                    run(scriptFiles{1})
                    
                    
                    ConstraintFunctions;
                    global MMM
                    MMM.yalmip = 1;
%                      MMM.testol = 1e-6;
% MMM.ranktol= 1.0000e-03
%                  MMM.pivotol= 1.0000e-03;
                    objfun = min(ObjectiveFunction);
                    if ~isempty(RelaxOrder)
                        if exist('Moments','var')
                            P = msdp(objfun,Moments,ConstraintFunctions,RelaxOrder);
                        else
                            P = msdp(objfun,ConstraintFunctions,RelaxOrder);
                        end
                    else
                        
                        if exist('Moments','var')
                            P = msdp(objfun,Moments,ConstraintFunctions);
                        else
                            P = msdp(objfun,ConstraintFunctions)%,RelaxOrder);
                        end
                    end
                    CurrentOrder = MMM.M{1, 1}.ord;
                    disp('Solving problem...')
                    tic
                    [status,opt_z,M,info] = msol(P);
                     Comp_Time = toc;
                    %                     [A,b,c,K] = msedumi(P);
                    %                     pars.fid = 1;
                    %                     pars.eps = 1e-5;
                    %                     [x,y,info] = sedumi(A,b,c,K,pars);
                    countOrder = CurrentOrder;
                    count_order = 1;
                    while status == 0
                        disp('Might NOT be optimal solution, try to increase the order')
                        if exist('Moments','var')
                            P = msdp(min(ObjectiveFunction),Moments,ConstraintFunctions,countOrder +count_order );
                        else
                            P = msdp(min(ObjectiveFunction),ConstraintFunctions,countOrder +count_order);
                        end
                        tic
                        [status,opt_z,M,~] = msol(P);
                        Comp_Time = toc;
                        count_order = count_order +1;
                        if count_order >=4
                            status = -1;
                        end
                    end
                    status
                    opt_z
                    if status ~= -1
                        run(scriptFiles{2})
                        solutions = X_opt;
                    else
                        
                        solutions = [];
                    end
                    
                    clear global;
                case ReconfigurationSolverType.Fmincon
                    scriptFiles = FminconGenerator(Problem,Path);
                    run(scriptFiles{3})
                    x0 = lb;
                    tic
                    [x,~,status,~] = fmincon(@FminconObjectiveFun,x0,A,b,Aeq,beq,lb,ub,@FminconConstraints);
                    Comp_Time = toc;
                    
                    X_opt.value = x;
                    Vars = [Problem.vars,Problem.AuxVar];
                    for i = 1:numel(Vars)
                        X_opt.VarName{i} = char(Vars(i));
                    end
                    solutions = X_opt;
                case ReconfigurationSolverType.Ga
                    scriptFiles = FminconGenerator(Problem,Path);
                    run(scriptFiles{3})
                    x0 = lb;
                    tic
                    [x,~,status,~] = ga(@FminconObjectiveFun,numel(x0),A,b,Aeq,beq,lb,ub,@FminconConstraints);
                    Comp_Time = toc;
                    
                    X_opt.value = x;
                    Vars = [Problem.vars,Problem.AuxVar];
                    for i = 1:numel(Vars)
                        X_opt.VarName{i} = char(Vars(i));
                    end
                    solutions = X_opt;
                case ReconfigurationSolverType.Paretosearch
                    scriptFiles = FminconGenerator(Problem,Path);
                    run(scriptFiles{3})
                    x0 = lb;
                    tic
                    [x,~,status,~]  = paretosearch(@FminconObjectiveFun,numel(x0),A,b,Aeq,beq,lb,ub,@FminconConstraints);
                    Comp_Time = toc;
                    X_opt.value = x;
                    Vars = [Problem.vars,Problem.AuxVar];
                    for i = 1:numel(Vars)
                        X_opt.VarName{i} = char(Vars(i));
                    end
                    solutions = X_opt;
                    
                case ReconfigurationSolverType.PSO
                    scriptFiles = FminconGenerator(Problem,Path);
                    run(scriptFiles{3})
                    x0 = ub; %options.PlotFcns = {@psoplotbestf,@psoplotswarmsurf};
                    tic
                    [x,fval,status,output,population,scores] = pso(@FminconObjectiveFun,numel(x0),A,b,Aeq,beq,lb,ub,@FminconConstraints);
                    Comp_Time = toc;
                    X_opt.value = x;
                    Vars = [Problem.vars,Problem.AuxVar];
                    for i = 1:numel(Vars)
                        X_opt.VarName{i} = char(Vars(i));
                    end
                    solutions = X_opt;
                    
                otherwise
                    CASPR_log.Error('Solver type is not available');
            end
        end
    end
end

