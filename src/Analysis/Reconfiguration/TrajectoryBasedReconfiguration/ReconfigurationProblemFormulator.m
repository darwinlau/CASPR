classdef ReconfigurationProblemFormulator < handle
    %UNTITLED6 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        model
        ReconfigElements
        Kinematic_Model
        OptimalValue
        valid_variable_id = [];
        error_digit_tf = 1e3; % planar robot in paper
        error_digit_cs = 1e3; % planar robot in paper
        %         error_digit_tf = 1e4; % spatial robot in paper
        %         error_digit_cs = 1e4; % spatial robot in paper
        Obj_Fun_idx = 1; %1 for TF, 2 for Min Cable Force
        ObjectiveType;
        ConstraintType;
        CompTime;
    end
    
    properties (Hidden)
        MaxReconfigNum;
        MaximumReconfigStep;
        AutoGenFilePath;
        DefaultRelaxOrder;
        RelaxOrder = [];
        ActivateWrenchFeasibleCondition = false;
        ProcessInterferenceAnalysis = false;
        FixedCableLength = false;
        VarRange;
        CurrentVar;
        Solver;
        RequestCable
        
        Interference_Condition = [];
        
        W_t = []
        F_max = [];
        Obstacles = [];
        
        CableLength = [];
        
        last_Cable_Index = [];
        last_TF = [];
        valid_variable = [];
        valid_length_var = [];
        Reconfig_Cable_ID = [];
        Cable_Array_ID = [];
        
        
        ifc_reconfig_cable = [];
    end
    methods
        function obj = ReconfigurationProblemFormulator(model,OA_Element,RobotType,ObjectiveType,ConstraintType,Options)
            obj.ObjectiveType = ObjectiveType;
            obj.ConstraintType = ConstraintType;
            obj.CurrentVar = zeros(size(OA_Element.Unknown_var))';
            for i = 1:2:numel(Options)
                if strcmpi(Options{i},'CableForceUpperBound')
                    obj.F_max =  Options{i+1};
                elseif strcmpi(Options{i},'Obstacles')
                    obj.Obstacles =  Options{i+1};
                elseif strcmpi(Options{i},'MaximumReconfigStep')
                    obj.MaximumReconfigStep =  Options{i+1};
                elseif strcmpi(Options{i},'IntitialVarValue')
                    obj.CurrentVar =  Options{i+1};
                elseif strcmpi(Options{i},'SolverType')
                    if isempty(Options{i+1})
                        obj.Solver = ReconfigurationProblemSolver(Fmincon);
                    end
                    obj.Solver = ReconfigurationProblemSolver(Options{i+1});
                end
            end
            
            obj.CompTime = 0;
            
            obj.Kinematic_Model = ReconfigureKinematicModel(model,OA_Element,RobotType);
            obj.MaxReconfigNum = OA_Element.Element_Count;
            obj.DefaultRelaxOrder = OA_Element.Element_Count;
            obj.Reconfig_Cable_ID = OA_Element.Reconfig_Cable_ID;
            
            
            obj.model = model;
            
            obj.AutoGenFilePath = [CASPR_configuration.LoadHomePath,'\AutoGenCodeFolder\'];
            obj.ReconfigElements = OA_Element;
            
            %             %%%%%%% hack here
            
            %             for i = 1:OA_Element.Element_Count
            %                 Range = OA_Element.Variable_Range(:,i);
            %                 OA_C = model.cableModel.r_OAs(1:3,OA_Element.Reconfig_Cable_ID(i));
            %
            %
            %                 R1 = norm(OA_Element.OA_path{i}(Range(2)) - OA_Element.OA_path{i}(Range(1)));
            %                 R2 = norm(OA_C - OA_Element.OA_path{i}(Range(1)));
            %                 obj.CurrentVar(i) = R2/R1*(Range(2) - Range(1)) + Range(1);
            %             end
            
            obj.ActivateWrenchFeasibleCondition = false;
            
            for i = 1:numel(obj.ConstraintType)
                if ReconfigurationConstraintType.InterferenceFree == obj.ConstraintType{i}
                    obj.ProcessInterferenceAnalysis = true;
                    if ~isempty(obj.Obstacles)
                        obj.Interference_Condition = {ReconfigConditionBase.CreateReconfigCondition(ReconfigConditionType.INTERFERENCE_CABLE_OBSTACLE,model,obj.Obstacles)};

                    end
                    break;
                end
            end
            
            
        end
        
        function Solve(obj,q,qd,qdd,w_ext,varargin)
            % condition = 1 -> wcc, condition = 2 -> wfc, condition = 3 ->
            % wcc const + min cable force
            
            if isempty(qd)
                qd = zeros(size(obj.model.q));
            end
            if isempty(qdd)
                qdd = zeros(size(obj.model.q));
            end
            
            if isempty(w_ext)
                w_ext = zeros(size(obj.model.q));
            end
            
            obj.model.update(q,qd,qdd,w_ext);
            
            
            obj.W_t = round((obj.model.M*qdd + obj.model.C + obj.model.G + obj.model.W_e),9);
            
            
            if ~isempty(varargin)
                for i = 1:2:numel(varargin)
                    if strcmpi(varargin{i},'MaximumReconfigNum')
                        obj.MaxReconfigNum =  varargin{i+1};
                    elseif strcmpi(varargin{i},'ActivateWrenchFeasibleCondition')
                        obj.ActivateWrenchFeasibleCondition =  varargin{i+1};
                    elseif strcmpi(varargin{i},'RelaxOrder')
                        obj.RelaxOrder =  varargin{i+1};
                    elseif strcmpi(varargin{i},'RequestCable')
                        obj.RequestCable =  varargin{i+1};
                    elseif strcmpi(varargin{i},'FixCableLength')
                        obj.FixedCableLength =  varargin{i+1};
                    elseif strcmpi(varargin{i},'MaximumReconfigStep')
                        obj.MaximumReconfigStep =  varargin{i+1};
                    end
                end
            end
            
            
            for i = 1:obj.model.numCables
                OA_0(:,i) =  obj.model.cableModel.cables{i}.attachments{1}.r_GA ;
            end
            OA_current = OA_0;
            
            
            
            if obj.ProcessInterferenceAnalysis
                
                OA_lower = OA_0;OA_upper = OA_0;
                for j = 1:numel(obj.Reconfig_Cable_ID)
                    numVarRequired = numel(obj.ReconfigElements.Info(j).VariableID);
                    U_var = obj.ReconfigElements.Info(j).VariableRange(1,:);
                    OA_lower(:, obj.Reconfig_Cable_ID(j)) =  obj.ReconfigElements.Info(j).Path(U_var);
                    
                end
                for j = 1:numel( obj.Reconfig_Cable_ID)
                    numVarRequired = numel(obj.ReconfigElements.Info(j).VariableID);
                    U_var = obj.ReconfigElements.Info(j).VariableRange(2,:);
                    OA_upper(:, obj.Reconfig_Cable_ID(j)) =  obj.ReconfigElements.Info(j).Path(U_var);
                end
                tic
                interference_range = obj.ReconfigElements.ReconfigurableRangeEvaluation(obj.model, q, obj.Interference_Condition);
%                 interference_range = ReconfigElement(obj.model, q, obj.Interference_Condition, OA_lower, OA_upper);
                toc
%                 for j = 1:numel(obj.Reconfig_Cable_ID)
%                     var = interference_range.FeasibleInterval{obj.Reconfig_Cable_ID(j)};
%                     closest_interval = interference_range.ClosestReconfigureRange(obj.Reconfig_Cable_ID(j));
%                     obj.VarRange(:,j) = interp1([0,1],obj.ReconfigElements.Variable_Range(:,j),var{closest_interval});
%                 end
                obj.VarRange = interference_range(obj.ReconfigElements.Reconfig_Cable_ID,:)';
                obj.ifc_reconfig_cable = obj.ReconfigElements.NeedReconfigure(obj.Reconfig_Cable_ID);
            else
                
                obj.VarRange = obj.ReconfigElements.Variable_Range;
            end
            
            
            %%%%%%%%%%%%%%%%%% handle this later %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            MRCI = obj.Reconfig_Cable_ID(find(obj.ifc_reconfig_cable));
%             remove_idx = [];
%             for i = 1:numel(MRCI)
%                 den = norm((OA_upper(:,MRCI(i)) - OA_lower(:,MRCI(i))));
%                 u_current = vecnorm((OA_current(:,MRCI(i)) - OA_lower(:,MRCI(i))))/den;
%                 ifc_range = interference_range.FeasibleInterval{MRCI(i)}{interference_range.ClosestReconfigureRange(:,MRCI(i))};
%                 if u_current > ifc_range(1)|| u_current < ifc_range(2)
%                     remove_idx = [remove_idx,i];
%                 end
%             end
%             MRCI(remove_idx) = [];
            
            MRCI = [obj.RequestCable,MRCI];
            
            
            
            %%%%%%%%%%%%%%%%%% Constraints Preparetion %%%%%%%%%%%%%%%%%%%%
            for CI = 1:numel(obj.ConstraintType)
                conditionType = obj.ConstraintType{CI};
                switch conditionType
                    case ReconfigurationConstraintType.WrenchClosure
                        
                        [Cable_Index,PerformanceIndex,sgn] = obj.CableCombination([],MRCI,[]);
                        
                        if PerformanceIndex ~= 0
                            [VariableRank,CableRank] = obj.RankVariableImportance(Cable_Index,PerformanceIndex,OA_0,[]);
                            
                        else
                            [VariableRank,CableRank] = obj.RankVariableImportance(obj.last_Cable_Index,PerformanceIndex,OA_0,[]);
                        end
                        
                        obj.HandleReconfigVariables(q,Cable_Index,CableRank,MRCI);
                        
                        POPConstraints = ReconfigureWrenchClosureConstraintGeneration(obj.Kinematic_Model,Cable_Index,sgn,obj.CableLength)
                        
                    case ReconfigurationConstraintType.WrenchFeasible
                        
%                         [Cable_Index,PerformanceIndex,sgn] = obj.CableCombination(obj.W_t,MRCI,[]);
%                         x = abs(PerformanceIndex);
                        A_eq = -obj.model.L';
                        b_eq = obj.W_t;
                        [n,m] = size(A_eq);
                        f = ones(1,m);
                        A_ieq = [];b_ieq = [];
                        ub = 1000000*ones(m,1);     lb = 0*ones(m,1);options = optimoptions('linprog','Display','none');
                        [x,~,~,~] = linprog(f',A_ieq,b_ieq,A_eq,b_eq,lb,ub,options);
                        PerformanceIndex = sum(x);
                        Cable_Index{1} = find(x~=0)'; Cable_Index{2} = [];
                        sgn = sign(det(A_eq(:,Cable_Index{1})));
                        if isempty(Cable_Index)
                            Cable_Index = obj.last_Cable_Index;
                        end
                        
                        
                        % tmp hack
%                         VariableRank = [1,2,3,4,7,8,5,6];
%                         CableRank = [1,2,3,4,7,8,5,6];
                        
                        if PerformanceIndex ~= 0
                            [VariableRank,CableRank] = obj.RankVariableImportance(Cable_Index,PerformanceIndex,OA_0,obj.W_t);
                            
                        else
                            [VariableRank,CableRank] = obj.RankVariableImportance(obj.last_Cable_Index,PerformanceIndex,OA_0,obj.W_t);
                        end
                        
                        obj.HandleReconfigVariables(q,Cable_Index,CableRank,MRCI);
                        
                        POPConstraints = ReconfigureWrenchFeasibleConstraintGeneration(obj.Kinematic_Model,Cable_Index,obj.W_t,obj.F_max,sgn,obj.CableLength);
                        
                    case ReconfigurationConstraintType.InterferenceFree
                        
                    otherwise
                        CASPR_log.Error('Constraint(s) not available');
                end
            end
            
            %%%%%%%%%%%%%%%%%% Objective Preparetion %%%%%%%%%%%%%%%%%%%%%%
            
            
            for CI = 1:numel(obj.ObjectiveType)
                objectiveType = obj.ObjectiveType{CI};
                switch objectiveType
                    case  ReconfigurationObjectiveType.TensionFactor
                        
                        POPObjective = ReconfigureTensionFactorConstraintGeneration(obj.Kinematic_Model,Cable_Index,sgn);
                        
                        break;
                    case ReconfigurationObjectiveType.CableForceSum
                        
                        POPObjective= ReconfigureMinimumCableForceConstraintGeneration(obj.Kinematic_Model,Cable_Index,obj.W_t,sgn,sum(x));
                        
                        break;
                    case ReconfigurationObjectiveType.ReconfigDist
                        
                    otherwise
                        CASPR_log.Error('Constraint(s) not available');
                end
            end
            
            Problem.Objective = POPObjective.Polynomials.Fun;
            Problem.Constraints.Inequalities = [POPConstraints.Polynomials.Inequalities,POPObjective.Polynomials.Inequalities];
            Problem.Constraints.Equalities = [POPObjective.Polynomials.Equalities,POPConstraints.Polynomials.Equalities];
            Problem.vars = obj.valid_variable;
            Problem.range = obj.VarRange;
            Problem.AuxVar = POPObjective.Polynomials.AuxVar;
            Problem.AuxVarRange = POPObjective.Polynomials.AuxVarRange;
            Problem.AuxVarRangeNum = POPObjective.Polynomials.AuxVarRangeNum;
            %             runFileName = POPGenerator(Problem,obj.AutoGenFilePath);
            [OptimalValue, status, obj.CompTime] = obj.Solver.solveProblem(Problem,obj.AutoGenFilePath,obj.RelaxOrder);
            
            if ~isempty(OptimalValue)
                obj.OptimalValue = obj.CurrentVar;
                for i = 1:numel(OptimalValue.VarName)
                    iVarIdx = find(strcmp(OptimalValue.VarName{i},string(obj.ReconfigElements.Unknown_var)));
                    if ~isempty(iVarIdx)
                        obj.OptimalValue(:,iVarIdx) = OptimalValue.value(i);
                    end
                end
                
                obj.last_Cable_Index = Cable_Index;
                obj.CurrentVar = obj.OptimalValue;
            else
                obj.OptimalValue = obj.CurrentVar;
            end
        end
        
        function HandleReconfigVariables(obj,q,Cable_Index,CableRank,MRCI)
            importance = unique([MRCI,CableRank],'stable'); % make the ifc cable most important one;
            if numel(importance) > obj.MaxReconfigNum
                chosen_cable = importance(1:obj.MaxReconfigNum);
            else
                chosen_cable = importance;
            end
            
            chosen_cable = chosen_cable(ismember(chosen_cable,cell2mat(Cable_Index)));
            if isempty(Cable_Index)
                Cable_Index = obj.last_Cable_Index;
            end
            
            
            ValidRangeInTime = [obj.CurrentVar - obj.MaximumReconfigStep; obj.CurrentVar + obj.MaximumReconfigStep];
            
            for IC = 1:size(obj.VarRange,2)
                obj.VarRange(:,IC) = IntervalIntersection(obj.VarRange(:,IC),ValidRangeInTime(:,IC))';
            end
            
            
            
            %                         valid_RCID = [obj.ReconfigElements.Info(ReconfigCableIndex).CableID];
            valid_RCID = chosen_cable;
            valid_RVID = [];
            for CI = 1:numel(valid_RCID)
                index = valid_RCID(CI) == [obj.ReconfigElements.Info.CableID];
                valid_RVID = [valid_RVID,obj.ReconfigElements.Info(index).VariableID];
            end
            fixedVarID = valid_RVID(~ismember(valid_RVID,1:numel(obj.ReconfigElements.Unknown_var)));
            allVarRange = obj.VarRange;
            allVarRange(:,fixedVarID) = obj.CurrentVar(fixedVarID);
            if numel(valid_RVID) > obj.MaxReconfigNum
                valid_RVID = valid_RVID(1:obj.MaxReconfigNum);
            end
            
            
            
            %                         obj.valid_variable = [obj.ReconfigElements.Info(ReconfigCableIndex).Variable];
            obj.valid_variable =  obj.ReconfigElements.Unknown_var(valid_RVID)';
            if numel(obj.valid_variable) > obj.MaxReconfigNum
                obj.valid_variable = obj.valid_variable(1:obj.MaxReconfigNum);
            end
            obj.VarRange = obj.VarRange(:,valid_RVID);
            
            C2VIdx = [];
            for CI = 1:numel(valid_RVID)
                for CJ = 1:numel(obj.ReconfigElements.Info)
                    if ismember(valid_RVID(CI),[obj.ReconfigElements.Info(CJ).VariableID])
                        C2VIdx(CI) = obj.ReconfigElements.Info(CJ).CableID;
                    end
                end
            end
            
            valid_RCID =      unique(C2VIdx);
            disp(['reconfig_variable: ',num2str(valid_RVID)]);
            disp(['reconfig_cid: ',num2str(valid_RCID)]);
            
                      
            ReconfigCableIndex = find(ismember(obj.ReconfigElements.Reconfig_Cable_ID,valid_RCID));
            CableLenghtVarID = ReconfigCableIndex;
            obj.Kinematic_Model.UpdateModel(q,obj.ReconfigElements.Reconfig_Cable_ID(ReconfigCableIndex));
            
            obj.Kinematic_Model.ReplaceNormByPolynomials(valid_RCID,obj.valid_variable,fixedVarID,allVarRange,obj.CurrentVar,C2VIdx)
            
            obj.valid_length_var = obj.ReconfigElements.Unknown_length;
            obj.valid_length_var = obj.valid_length_var(CableLenghtVarID,:)';
            obj.valid_length_var = obj.valid_length_var(:)';
            
            
            if obj.FixedCableLength 
                obj.CableLength = obj.model.cableLengths(valid_RCID);
            end
        end
        
        
        function [VariableRank,CableRank] = RankVariableImportance(obj,Cable_Index,ReferencePerformance,OA_0,Wrench)
            % test which cable affect the tf the most
            CableRank = [];
            for j = 1:numel(obj.ReconfigElements.Unknown_var)
                for i = 1:numel(obj.ReconfigElements.Info)
                    if any(obj.ReconfigElements.Info(i).VariableID == j)
                        PathID = i;
                    end
                end
                CableID =  obj.ReconfigElements.Info(PathID).CableID;
                CableRank = [CableRank,CableID];
                VarID = obj.ReconfigElements.Info(PathID).VariableID;
                numVar = numel(obj.ReconfigElements.Info(PathID).Variable);
                Varidx = obj.ReconfigElements.Info(PathID).VariableID;
                Var = obj.CurrentVar(Varidx);
                
                for i = 1:numVar
                    VariateVar = Var;
                    VariateVar(i) = VariateVar(i) - 0.02;
                    OA_1 = OA_0;
                    OA_1(:,CableID) = obj.ReconfigElements.Info(PathID).Path(VariateVar);
%                     OA_1(:,CableID) = obj.ReconfigElements.UpdateOA_i(PathID,VariateVar);
                    
                    ModelConfigUpdate(obj.model,OA_1,obj.model.q);
                    if isempty(Wrench)
                        var1 = rref([obj.model.L(Cable_Index{1},:);sum(obj.model.L(Cable_Index{2},:),1)]');
                        var2 = unique(abs(var1)); var2(var2 == 0) = [];
                        TmpPerformance(j) = min(var2)/max(var2);
                    else
                        A_eq = -obj.model.L';
                        b_eq = obj.W_t;
                        [n,m] = size(A_eq);
                        f = ones(1,m);
                        A_ieq = [];b_ieq = [];
                        ub = 1000000*ones(m,1);     lb = 0*ones(m,1);options = optimoptions('linprog','Display','none');
                        [x,~,~,~] = linprog(f',A_ieq,b_ieq,A_eq,b_eq,lb,ub,options);
                        TmpPerformance(j) = sum(x);
                        %                     L = -obj.model.L;
                        %                     var1 = rref([L(Cable_Index{1},:);sum(L(Cable_Index{2},:),1);-Wrench']');
                        %                     TmpPerformance(j) = sum(var1(:,end));
                    end
                    [~,~] = ModelConfigUpdate(obj.model,OA_0,obj.model.q);
                end
            end
            [~,importance] = sort(abs((TmpPerformance - ReferencePerformance)/ReferencePerformance),'descend');
            %             [~,importance] = sort(abs(TmpPerformance - ReferencePerformance),'descend');
            VariableRank = importance;
            %             CableRank    =
            CableRank =unique(CableRank(importance),'stable');
            %             ImportanceRank = [7,8,6,5];
            %             ImportanceRank = ImportanceRank(ismember(ImportanceRank, cell2mat(Cable_Index)));
            
        end
        
        function [Cable_Index,Performance_Index,wcc_sign] = CableCombination(obj,w,fixedCable,remove_ind)
            L = -obj.model.L';
            
            [n,m] = size(L);
            
            TF = [];
            cable_select = {};
            wcc_sign = [];
            count = 1;
            
            if ~isempty(w) % go for wrench feasible condition
                %                  if m - n == 2 % redundancy = 2
                cid  = 1:m;
                fix_col_id = nchoosek(cid,n-1);
                
                for i = 1:size(fix_col_id,1)
                    fix_col = L(:,fix_col_id(i,:));
                    remain_col_id = cid(~ismember(cid,fix_col_id(i,:)));
                    linear_comb_num = numel(remain_col_id);
                    for j = linear_comb_num:-1:1
                        lc_col_id = nchoosek(remain_col_id,j);
                        for k = 1:size(lc_col_id,1)
                            redundant_col_sum = sum(L(:,lc_col_id(k,:)),2);
                            L_bar = [fix_col,redundant_col_sum,-w];
                            L_rref = rref(L_bar);
                            c_end = L_rref(:,end);
                            if all(c_end<=0) && isequal(L_rref(1:n,1:n),eye(n))
                                %                                 TF = [TF;min(abs(c_end))/max(abs(c_end))];
                                TF = [TF;sum(c_end)];
                            else
                                TF = [TF;Inf];
                            end
                            %                         wcc_sign = [wcc_sign;-sign(det(L(1:n,1:n)))];
                            cable_select{count,1} = fix_col_id(i,:);
                            cable_select{count,2} = lc_col_id(k,:);
                            count  = count + 1;
                        end
                    end
                    
                end
            else % go for wrench closure condition
                
                cid  = 1:m;
                fix_col_id = nchoosek(cid,n);
                
                for i = 1:size(fix_col_id,1)
                    fix_col = L(:,fix_col_id(i,:));
                    remain_col_id = cid(~ismember(cid,fix_col_id(i,:)));
                    linear_comb_num = numel(remain_col_id);
                    for j = linear_comb_num:-1:1
                        lc_col_id = nchoosek(remain_col_id,j);
                        for k = 1:size(lc_col_id,1)
                            redundant_col_sum = sum(L(:,lc_col_id(k,:)),2);
                            
                            L_bar = [fix_col,redundant_col_sum];
                            L_rref = round(rref(L_bar),6);
                            c_end = L_rref(:,end); c_end(end+1) = -1;
                            if all(c_end<=0) && isequal(L_rref(1:n,1:n),eye(n))
                                TF = [TF;min(abs(c_end))/max(abs(c_end))];
                            else
                                TF = [TF;0];
                            end
                            %                         wcc_sign = [wcc_sign;-sign(det(L(1:n,1:n)))];
                            cable_select{count,1} = fix_col_id(i,:);
                            cable_select{count,2} = lc_col_id(k,:);
                            count  = count + 1;
                        end
                    end
                    
                end
                
            end
            %             wcc_sign(TF == 0,:) = [];
            cable_select(TF == 0,:) = []; cable_select(TF == Inf,:) = [];
            TF(TF == 0,:) = []; TF(TF == Inf,:) = [];
            [TF,ind] = sort(TF,'descend');
            cable_select = cable_select(ind,:);
            if isempty(cable_select)
                Cable_Index = [];
                Performance_Index = 0;
                wcc_sign = -1;
            else
                remove_row = [];
                if ~isempty(remove_ind)
                    for i = 1:numel(TF)
                        CableSet = cell2mat(cable_select(i,2));
                        if any(ismember(remove_ind,CableSet))
                            remove_row = [remove_row,i];
                        end
                    end
                    
                    TF(remove_row) = [];
                    cable_select(remove_row,:) = [];
                end
                
                
                if ~isempty(fixedCable)
                    for i = 1:numel(TF)
                        CableSet = cell2mat(cable_select(i,:));
                        if all(ismember(fixedCable,CableSet))
                            Cable_Index = cable_select(i,:);
                            Performance_Index = TF(i);
                            L_s = [L(:,Cable_Index{1,1}),sum(L(:,Cable_Index{1,2}),2)];
                            wcc_sign = sign(det(L_s(1:n,1:n)));
                            return;
                        elseif i == numel(TF)
                            error('No Reconfiguration Available')
                        end
                    end
                else
                    Cable_Index = cable_select(1,:);
                    Performance_Index = TF(1);
                    L_s = [L(:,Cable_Index{1,1}),sum(L(:,Cable_Index{1,2}),2)];
                    wcc_sign = sign(det(L_s(1:n,1:n)));
                end
            end
            
        end
        
    end
end