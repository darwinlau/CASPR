function runFileName = POPGenerator(Problem,FilePath)
runFileName{1} = [FilePath 'Reconfig_AutoGenProblem.m'];
fid = fopen(runFileName{1},'wt');


vars = [Problem.vars,Problem.AuxVar];
solutionVars = Problem.vars;

%% generate variables
for i = 1:numel(vars)
    printtext = ['mpol ', char(vars(i)),';\n'];
    fprintf(fid,printtext);
end

%% generate objective function
printtext =['ObjectiveFunction = ', char(Problem.Objective), ';\n'];
fprintf(fid,printtext);

%% generate equalities constraints
count = 1;
for i = 1:numel(Problem.Constraints.Equalities)
    printtext =['ConstraintFunctions(', num2str(count), ',:)= ', char(Problem.Constraints.Equalities(i)) ';\n'];
    fprintf(fid,printtext);
    count = count + 1;
end
%% generate inequalities constraints
for i = 1:numel(Problem.Constraints.Inequalities)
    printtext =['ConstraintFunctions(', num2str(count), ',:)= ', char(Problem.Constraints.Inequalities(i)) ';\n'];
    fprintf(fid,printtext);
    count = count + 1;
end

%% generate Aux variable Range
for i = 1:numel(Problem.AuxVarRange)
    printtext =['ConstraintFunctions(', num2str(count), ',:)= ', char(Problem.AuxVarRange(i)) ';\n'];
    fprintf(fid,printtext);
    count = count + 1;
end

%% generate boundaries
for i = 1:numel(solutionVars)
    printtext = ['ConstraintFunctions(', num2str(count), ',:)= ', ...
                   char(solutionVars(i)),' >= ',num2str(Problem.range(1,i)), ';\n',...
                 'ConstraintFunctions(', num2str(count+1), ',:)= ', ...
                    char(solutionVars(i)),' <= ',num2str(Problem.range(2,i)), ';\n'];
    fprintf(fid,printtext);
    count = count + 2;
end


fclose(fid);



%% get soltuion files
runFileName{2} = [FilePath 'Reconfig_GetSolution.m'];
ans_range = 1:numel(solutionVars);
fid = fopen(runFileName{2},'wt');

printtext = ['global MMM;\n',...
             'X_opt.value = MMM.M{1,1}.sol;\n' ,...  
             'X_opt.VarName = MMM.var;\n'];
% printtext = ['X_opt = y([',num2str(ans_range), '],:);\n'];
fprintf(fid,printtext);

% printtext = ['delete '];
% fprintf(fid,printtext);
% for i = 1:numel(vars)
%     printtext = [char(vars(i)) ' '];
%     fprintf(fid,printtext);
% end




% printtext = ['X_opt = double([',...
%     strjoin(arrayfun(@char, solutionVars, 'uniform', 0)),...
%     ']);'];
% fprintf(fid,printtext);

fclose(fid);


end