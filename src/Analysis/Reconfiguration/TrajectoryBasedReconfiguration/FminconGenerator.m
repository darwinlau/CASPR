function runFileName = FminconGenerator(Problem,FilePath)

vars = [Problem.vars,Problem.AuxVar];
solutionVars = Problem.vars;


runFileName{1} = [FilePath 'FminconConstraints.m'];
fid = fopen(runFileName{1},'wt');

printtext =['function [c,ceq] = FminconConstraints(x)\n'];
fprintf(fid,printtext);

for i = 1:numel(vars)
printtext =[char(vars(i)), ' = x(', num2str(i),');\n'];
fprintf(fid,printtext);

end
count = 1;
for i = 1:numel(Problem.Constraints.Inequalities)
   Constraints = char(Problem.Constraints.Inequalities(i));
   Constraints = extractBefore(Constraints," <=");
   printtext =['c(', num2str(count), ') =' Constraints,';\n'];
   fprintf(fid,printtext);
   count = count + 1;
end

printtext =['ceq = [];\n'];
fprintf(fid,printtext);
count = 1;
for i = 1:numel(Problem.Constraints.Equalities)
   Constraints = char(Problem.Constraints.Equalities(i));
   Constraints = extractBefore(Constraints," ==");
   printtext =['ceq(', num2str(count), ') =' Constraints,';\n'];
   fprintf(fid,printtext);
   count = count + 1;
end



fclose(fid);


%% generate objective function
runFileName{2} = [FilePath 'FminconObjectiveFun.m'];
fid = fopen(runFileName{2},'wt');

printtext =['function out = FminconObjectiveFun(x)\n'];
fprintf(fid,printtext);

for i = 1:numel(vars)
printtext =[char(vars(i)), ' = x(', num2str(i),');\n'];
fprintf(fid,printtext);

end

printtext =['out = ',char(Problem.Objective),';\n'];
fprintf(fid,printtext);

fclose(fid);
%% generate main call function
runFileName{3} = [FilePath 'Reconfig_AutoGenProblem.m'];
fid = fopen(runFileName{3},'wt');

%% generate boundaries
count = 1;
for i = 1:numel(solutionVars)
    printtext = ['lb(', num2str(count), ')= ', num2str(Problem.range(1,i)), ';\n'];
    fprintf(fid,printtext);
    printtext = ['ub(', num2str(count), ')= ', num2str(Problem.range(2,i)), ';\n'];
    fprintf(fid,printtext);
    count = count + 1;
end

for i = 1:size(Problem.AuxVarRangeNum,2)

    printtext = ['lb(', num2str(count), ')= ', num2str(Problem.AuxVarRangeNum(1,i)), ';\n'];
    fprintf(fid,printtext);
   
    printtext = ['ub(', num2str(count), ')= ', num2str(Problem.AuxVarRangeNum(2,i)), ';\n'];
    fprintf(fid,printtext);

end

printtext = ['A = [];\n'];
fprintf(fid,printtext);
printtext = ['Aeq = [];\n'];
fprintf(fid,printtext);
printtext = ['b = [];\n'];
fprintf(fid,printtext);
printtext = ['beq = [];\n'];
fprintf(fid,printtext);    
fclose(fid);




% %% get soltuion files
% runFileName{2} = [FilePath 'Reconfig_GetSolution.m'];
% ans_range = 1:numel(solutionVars);
% fid = fopen(runFileName{2},'wt');
% 
% printtext = ['global MMM;\n',...
%              'X_opt.value = MMM.M{1,1}.sol;\n' ,...  
%              'X_opt.VarName = MMM.var;\n'];
% % printtext = ['X_opt = y([',num2str(ans_range), '],:);\n'];
% fprintf(fid,printtext);
% 
% % printtext = ['delete '];
% % fprintf(fid,printtext);
% % for i = 1:numel(vars)
% %     printtext = [char(vars(i)) ' '];
% %     fprintf(fid,printtext);
% % end
% 
% 
% 
% 
% % printtext = ['X_opt = double([',...
% %     strjoin(arrayfun(@char, solutionVars, 'uniform', 0)),...
% %     ']);'];
% % fprintf(fid,printtext);
% 
% fclose(fid);


end