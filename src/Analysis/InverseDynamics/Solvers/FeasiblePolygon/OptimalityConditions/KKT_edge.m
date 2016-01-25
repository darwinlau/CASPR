function [Cond,Opt] = KKT_edge(i,N,bi,max,min,t_sol)
% 2-Norm Opt, KKT condition for edges
% Cond = 0(Not optimal) or 1(Optimal)
ni = N(i,:);
if bi == 1,
    Li = max(i);
else
    Li = min(i);
end
ai = -bi * ni';
ui = -bi * (Li - t_sol(i))/(ai' *ai);
Opt = ui * ai;
Indexset = [];
for index = 1:length(N)
    if min(index) - t_sol(index) <= N(index,:) * Opt && N(index,:) * Opt <= max(index) - t_sol(index)
        Indexset = [Indexset,index];
    end
end
if ui>=0 && length(Indexset) == length(N)
    Cond = 1;
else
    Cond = 0;
end
end

