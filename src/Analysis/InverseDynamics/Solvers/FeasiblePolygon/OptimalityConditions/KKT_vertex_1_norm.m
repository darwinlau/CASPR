function Cond = KKT_vertex_1_norm(N,i,j,bi,bj)
% 2-Norm Opt, KKT condition for a vertex
% Cond = 0(Not optimal) or 1(Optimal)
ni = N(i,:);
nj = N(j,:);
ai = -bi * ni';
aj = -bj * nj';
Aij = [ai,aj];
mu = Aij\(N'*ones(length(N),1));
if mu(1)>=0 && mu(2)>=0
    Cond = 1;
else 
    Cond = 0;
end

end