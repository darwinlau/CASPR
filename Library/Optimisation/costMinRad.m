function f = costMinRad(x,A,b)
    q = size(A,1); r = size(A,2);
    A = [A,zeros(q,1)];
    for i =1:q
        A(i,r+1) = norm(A(i,:));
    end
    ra = zeros(q,1);
    for k =1:q
        ra(k) = (1/A(k,r+1))*(b(k) - A(k,1:r)*x);
    end
    f = -min(ra);
end