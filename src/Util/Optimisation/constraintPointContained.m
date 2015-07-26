function [c,ceq] = constraintPointContained(x,A,b,x_ref,buffer)
    q = size(A,1); r = size(A,2);
    A = [A,zeros(q,1)];
    for i =1:q
        A(i,r+1) = norm(A(i,:));
    end
    ra = zeros(q,1);
    for k =1:q
        ra(k) = (1/A(k,r+1))*(b(k) - A(k,1:r)*x);
    end
    r = -min(ra);
    % Compute nonlinear inequality
    c = [(x-x_ref)',r]*diag([1,1,-1])*[(x-x_ref);r] + buffer; 
    ceq = 0;   % Compute nonlinear equalities at x.
end