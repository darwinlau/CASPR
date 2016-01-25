function [ni_prepend] = Cal_ni(N,i,j,bj)
% Calculate the directional vector
ni_prepend = [N(i,2) -N(i,1)]'; % Moving direction
if bj==-1
    if N(j,:) * ni_prepend <0
        ni_prepend = [-N(i,2) N(i,1)]'; % Moving direction
    end
elseif bj==1
    if N(j,:) * ni_prepend >0
        ni_prepend = [-N(i,2) N(i,1)]'; % Moving direction
    end     
end
end

