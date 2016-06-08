% Calculate the directional vector
%
% Please cite the following paper when using this algorithm:
% M. Gouttefarde, J. Lamaury, C. Reichert and T. Bruckmann, "A Versatile
% Tension Distribution Algorithm for n-DOF Parallel Robots Driven by n+2
% Cables", IEEE Trans. Robot., vol. 31, no. 6, pp. 1444-1457, 2015.
%
% Author        : Jihong ZHU
% Created       : 2016
function [ni_prepend] = Cal_ni(N,i,j,bj)
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

