% Calculate the Center of the ploygon
%
% Please cite the following paper when using this algorithm:
% M. Gouttefarde, J. Lamaury, C. Reichert and T. Bruckmann, "A Versatile
% Tension Distribution Algorithm for n-DOF Parallel Robots Driven by n+2
% Cables", IEEE Trans. Robot., vol. 31, no. 6, pp. 1444-1457, 2015.
%
% Author        : Jihong ZHU
% Created       : 2016
function [c1,c2] = Centroid(Vertices)
temp = 0;
for i = 1:(length(Vertices)-1),
    temp = temp + (Vertices(1,i)*Vertices(2,i+1) - Vertices(1,i+1)*Vertices(2,i));
end
A = temp/2;
temp_c1 = 0;
for i = 1:(length(Vertices)-1),
    temp_c1 = temp_c1 + (Vertices(1,i) + Vertices(1,i+1)) * (Vertices(1,i)*Vertices(2,i+1) - Vertices(1,i+1)*Vertices(2,i));
end
temp_c2 = 0;
for i = 1:(length(Vertices)-1),
    temp_c2 = temp_c2 + (Vertices(2,i) + Vertices(2,i+1)) * (Vertices(1,i)*Vertices(2,i+1) - Vertices(1,i+1)*Vertices(2,i));
end
c1 = temp_c1/(6*A);
c2 = temp_c2/(6*A);
end
