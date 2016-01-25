function [c1,c2] = Centroid(Vertices)
% Calculate the Center of the ploygon
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
