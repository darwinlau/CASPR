% function that converts a 6-D spatial vector into a 6-by-6 matrix which is
% a cross product projector for a force type spatial vector
function res = vecX6DF(vec)
    res = [ vecX3D(vec(1:3))    vecX3D(vec(4:6));
            zeros(3)            vecX3D(vec(1:3))];
end