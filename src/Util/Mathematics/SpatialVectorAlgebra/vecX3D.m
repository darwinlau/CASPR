% function that converts a 3-D vector into a 3-by-3 matrix which is a cross
% product projector
function res = vecX3D(vec)
    res = [ 0       -vec(3) vec(2);
            vec(3)  0       -vec(1);
            -vec(2) vec(1)  0];
end