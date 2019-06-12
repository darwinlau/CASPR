% function that takes a translation vector from frame A to B and gives back a
% spatial vector transformation from the corresponding 6-D space A to B
% that is a direct result of the translation represented by r
% this is for force type vector's transformation
function res = xltFMat(r)
    res = [ eye(3)      -vecX3D(r);
            zeros(3)    eye(3)];
end