% function that takes a translation vector from frame A to B and gives back a
% spatial vector transformation from the corresponding 6-D space A to B
% that is a direct result of the translation represented by r
function res = xltMMat(r)
    res = [ eye(3)    	zeros(3);
            -vecX3D(r)  eye(3)];
end