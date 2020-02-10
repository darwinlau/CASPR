% Object representation of the wrench-set
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    :
classdef Zonotope < ConvexPolytope
        
    properties (SetAccess = protected)
    end
    
    methods        
        % Builds the zonotope using the jacobian and force bounds. This
        % version does not make an external system call.
        % The system model is defined as:
        %   w = L * f + offset
        % Note: As is the structure matrix
        function ws = Zonotope(L, F_u, F_l, offset)
            % n refers to the degree of freedom (dimension of wrench)
            n = size(L,1); 
            % m refers to the number of actuators
            m = size(L,2);
            
            if (nargin < 4 || isempty(offset))
                offset = zeros(n, 1);
            end
            n_points = 2^m;
            F = zeros(n_points, m);
            W = zeros(n_points, n);
            for k=1:n_points
                % Convert k to binary (the double function turns the number
                % to 48 or 49 depending if it is a 0 or 1, respectively
                beta = double(dec2bin(k-1,m)) - 48;
                F(k,:) = (eye(m) - diag(beta))*F_l + diag(beta)*F_u;
                W(k,:) = L*F(k,:)' + offset;
            end
            ws@ConvexPolytope(W');
        end
    end
end