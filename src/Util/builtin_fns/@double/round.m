% Implementation of round function for decimals to be used for old versions
% of MATLAB (prior to 2014b)
%
% Author        : Darwin LAU
% Created       : 2017
% Description   : 
function [ out ] = round( varargin )
    % Version 8.4 is 2014b
    if (verLessThan('matlab', '8.4'))
        if (nargin == 2)
            x = varargin{1};
            n = varargin{2};
            out = 10.^(-n) .* builtin('round', x/10.^(-n));
        else
            out = builtin('round', varargin{:});
        end
    else
        out = builtin('round', varargin{:});
    end
end

