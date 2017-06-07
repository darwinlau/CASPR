% Implementation of round function for decimals to be used for old versions
% of MATLAB (prior to 2014b)
%
% Author        : Darwin LAU
% Created       : 2017
% Description   : 
function [ out ] = round( varargin )
    % Version 8.4 is 2014b
    if (verLessThan('matlab', '8.4'))
        out = 10.^(-n) .* builtin('round', x/10.^(-n));
    else
        out = builtin('round', varargin{:});
    end
end

