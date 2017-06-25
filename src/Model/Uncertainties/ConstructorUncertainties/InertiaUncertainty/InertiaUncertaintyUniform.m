% Inertia uncertainties with uniform probability distribution 
%
% Author        : Jonathan EDEN
% Created       : 2017
% Description    :
%    Inertia uncertainties with a uniform probability distribution.
classdef InertiaUncertaintyUniform < InertiaUncertaintyBase
    properties
        m_uncertainty_range % A vector which defines the upper and lower bounds on mass
        r_G_uncertainty_range % A vector which defines the upper and lower bounds on r_G
        I_uncertainty_range % A vector which defines the upper and lower bounds on Inertia (I_G)
    end
    
    methods
        % Constructor
        function iu = InertiaUncertaintyUniform(m_uncertainty_range,r_G_uncertainty_range,I_uncertainty_range)
            iu.m_uncertainty_range = m_uncertainty_range;
            % I input is optional
            if(nargin == 1)
                iu.r_G_uncertainty_range = [];
                iu.I_uncertainty_range = [];
            elseif(nargin == 2)
                iu.r_G_uncertainty_range = r_G_uncertainty_range;
                iu.I_uncertainty_range = [];
            else
                iu.r_G_uncertainty_range = r_G_uncertainty_range;
                iu.I_uncertainty_range = I_uncertainty_range;
            end
        end
    end
    
    methods(Access=protected)
        % Implementation of uncertainty function
        function im = applyInertiaUncertainty(obj,sm)
            for k = 1:sm.numLinks
                im.m{k} = -obj.m_uncertainty_range(2*(k-1)+1) + rand*(obj.m_uncertainty_range(2*(k-1)+2) + obj.m_uncertainty_range(2*(k-1)+1));
                if(~isempty(obj.r_G_uncertainty_range))
                    r_G_offset = -obj.r_G_uncertainty_range(6*(k-1)+1:2:6*(k-1)+5) + rand*(obj.r_G_uncertainty_range(6*(k-1)+2:2:6*(k-1)+6)+obj.r_G_uncertainty_range(6*(k-1)+1:2:6*(k-1)+5));
                    im.r_G{k} = r_G_offset;
                elseif(~isempty(obj.I_uncertainty_range))
                    Igxx_adjust = -obj.I_uncertainty_range(12*(k-1)+1) + rand*(obj.I_uncertainty_range(12*(k-1)+2)+obj.I_uncertainty_range(12*(k-1)+1));
                    Igxy_adjust = -obj.I_uncertainty_range(12*(k-1)+3) + rand*(obj.I_uncertainty_range(12*(k-1)+4)+obj.I_uncertainty_range(12*(k-1)+3));
                    Igxz_adjust = -obj.I_uncertainty_range(12*(k-1)+5) + rand*(obj.I_uncertainty_range(12*(k-1)+5)+obj.I_uncertainty_range(12*(k-1)+5));
                    Igyy_adjust = -obj.I_uncertainty_range(12*(k-1)+7) + rand*(obj.I_uncertainty_range(12*(k-1)+8)+obj.I_uncertainty_range(12*(k-1)+7));
                    Igyz_adjust = -obj.I_uncertainty_range(12*(k-1)+9) + rand*(obj.I_uncertainty_range(12*(k-1)+10)+obj.I_uncertainty_range(12*(k-1)+9));
                    Igzz_adjust = -obj.I_uncertainty_range(12*(k-1)+11) + rand*(obj.I_uncertainty_range(12*(k-1)+12)+obj.I_uncertainty_range(12*(k-1)+11));
                    I_G_offset = [Igxx_adjust,Igxy_adjust,Igxz_adjust;Igxy_adjust,Igyy_adjust,Igyz_adjust;Igxz_adjust,Igyz_adjust,Igzz_adjust];
                    im.I_G{k} = I_G_offset;
                end                
            end
        end
    end
end

