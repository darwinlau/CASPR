% Inertia uncertainties with uniform probability distribution 
%
% Author        : Jonathan EDEN
% Created       : 2017
% Description    :
%    Inertia uncertainties with a uniform probability distribution.
classdef InertiaUncertaintyUniform < InertiaUncertaintyBase
    properties
        m_uncertainty_range % A vector which defines the upper and lower bounds on mass
        I_uncertainty_range % A vector which defines the upper and lower bounds on Inertia (I_G)
    end
    
    methods
        % Constructor
        function iu = InertiaUncertaintyUniform(m_uncertainty_range,I_uncertainty_range)
            iu.m_uncertainty_range = m_uncertainty_range;
            % I input is optional
            if(nargin == 1)
                iu.I_uncertainty_range = [];
            else
                iu.I_uncertainty_range = I_uncertainty_range;
            end
        end
    end
    
    methods(Access=protected)
        % Implementation of uncertainty function
        function im = applyInertiaUncertainty(obj,sm)
            im = sm.bodyModel.massInertiaMatrix;
            for k = 1:sm.numLinks
                m_adjust = -obj.m_uncertainty_range(2*(k-1)+1) + rand*(obj.m_uncertainty_range(2*(k-1)+2) + obj.m_uncertainty_range(2*(k-1)+1));
                if(~isempty(obj.I_uncertainty_range))
                    Igxx_adjust = -obj.I_uncertainty_range(12*(k-1)+1) + rand*(obj.I_uncertainty_range(12*(k-1)+2)+obj.I_uncertainty_range(12*(k-1)+1));
                    Igxy_adjust = -obj.I_uncertainty_range(12*(k-1)+3) + rand*(obj.I_uncertainty_range(12*(k-1)+4)+obj.I_uncertainty_range(12*(k-1)+3));
                    Igxz_adjust = -obj.I_uncertainty_range(12*(k-1)+5) + rand*(obj.I_uncertainty_range(12*(k-1)+5)+obj.I_uncertainty_range(12*(k-1)+5));
                    Igyy_adjust = -obj.I_uncertainty_range(12*(k-1)+7) + rand*(obj.I_uncertainty_range(12*(k-1)+8)+obj.I_uncertainty_range(12*(k-1)+7));
                    Igyz_adjust = -obj.I_uncertainty_range(12*(k-1)+9) + rand*(obj.I_uncertainty_range(12*(k-1)+10)+obj.I_uncertainty_range(12*(k-1)+9));
                    Igzz_adjust = -obj.I_uncertainty_range(12*(k-1)+11) + rand*(obj.I_uncertainty_range(12*(k-1)+12)+obj.I_uncertainty_range(12*(k-1)+11));
                    I_G_offset = [Igxx_adjust,Igxy_adjust,Igxz_adjust;Igxy_adjust,Igyy_adjust,Igyz_adjust;Igxz_adjust,Igyz_adjust,Igzz_adjust];
                    im(6*k-5:6*k, 6*k-5:6*k) = im(6*k-5:6*k, 6*k-5:6*k) + [m_adjust*eye(3),zeros(3);zeros(3),I_G_offset];
                else
                    im(6*k-5:6*k, 6*k-5:6*k) = im(6*k-5:6*k-3, 6*k-5:6*k-3) + m_adjust*eye(3);
                end                
            end
        end
    end
end

