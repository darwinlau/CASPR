% Constant inertia uncertainties 
%
% Author        : Chen SONG
% Created       : 2017
% Description    :
%    Constant inertia uncertainties .
classdef InertiaUncertaintyConstant < InertiaUncertaintyBase
    properties
        m_uncertainty               % A vector which defines the upper and lower bounds on mass
        r_G_uncertainty             % A vector which defines the upper and lower bounds on r_G
        I_uncertainty               % A vector which defines the upper and lower bounds on Inertia (I_G)
        use_relative_uncertainty    % the input uncertainty is relative if this is true
    end
    
    methods
        % Constructor
        function iu = InertiaUncertaintyConstant(m_uncertainty,r_G_uncertainty,I_uncertainty, use_relative_uncertainty)
            iu.m_uncertainty = m_uncertainty;
            % I input is optional
            if(nargin == 1)
                iu.r_G_uncertainty = [];
                iu.I_uncertainty = [];
                iu.use_relative_uncertainty = true;
            elseif(nargin == 2)
                iu.r_G_uncertainty = r_G_uncertainty;
                iu.I_uncertainty = [];
                iu.use_relative_uncertainty = true;
            elseif (nargin == 3)
                iu.r_G_uncertainty = r_G_uncertainty;
                iu.I_uncertainty = I_uncertainty;
                iu.use_relative_uncertainty = true;
            else
                iu.r_G_uncertainty = r_G_uncertainty;
                iu.I_uncertainty = I_uncertainty;
                iu.use_relative_uncertainty = use_relative_uncertainty;
            end
        end
    end
    
    methods(Access=protected)
        % Implementation of uncertainty function
        function im = applyInertiaUncertainty(obj,sm)
            for k = 1:sm.numLinks
                im.m{k} = sm.bodyModel.bodies{k}.m;
                im.r_G{k} = sm.bodyModel.bodies{k}.r_G;
                im.I_G{k} = sm.bodyModel.bodies{k}.I_G;
                if (obj.use_relative_uncertainty)
                    im.m{k} = sm.bodyModel.bodies{k}.m .* (ones(size(obj.m_uncertainty(k))) + obj.m_uncertainty(k));
                else
                    im.m{k} = sm.bodyModel.bodies{k}.m + obj.m_uncertainty(k);
                end
                if(~isempty(obj.r_G_uncertainty))
                    r_G_offset = obj.r_G_uncertainty(3*(k-1)+1:3*k);
                    if (obj.use_relative_uncertainty)
                        im.r_G{k} = sm.bodyModel.bodies{k}.r_G .* (ones(size(r_G_offset)) + r_G_offset);
                    else
                        im.r_G{k} = sm.bodyModel.bodies{k}.r_G + r_G_offset;
                    end
                elseif(~isempty(obj.I_uncertainty))
                    Igxx_adjust = obj.I_uncertainty(6*(k-1)+1);
                    Igxy_adjust = obj.I_uncertainty(6*(k-1)+2);
                    Igxz_adjust = obj.I_uncertainty(6*(k-1)+3);
                    Igyy_adjust = obj.I_uncertainty(6*(k-1)+4);
                    Igyz_adjust = obj.I_uncertainty(6*(k-1)+5);
                    Igzz_adjust = obj.I_uncertainty(6*(k-1)+6);
                    I_G_offset = [Igxx_adjust,Igxy_adjust,Igxz_adjust;Igxy_adjust,Igyy_adjust,Igyz_adjust;Igxz_adjust,Igyz_adjust,Igzz_adjust];
                    if (obj.use_relative_uncertainty)
                        im.I_G{k} = sm.bodyModel.bodies{k}.I_G .* (ones(size(I_G_offset)) + I_G_offset);
                    else
                        im.I_G{k} = sm.bodyModel.bodies{k}.I_G + I_G_offset;
                    end
                end                
            end
        end
    end
end

