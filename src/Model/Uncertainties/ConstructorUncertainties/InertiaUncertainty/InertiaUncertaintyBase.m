% Base class for inertia uncertainties 
%
% Author        : Jonathan EDEN
% Created       : 2017
% Description    :
%    Base class for inertia uncertainties. These uncertainties can be 
%    considered at construction.
classdef (Abstract) InertiaUncertaintyBase < ConstructorUncertaintyBase
    methods
        % Implementation of applyConstructorUncertainty
        function applyConstructorUncertainty(obj,system_model)
            sm = system_model;
            % Generate the dynamic component of the model if needed to
            if(~sm.bodyModel.occupied.dynamics)
                sm.bodyModel.createMassInertiaMatrix();
                sm.bodyModel.occupied.dynamics = true;
                sm.bodyModel.updateDynamics();
            end
            im = obj.applyInertiaUncertainty(sm);
            sm.updateInertiaProperties(im.m,im.r_G,im.I_G,1);
        end
    end
    methods(Abstract,Access=protected)
        % Abstract method for applying uncertainties to inertia
        im = applyInertiaUncertainty(obj,system_model);
    end
end

