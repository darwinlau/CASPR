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
        function applyConstructorUncertainty(obj, system_model)
            sm = system_model;
                
                % TODO: add check to see if model is in compute dynamics
                % mode, otherwise this uncertainty is useless
%             if(sm.modelOptions.isComputeDynamics)
%                 sm.bodyModel.createMassInertiaMatrix();
%             end
            im = obj.applyInertiaUncertainty(sm);
            sm.updateInertiaProperties(im.m, im.r_G, im.I_G);
        end
    end
    methods(Abstract,Access=protected)
        % Abstract method for applying uncertainties to inertia
        im = applyInertiaUncertainty(obj,system_model);
    end
end

