% Base class for uncertainties associated with external wrench disturbances
%
% Author        : Chen SONG
% Created       : 2017
% Description    :
%    Base class for uncertainties associated with external wrench disturbances
classdef (Abstract) ExternalWrenchUncertaintyBase < PreUpdateUncertaintyBase
    properties
        model                       % Model object
        flag_apply_disturbance      % If the disturbance definition checked out, disturbance will be applied
    end
    
    methods
        
        function ipu = ExternalWrenchUncertaintyBase(model) 
            ipu.model           =   model;
        end
        
    end
    methods(Abstract)
        % Apply external wrench disturbance before the update
        [w_ext] = applyWrechDisturbance(obj, t);
        % do frequency domain analysis
        % extract frequency domain analysis
    end
end

