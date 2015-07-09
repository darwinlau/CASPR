classdef BodyDynamics < handle
    %BODYKINEMATICS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        m                       % Mass of body
        I_G                     % Inertia of body about its centre of mass
    end
    
    properties (SetAccess = private)
        id                      % Body ID
    end
   
    methods
        function bk = BodyDynamics(id)
            bk.id = id;
        end
    end
end

