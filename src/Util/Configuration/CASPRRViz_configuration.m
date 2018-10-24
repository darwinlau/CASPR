% Facilities for handling CASPR-RViz configurations in MATLAB.
%
% Author        : Dominic Chan
% Created       : 2018
% Description   : A class for loading and saving to CASPR-RViz configurations
classdef CASPRRViz_configuration
    methods (Static)
        % Load ROS_MASTER_URI
        function ROS_MASTER_URI = LoadROS_MASTER_URI()
            try
                config = load('CASPRRVizConfig.mat', 'ROS_MASTER_URI');
            catch
                CASPR_log.Error('Config does not exist. Please create config by CASPRRViz_configuration.SetROSConfig([caspr-rviz IP],[caspr IP]).');
            end
            ROS_MASTER_URI = config.ROS_MASTER_URI;            
        end
        % Load ROS_IP
        function ROS_IP = LoadROS_IP()
            try
                config = load('CASPRRVizConfig.mat', 'ROS_IP');
            catch
                CASPR_log.Error('Config does not exist. Please create config by CASPRRViz_configuration.SetROSConfig([caspr-rviz IP],[caspr IP]).');
            end
            ROS_IP = config.ROS_IP;            
        end
        % Set the compiled_mode flag
        function SetROSConfig(caspr_rviz_ip, caspr_ip) 
            ROS_MASTER_URI = sprintf('http://%s:11311', caspr_rviz_ip);
            ROS_IP = caspr_ip;
            home_path = CASPR_configuration.LoadHomePath();
            save([home_path,'/data/config/CASPRRVizConfig.mat'], 'ROS_MASTER_URI', 'ROS_IP');    
            CASPR_log.Info(sprintf('Set ROS_MASTER_URI: %s', ROS_MASTER_URI));
            CASPR_log.Info(sprintf('Set ROS_IP: %s', ROS_IP));
        end        
    end
end
