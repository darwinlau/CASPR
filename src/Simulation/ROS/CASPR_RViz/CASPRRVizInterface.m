% Class for interacting with CASPR-RViz through ROS
%
% Author        : Dominic Chan
% Created       : 2018
% Description   :
%    This class enables visualization of cable robots in RViz by
%    interacting with CASPR-RViz
% CASPR-RViz    : https://github.com/darwinlau/CASPR-RViz

classdef CASPRRVizInterface < CableActuatorInterfaceBase
    properties (Access = private)        
        ROS_MASTER_URI;             % URI or the device running master
        ROS_IP;                     % IP of the local device        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Visualization
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        link_name_pub;              % Publisher for link names             
        link_name_msg;              % Message type for link names                   
        link_tf_pub;                % Publisher for link tf
        link_tf_msg;                % Message type for link tf
        cable_pub;                  % Publisher for cables
        cable_msg;                  % Message type for cables
        ee_pub;                     % Publisher for end-effector
        ee_msg;                     % Message type for end-effector
        f_pub;                      % Publisher for cable forces
        f_msg;                      % Message type for cable forces
    end
   
    properties (Access = private, Constant = true)  
        % Node name
        nodeName = '/CASPR_MATLAB';
           
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Visualization
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Link names
        link_name_topic = '/link_name';
        link_name_msg_type = 'std_msgs/String';
        % Link tf matrix
        link_tf_topic = '/link_tf';   
        link_tf_msg_type = 'std_msgs/Float32MultiArray';
        % Cable attachment vector
        cable_topic = '/cable';   
        cable_msg_type = 'std_msgs/Float32MultiArray';
        % End-effector pos
        ee_topic = '/ee';   
        ee_msg_type = 'std_msgs/Float32MultiArray';
        % Cable Force
        f_topic = '/force';
        f_msg_type = 'std_msgs/Float32MultiArray';
    end
    
    methods (Access = public)
        % Constructor
        function interface = CASPRRVizInterface()
            interface@CableActuatorInterfaceBase();
            interface.ROS_MASTER_URI = CASPRRViz_configuration.LoadROS_MASTER_URI();
            interface.ROS_IP = CASPRRViz_configuration.LoadROS_IP();      
            interface.initialise();
        end
    
        % Initialising is the same as creating a new ROS node
        function initialise(obj)               
            obj.open();
        end
        
        % Opening means creating a new ROS node
        function open(obj)  
            rosshutdown
            % connect to the master through the master ip
            try       
                rosinit(obj.ROS_MASTER_URI, 'NodeHost', obj.ROS_IP, ...
                    'NodeName', obj.nodeName);                     
            catch
                CASPR_log.Error('Failed to initialize ROS!');
            end           
            % create publisher & subscriber
            obj.createPublisher();
            obj.createSubscriber();
        end
        
        % shut down ros
        function close(obj)
            rosshutdown;
        end
       
        % detect whether the ros node is avaible on the master device
        function [success] =detectDevice(obj)
            % Try to get node info to check if the node is available
            try 
                % node info
                rosinfo = rosnode('info', obj.nodeName); 
                success = true;
            catch
                success = false;
            end                  
        end        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Visualization
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%         
        function visualize(obj, cdpr, f)            
            obj.linkInfoSend(cdpr);
            obj.cableInfoSend(cdpr);
            if ~isempty(cdpr.y)
                obj.eeInfoSend(cdpr);
            end          
            if nargin > 2
                CASPR_log.Assert(cdpr.numCables==length(f), ...
                    'Size of force vectors does not match the number of cables of the cdpr.');
                obj.forceInfoSend(cdpr, f);
            end           
        end
        
        function createStaticObj(obj, name, pos, quat)
            % Send object name
            obj.link_name_msg.Data = strcat(name, ',;/');            
            % Send object tfs
            tf = [pos(1); pos(2); pos(3); quat.q1; ...
                quat.q2; quat.q3; quat.q0];
            obj.link_tf_msg.Data = tf;            
            send(obj.link_name_pub, obj.link_name_msg);
            send(obj.link_tf_pub, obj.link_tf_msg);            
            pause(0.2);
        end
        
        % publish tf of links
        function linkInfoSend(obj, cdpr)
            % Send link names
            link_names = '';
            for i = 1:cdpr.numLinks                
                link_names = strcat(link_names, cdpr.bodyModel.bodies{i}.name);
                link_names = strcat(link_names, ',;/');
            end
            obj.link_name_msg.Data = link_names;
            send(obj.link_name_pub, obj.link_name_msg);
            % Send link tfs
            link_tf = zeros(7, cdpr.numLinks); % Vector3+Quaternion
            for i = 1:cdpr.numLinks
                rot = cdpr.bodyModel.bodies{i}.R_0k;
                link_tf(1:3, i) = rot*cdpr.bodyModel.bodies{i}.r_OG;                
                quat = Quaternion.FromRotationMatrix(rot);
                link_tf(4:7, i) = [quat.q1; quat.q2; quat.q3; quat.q0];                
            end
            link_tf = reshape(link_tf, 1, 7*cdpr.numLinks);
            obj.link_tf_msg.Data = link_tf;
            send(obj.link_tf_pub, obj.link_tf_msg);
        end
        
        % publish attachements of segments
        function cableInfoSend(obj, cdpr)
            cable_vec = [];
            % Cables
            for c = 1:cdpr.cableModel.numCables
                this_cable = cdpr.cableModel.cables{c};
                for s = 1:length(this_cable.segments)
                    this_segment = this_cable.segments{s};
                    % 1st attachment point
                    cable_vec = [cable_vec; this_segment.attachments{1}.r_OA];
                    % 2nd attachment point
                    cable_vec = [cable_vec; this_segment.attachments{2}.r_OA];
                end                
            end
            obj.cable_msg.Data = cable_vec;
            send(obj.cable_pub, obj.cable_msg);
        end
        
        % publish end-effector
        function eeInfoSend(obj, cdpr)
            % Assume only 1 ee for now                           
            obj.ee_msg.Data = cdpr.y;
            send(obj.ee_pub, obj.ee_msg);
        end
        
        % publish cable force
        function forceInfoSend(obj, cdpr, f)
            % Create an array to hold the force for every segment          
            f_seg = zeros(cdpr.cableModel.numSegments, 1);
            seg_count = 1;
            % Assign the corresponding force to the segment
            for c = 1:cdpr.numCables
                for s = 1:length(cdpr.cableModel.cables{c}.segments)
                    % Only send a valid force mag for start and end segment
                    if s == 1 || s == length(cdpr.cableModel.cables{c}.segments)
                        f_seg(seg_count) = f(c);
                    else
                        % Otherwise, set it to -1 and CASPR-RViz will
                        % handle this special case
                        f_seg(seg_count) = -1;
                    end
                    seg_count = seg_count + 1;
                end
            end
            % Send the message out
            obj.f_msg.Data = f_seg;
            send(obj.f_pub, obj.f_msg);
        end
        
        % terminating function
        function terminate(obj, n_cables)
            % send '-1's if wanting to terminate
            obj.cmd_msg.Data = -1*ones(1, n_cables);
            send(obj.cmd_pub, obj.cmd_msg);
        end
        
        % callback function (not used)
        function sub_callback(obj, src, msg)
            % Nothing here
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Unused functions
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function forceFeedbackRead(obj)
        end
        function switchOperatingMode(obj)
        end
        function systemOffSend(obj)
        end
        function systemOnSend(obj)
        end
        function lengthFeedbackRead(obj)
        end
        function lengthInitialSend(obj)
        end
        function forceCommandSend(obj)
        end
        function lengthCommandSend(obj)
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
        
    methods (Access = private)        
        % create pusblishers
        function createPublisher(obj)
            try
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Visualization
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                [obj.link_name_pub, obj.link_name_msg] = rospublisher(obj.link_name_topic, obj.link_name_msg_type); 
                [obj.link_tf_pub, obj.link_tf_msg] = rospublisher(obj.link_tf_topic, obj.link_tf_msg_type);
                [obj.cable_pub, obj.cable_msg] = rospublisher(obj.cable_topic, obj.cable_msg_type);
                [obj.ee_pub, obj.ee_msg] = rospublisher(obj.ee_topic, obj.ee_msg_type);
                [obj.f_pub, obj.f_msg] = rospublisher(obj.f_topic, obj.f_msg_type);
            catch
                CASPR_log.Error('Failed to create publisher!');
            end
        end
        % create subscribers
        function createSubscriber(obj)
            try
                % No subscriber yet
            catch
                CASPR_log.Error('Failed to create subscriber!');
            end
        end
    end
end