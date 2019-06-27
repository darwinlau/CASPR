% Class for interacting with MYOmuscle units through ROS
%
% Author        : Dominic Chan
% Created       : 2017
% Description    :
%    This class enables interaction with myomuscle units using ROS.
%    Operations involve creation of ROS node, sending force commands, and
%    reading position and velocity feedbacks.

classdef MyoInterface < CableActuatorInterfaceBase
    properties (Access = private)        
        ROS_MASTER_URI;             % URI or the device running master
        ROS_IP;                     % IP of the local device
        connection_status;          % connected to ROS master or not
        cable_cmd_pub;              % length command publisher
        cable_cmd_msg;              % length command message   
        joint_cmd_pub;              % length command publisher
        joint_cmd_msg;              % length command message 
        pose_sub;                   % pose subscriber
        pose_msg;                   % pose message
    end
   
    properties (Access = private, Constant = true)  
        nodeName = '/CASPR_MATLAB';
        cable_cmd_topic = '/MATLAB/CableCommand';
        cable_cmd_msg_type = 'sensor_msgs/JointState';    
        joint_cmd_topic = '/MATLAB/JointCommand';
        joint_cmd_msg_type = 'sensor_msgs/JointState';   
        pose_topic = '/MATLAB/Feedback';
        pose_msg_type = 'sensor_msgs/JointState';        
    end
    
    methods (Access = public)       
        function interface = MyoInterface(ROS_MASTER_URI, ROS_IP)
            interface@CableActuatorInterfaceBase();
            interface.ROS_MASTER_URI = ROS_MASTER_URI;
            interface.ROS_IP = ROS_IP;
            interface.connection_status = false;
            interface.initialise();
        end
    end
   
    methods (Access = private)
        
        % create a pusblisher
        function createPublisher(obj)
            try
                [obj.cable_cmd_pub, obj.cable_cmd_msg] = rospublisher(obj.cable_cmd_topic, obj.cable_cmd_msg_type);                
                [obj.joint_cmd_pub, obj.joint_cmd_msg] = rospublisher(obj.joint_cmd_topic, obj.joint_cmd_msg_type);
            catch
                CASPR_log.Error('Failed to create publisher!\n');
            end
        end
        % create a subscriber
        function createSubscriber(obj)
            try
                obj.pose_sub = rossubscriber(obj.pose_topic, obj.pose_msg_type, @obj.sub_callback);                
            catch
                CASPR_log.Error('Failed to create subscriber!\n');
            end
        end
    end
    
    methods
        function initialise(obj)
            % try to shut down the ros node
            try 
                rosshutdown();
                CASPR_log.Debug('ROS shut down.\n');
            catch
                CASPR_log.Debug('No node was running.\n');
            end
            obj.connection_status = false;           
        end
        
        % ros init
        function open(obj)
            % init if connected 
            if obj.connection_status 
                obj.initialise();
            end
            % connect to the master through the master ip
            try
                setenv('ROS_MASTER_URI',obj.ROS_MASTER_URI);
                setenv('ROS_HOSTNAME',obj.ROS_IP);
                rosinit('NodeName', obj.nodeName);

                obj.connection_status = true;
                CASPR_log.Debug('ROS initialized.\n');
            catch
                CASPR_log.Error('Failed to initialize ROS!\n');
            end          
            % create publisher & subscriber
            obj.createPublisher();
            obj.createSubscriber();
        end
        
        % shut down ros
        function close(obj)
            obj.initialise();
        end
        
        % publish commands
        function sendCableCommands(obj, cmd, q, q_d)
            obj.cable_cmd_msg.Effort = cmd;
            obj.cable_cmd_msg.Position = q;
            obj.cable_cmd_msg.Velocity = q_d;
            send(obj.cable_cmd_pub, obj.cable_cmd_msg);
        end
        
        % publish commands
        function sendJointCommands(obj, cmd, q, q_d)
            obj.joint_cmd_msg.Effort = cmd;
            obj.joint_cmd_msg.Position = q;
            obj.joint_cmd_msg.Velocity = q_d;
            send(obj.joint_cmd_pub, obj.joint_cmd_msg);
        end
        
        % subscribe pose information
        function [l, q, q_dot] = poseFeedbackRead(obj) 
            obj.pose_msg = obj.pose_sub.LatestMessage;
            while isempty(obj.pose_msg)
                obj.pose_msg = obj.pose_sub.LatestMessage;
            end        
            l = obj.pose_msg.Effort;
            q = obj.pose_msg.Position;
            q_dot = obj.pose_msg.Velocity;   
            obj.pose_msg = [];
        end            
        
        % terminating function
        function terminate(obj, n_cables)
            % send '-1's if wanting to terminate
            obj.cable_cmd_msg.Effort = -1*ones(1, n_cables);
            send(obj.cable_cmd_pub, obj.cable_cmd_msg);
            rosshutdown
        end
        
        function sub_callback(obj, ~, msg)
            obj.pose_msg = msg;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Unused functions
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function forceFeedbackRead(~)
        end
        function switchOperatingMode(~)
        end
        function systemOffSend(~)
        end
        function systemOnSend(~)
        end
        function lengthFeedbackRead(~)
        end
        function lengthInitialSend(~)
        end      
        function lengthCommandSend(~)            
        end
        function [success] =detectDevice(~)
        end  
        function forceCommandSend(~, ~)
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
end