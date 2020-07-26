% Class for interacting with CARDSFlow through ROS
%
% Author        : Dominic Chan
% Created       : 2018
% Description   :
%    This class enables visualization of CDPRs through interacting with 
%    CARDSFlow
% CARDSFlow     : https://github.com/CARDSflow/CARDSflow.git

classdef CARDSFlowInterface < CableActuatorInterfaceBase
    properties (Access = private)        
        ROS_MASTER_URI;             % URI or the device running master
        ROS_IP;                     % IP of the local device        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Visualization
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        robot_state_pub;            % Publisher for robot states
        robot_state_msg;            % Message for robot states
        tendon_state_pub;           % Publisher for tendon states
        tendon_state_msg;           % Message for tendon states
    end
   
    properties (Access = private, Constant = true)  
        % Node name
        nodeName = '/CASPR_MATLAB';           
        %%%%%%%%%%%%%%%%%%%%%%%%%%
        % Robot state
        robot_state_topic = '/robot_state';
        robot_state_msg_type = 'geometry_msgs/PoseStamped';        
        % Tendon state
        tendon_state_topic = '/tendon_state';   
        tendon_state_msg_type = 'roboy_simulation_msgs/Tendon';               
    end
    
    methods (Access = public)
        % Constructor
        function interface = CARDSFlowInterface()
            interface@CableActuatorInterfaceBase();
            interface.ROS_MASTER_URI = CARDSFlow_configuration.LoadROS_MASTER_URI();
            interface.ROS_IP = CARDSFlow_configuration.LoadROS_IP();      
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
        function visualize(obj, cdpr)
            obj.robotStateSend(cdpr);
            obj.tendonStateSend(cdpr);            
        end      
                
        % publish robot state
        function robotStateSend(obj, cdpr)  
            pose_msg = rosmessage('geometry_msgs/Pose');
            position_msg = rosmessage('geometry_msgs/Point');
            orientation_msg = rosmessage('geometry_msgs/Quaternion');            
            for i = 1:cdpr.numLinks     
                % Link name                
                obj.robot_state_msg.Header.FrameId = cdpr.bodyModel.bodies{i}.name;
                % TF
                rot = cdpr.bodyModel.bodies{i}.R_0k;
                position = rot*cdpr.bodyModel.bodies{i}.r_OG;                
                quat = Quaternion.FromRotationMatrix(rot); 
                % Position
                position_msg.X = position(1); 
                position_msg.Y = position(2);
                position_msg.Z = position(3);
                % Orientation
                orientation_msg.X = quat.q1;
                orientation_msg.Y = quat.q2;
                orientation_msg.Z = quat.q3;
                orientation_msg.W = quat.q0;
                % Load to pose_msg
                pose_msg.Position = position_msg;
                pose_msg.Orientation = orientation_msg;
                % Load to robot_state_msg
                obj.robot_state_msg.Pose = pose_msg;                
                % Publish                
                send(obj.robot_state_pub, obj.robot_state_msg);                
            end   
        end
        
        % publish tendon state
        function tendonStateSend(obj, cdpr)  
            cable_Names = cell(cdpr.cableModel.numCables,1);
            n_Viapoints = zeros(cdpr.cableModel.numCables,1); 
            via_array = robotics.ros.msggen.geometry_msgs.Vector3.empty(2*cdpr.cableModel.numSegmentsTotal,0);
            % No use for now            
            f = zeros(cdpr.cableModel.numCables,1);            
            % Cables
            v_count = 1; 
            
            for c = 1:cdpr.cableModel.numCables                
                % Cable name               
                cable_name = sprintf('cable_%d', c);
                cable_Names{c} = cable_name;                 
                this_cable = cdpr.cableModel.cables{c};
                n_Viapoints(c) = 2*length(this_cable.segments); 
                % Load all viapoints to cable_vec               
                for s = 1:length(this_cable.segments)
                    this_segment = this_cable.segments{s};     
                    % 1st attachment point                        
                    viapoints_msg_1 = rosmessage('geometry_msgs/Vector3');                    
                    via_point_1 = this_segment.attachments{1}.r_OA;
                    viapoints_msg_1.X = via_point_1(1);
                    viapoints_msg_1.Y = via_point_1(2);
                    viapoints_msg_1.Z = via_point_1(3);                    
                    via_array(v_count) = viapoints_msg_1;                    
                    v_count = v_count + 1;
                    % 2nd attachment point                    
                    viapoints_msg_2 = rosmessage('geometry_msgs/Vector3');                    
                    via_point_2 = this_segment.attachments{2}.r_OA;
                    viapoints_msg_2.X = via_point_2(1);
                    viapoints_msg_2.Y = via_point_2(2);
                    viapoints_msg_2.Z = via_point_2(3);                    
                    via_array(v_count) = viapoints_msg_2;                   
                    v_count = v_count + 1;
                end                     
            end              
            obj.tendon_state_msg.Viapoints = via_array;
            obj.tendon_state_msg.Name = cable_Names;
            obj.tendon_state_msg.NumberOfViapoints = n_Viapoints;
            obj.tendon_state_msg.L = cdpr.cableLengths;
            obj.tendon_state_msg.Force = f;
            obj.tendon_state_msg.Ld = cdpr.cableLengthsDot;            
            % Publish
            send(obj.tendon_state_pub, obj.tendon_state_msg);  
        end
        
        % publish end-effector
        function eeInfoSend(obj, cdpr)
            % Assume only 1 ee for now                           
%             obj.ee_msg.Data = cdpr.y;
%             send(obj.ee_pub, obj.ee_msg);
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
                [obj.robot_state_pub, obj.robot_state_msg] = rospublisher(obj.robot_state_topic, obj.robot_state_msg_type); 
                [obj.tendon_state_pub, obj.tendon_state_msg] = rospublisher(obj.tendon_state_topic, obj.tendon_state_msg_type);
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