% Author        : 
% Created       : 2019
% Description	: Assuming cables are 

classdef ScrewDriveRotatingPulleyAttachment < CableAttachmentBase
    properties %(SetAccess = private)
        constantWinchPoint          % Refers to fixed point 
        winchVector
        varyingPulleyPoint
        cablePulleyEntryVector
        pulleyCentrePoint
        nextAttachment
        
        cableLengthInit             % Refers to L0, the cable length for q0/system length during initialisation
        cableLengthCurrent          % Refers to current cable length, which = L0 + deltaL
        initialised
    end
    
    properties %(Access = private)
        pulleyRadius
        winchRadius
        winchGrooveWidth
        outletPose                  % Initial outlet pose t
        currentOutletPose           % Current outlet pose t     
        l0_offset
    end
    
    methods
        function ca = ScrewDriveRotatingPulleyAttachment(constant_winch_point, winch_vector, cable_pulley_entry_vector, pulley_radius, winch_radius, winch_groove_width, next_attachment)
            ca.link_num = 0;
            ca.constantWinchPoint = constant_winch_point;
            ca.winchVector = winch_vector;
            ca.cablePulleyEntryVector = cable_pulley_entry_vector;
            ca.pulleyRadius = pulley_radius;
            ca.winchRadius = winch_radius;
            ca.winchGrooveWidth = winch_groove_width;
            ca.nextAttachment = next_attachment;
            ca.r_GA = [0;0;0];
            ca.initialised = 0;
        end
        
        function update(obj, bodyKinematics)
            obj.nextAttachment.update(bodyKinematics);
            % Solving L0 using given t
            if (obj.initialised == 0)
                % t = 0.03 should be replaced by hardware reading
                [obj.cableLengthInit, obj.varyingPulleyPoint, obj.r_GA, obj.length_offset, obj.pulleyCentrePoint] = obj.computeInitialCableLength(obj, obj.cablePulleyEntryVector, obj.pulleyRadius, obj.constantWinchPoint, obj.winchVector, obj.nextAttachment.r_OA, 0.03);
                [obj.outletPose] = obj.obtainInitialOutletPose;
                obj.initialised = 1;
            else
            % Updating the varyingPulleyPoint
                [obj.varyingPulleyPoint] = obj.ComputeVaryingPulleyPoint(obj, obj.winchGrooveWidth, obj.winchRadius, obj.constantWinchPoint, obj.winchVector, obj.nextAttachment.r_OA, obj.cableLengthInit);
                [obj.r_GA, obj.length_offset, obj.pulleyCentrePoint] = obj.ComputePulleyLeavingLocation(obj.varyingPulleyPoint, obj.cablePulleyEntryVector, obj.nextAttachment.r_OA, obj.pulleyRadius);
            end
            update@CableAttachmentBase(obj, bodyKinematics);
        end
        
        function updateAttachmentLocation(obj, r_A, ~)
% %             CASPR_log.Error('updateAttachmentLocation function not implemented for BaseRotatingPulleyAttachment yet');
%             % Re-look at whether this should be the implementation

        end
    end
    
    methods (Static)
        function ca = LoadXmlObj(xmlObj, next_attachment)
            constant_winch_point = XmlOperations.StringToVector3(char(xmlObj.getElementsByTagName('constant_winch_point').item(0).getFirstChild.getData));
            winch_vector = XmlOperations.StringToVector3(char(xmlObj.getElementsByTagName('winch_vector').item(0).getFirstChild.getData));
            cable_pulley_entry_vector = XmlOperations.StringToVector3(char(xmlObj.getElementsByTagName('cable_pulley_entry_vector').item(0).getFirstChild.getData));
            pulley_radius = str2double(xmlObj.getElementsByTagName('pulley_radius').item(0).getFirstChild.getData);
            winch_radius = str2double(xmlObj.getElementsByTagName('winch_radius').item(0).getFirstChild.getData);
            winch_groove_width = str2double(xmlObj.getElementsByTagName('winch_groove_width').item(0).getFirstChild.getData);
            ca = ScrewDriveRotatingPulleyAttachment(constant_winch_point, winch_vector, cable_pulley_entry_vector, pulley_radius, winch_radius, winch_groove_width, next_attachment);  
        end
        

        function [r_A] = ComputeVaryingPulleyPoint(obj, winch_groove_width, winch_radius, constant_winch_point, winch_vector, r_OB, cableLengthInit)
            % Constants
            h = winch_groove_width;              % groove width of winch
            D = winch_radius*2;               % Diameter of winch
            r_A0 = constant_winch_point;   % start Pt of winch
            v = winch_vector;          % Direction of moving spool
            l0 = cableLengthInit;
            
            % l_arc calculation
%             r_pull = obj.pulleyRadius;
%             z = abs(r_OB(3) - r_A0(3)) + obj.pulleyRadius;
%             R = abs(r_OB(1) - r_A0(1));
%             T = sqrt((R - r_pull)^2 + (z - r_pull)^2);
%             alpha = atan((z-r_pull)/(R-r_pull)) + asin(r_pull/T);
%             l_arc = pi*r_pull*(rad2deg(alpha) + 90)/180;
            
%             [~,~,l_arc] = calculateKingpinPulley(obj.pulleyRadius, r_A0, r_OB);
            
            % Variables
            c = -r_A0 + r_OB;
            c_1 = 1 - (h/pi/D)^2 * dot(v,v);
            c_2 = 2*obj.l0_offset + (2*h/pi/D)*dot(c,v);
            c_3 = obj.l0_offset^2 - dot(c,c);
%             c_2 = 2*l0 + (2*h/pi/D)*dot(c,v) + 2*l_arc*h/pi/D*norm(v);
%             c_3 = l0^2 - dot(c,c) - 2*l_arc*norm(c) - l_arc^2;
            deltaLRaw = roots([c_1, c_2, c_3]);
            [deltaL] = obj.deltaSelection(deltaLRaw, l0, constant_winch_point, r_OB);
            deltat = deltaL*h/pi/D;                  % delta t, change in t that to compensate the change in delta L
            obj.cableLengthCurrent = l0 + deltaL;
            obj.currentOutletPose = deltat + obj.outletPose;
            r_A = r_A0 + obj.currentOutletPose*v; % actual r_OA
        end
        
        % This function is to collect initial outlet position from external
        % sensor, which has to be modified and added in future updates
        function [t] = obtainInitialOutletPose()
            encoder = 28032;
            t = encoder*10^-6+0.038819;
        end
        
        % This function used to distinguish the useful roots in calculating
        % the delta L
        function [deltaL] = deltaSelection(deltaLRaw, l0, constant_winch_point, r_OB)
            approxL = norm(-constant_winch_point + r_OB);
            approxLCurrent = l0.*ones(2,1)+ deltaLRaw;
            d = approxL - approxLCurrent;
            [~,I] = min(d);
            deltaL = deltaLRaw(I);
        end
        
        % Function to determine the initial cable length attaching the
        % end-effector and the exit pulley at the motor when the position
        % of exit pulley (t) is known. This is only run for once when 
        function [l0, r_A, r_D, l_arc, r_C] = computeInitialCableLength(obj, cablePulleyEntryVector, R, constant_winch_point, winch_vector, r_OB, t)
%             deltaL = t*pi*winch_radius*2/winch_groove_width;
%             cableLength = norm(-constant_winch_point + r_OB - t*winch_vector);
%             c_2 = -2*deltaL;
%             c_3 = deltaL^2 - cableLength;
%             sol = roots([1;c_2;c_3]);
%             l0 = sol(sol>0);
            r_A = constant_winch_point + t*winch_vector;
            [r_D, l_arc, r_C] = obj.ComputePulleyLeavingLocation(r_A, cablePulleyEntryVector, r_OB, R);
%             [~,~,l_arc_2] = obj.calculateKingpinPulley(obj.pulleyRadius, r_A, r_OB);
%             l_arc - l_arc_2
            l0 = norm(-r_D + r_OB) + l_arc;
            obj.l0_offset = norm(-constant_winch_point + r_OB);
%             l0_diff = l0 - norm(-r_A + r_OB)
        end
        
        function [D, center, l_arc] = calculateKingpinPulley(r,B,A)
            A_prime = (A-B);                     %Only l_arc is accounted
            theta = atan(A_prime(2)/A_prime(1));
%             if(A(3)<B(3))
%                 theta = pi/2 - theta
%             end
            R = A(1)/cos(theta);
            if A_prime(3) < 0
                z = B(3) - A(3)+ r;
            else
%                 z_1 = B(3) - A(3)+ r;
                z = A(3);
            end
            center = [r*cos(theta); r*sin(theta); 0] + B;
            T = sqrt((R-r)^2 + (z-r)^2);
            beta = atan((z-r)/(R-r));
            alpha = beta + asin(r/T);
            D = [(r+r*sin(alpha))*cos(theta); (r+r*sin(alpha))*sin(theta); r-r*cos(alpha)] + B;
            l_arc = pi*r*(rad2deg(alpha) + 90)/180;
        end
        
        % Function to determine the leaving location of the pulley given:
        % INPUTS
        %   - r_A: Constant location on the pulley where cable enters
        %   - v_A: Direction vector of the cable entering r_A
        %   - r_B: Location of the other end of the cable on the robot
        %   - R: Radius of the pulley
        % OUTPUTS
        %   - r_D: The position of the cable leaving the pulley
        %   - l_arc: length of the arc on the pulley
        %   - r_C: The position of the centre of the pulley location
        function [r_D, l_arc, r_C] = ComputePulleyLeavingLocation(r_A, v_A, r_B, R)
            n_plane = cross(v_A, r_B - r_A);
            p_plane = [0;0;0];
            n_c_perpendicular = v_A;
            p_c_perpendicular = [0;0;0];
            
            [p_C, v_C, ~] = PlaneOperations.PlaneIntersection(n_c_perpendicular, p_c_perpendicular, n_plane, p_plane);
            a = v_C' * v_C;
            b = 2 * p_C' * v_C;
            c = p_C' * p_C - R^2;
            t = (-b - sqrt(b^2 - 4 * a * c))/(2*a);
            r_C = r_A + (p_C + t*v_C);
            
            n_d_perpendicular = (r_C - r_B);
            p_d_perpendicular = [0; 0; -R^2/n_d_perpendicular(3)];
            
            [p_D, v_D, ~] = PlaneOperations.PlaneIntersection(n_d_perpendicular, p_d_perpendicular, n_plane, p_plane);
            a = v_D' * v_D;
            b = 2 * p_D' * v_D;
            c = p_D' * p_D - R^2;
            t = (-b - sqrt(b^2 - 4 * a * c))/(2*a);
            r_D = r_C + (p_D + t*v_D);
            
            n1 = (r_B - r_D)/norm(r_B-r_D);
            n2 = v_A/norm(v_A);
            l_arc = R * acos(n1' * n2);
        end
    end
end
