classdef EndEffector < handle    
    properties (Access = private)
        % parallel to x direction
        length
        % parallel to y direction
        width
        % parallel to z direction
        height
        
        %
        hEE
        vertices_origin
    end
    methods
        function ee = EndEffector()
            ee.length = 0.1;
            ee.width = 0.2;
            ee.height = 0.04;
        end
        
        function plot(obj,dotted)
            % Here the order followes the motor order of the 2m by 2m
            % constructing robot on the first floor of AB1
            cube_ver1 = [-obj.length/2; obj.width/2; obj.height/2];
            cube_ver2 = [ obj.length/2; obj.width/2;-obj.height/2];
            cube_ver3 = [ obj.length/2; obj.width/2; obj.height/2];
            cube_ver4 = [ obj.length/2;-obj.width/2;-obj.height/2];
            cube_ver5 = [ obj.length/2;-obj.width/2; obj.height/2];
            cube_ver6 = [-obj.length/2;-obj.width/2;-obj.height/2];
            cube_ver7 = [-obj.length/2;-obj.width/2; obj.height/2];
            cube_ver8 = [-obj.length/2; obj.width/2;-obj.height/2];
            obj.vertices_origin = [cube_ver1,cube_ver2,cube_ver3,cube_ver4,cube_ver5,cube_ver6,cube_ver7,cube_ver8]';
            face = [8 1 3 2;2 3 5 4;4 5 7 6;6 7 1 8;1 7 5 3;6 8 2 4];
            face = (flip(eye(4))*face')';
            if(dotted)
                obj.hEE = patch('Vertices',obj.vertices_origin,'Faces',face);
                set(obj.hEE,'LineStyle',':');
                set(obj.hEE,'FaceColor','none');
                set(obj.hEE,'FaceAlpha',0.3);%transparency:0~1, The bigger this number is, the lower the transparency is.
            else
                obj.hEE = patch('Vertices',obj.vertices_origin,'Faces',face,'FaceVertexCData',gray(8));%gray hsv hot parula copper pink
                set(obj.hEE,'FaceColor','flat');
                set(obj.hEE,'FaceAlpha',0.9);%transparency:0~1, The bigger this number is, the lower the transparency is.
            end
            set(obj.hEE,'LineWidth',1);
        end
        
        function animate(obj, q)
            R = rpy2r(q(4),q(5),q(6));
            vert_temp = (R*obj.vertices_origin')';
            vert = vert_temp + ones(8,1)*[q(1) q(2) q(3)];
            set(obj.hEE,'Vertices',vert);
        end
    end
end