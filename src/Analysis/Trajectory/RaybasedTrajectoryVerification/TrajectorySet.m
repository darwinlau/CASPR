%trajectory used in the paper
function  trajectory = TrajectorySet(num)
%%
% 1 -> linear
% 2 -> quadratic
% 3 -> cubic
% 4 -> sextic
% 5 -> helical
% 6 -> infinity sign
% 7 -> quadratic sets
% 8 -> non for paper
%%
x_y_z = []; angles = [];
switch num
    case 1
        x_y_z{1} = [2.5 1.9;
            1.7 1.2;
            3.3 1.7];
        angles = [20 5;
            0 5;
            0 10];
    case 2
        x_y_z{1} = [0.6 1.3 3.2;
            2.6 0.9 2.3;
            3.4 2.6 3.2];
        angles = [0 5;
            0 0;
            0 5];
    case 3
        x_y_z{1} = [3.5 2.5 1.7 0.8;
            1.1 1.2 2.3 3;
            2.5 2.5 2.5 2.5];
        %             3 1.9 2.9 3];
        angles = [0 30;
            0 0;
            0 0];
    case 4
        x_y_z{1} = [0.5 1 1.5 2 2.5 3 3.5;
            2 2 2 2 2 2 2 ;
            3 1 3 1 3 1 3];
        angles = [0 30;
            0 0;
            0 0];
    case 5
        t = linspace(0,4*pi,51);
        x = 0.8*cos(t)+2;
        y = 0.8*sin(t)+2;
        z = 0.25*t+0.5;
        take_index = round(linspace(1,length(t),12),0);
        x_y_z{1} = [x(take_index)
            y(take_index)
            z(take_index)];
        angles = [0 0;
            0 0;
            0 0];
    case 6
        t = linspace(0,2*pi,51);
        y = 1.5*cos(t)+2;
        z = 1.5*sin(2*t) / 2 +2;
        x = ones(1,51)*2;
        take_index = round(linspace(1,length(t),51),0);
        x_y_z{1} = [x(take_index)
            y(take_index)
            z(take_index)];
        angles = [5 0;
            0 5;
            0 5];
    case 7
        for i = 1:11
            x_y_z{1}  = [0.5 2 3.5;
                2 (i-1)*0.3+0.5 2;
                2 2 2];
        end
        for i = 1:11
            x_y_z{2}  = [0.5 2 3.5;
                2 2 2;
                2 (i-1)*0.3+0.5 2];
        end
        for i = 1:11
            x_y_z{3}  = [0.5 2 3.5;
                2 (i-1)*0.25+0.75 2;
                2 3.25-(i-1)*0.25 2];
        end
        for i = 1:11
            x_y_z{4} = [0.5 2 3.5;
                2 (i-1)*0.25+0.75 2;
                2 (i-1)*0.25+0.75 2];
        end
        angles = [0 0;
            5 0;
            0 0];
    otherwise
end
angles = deg2rad(angles);
for i = 1:size(x_y_z,1)
    trajectory{i}{1,:}  = x_y_z{i}(1,:);
    trajectory{i}{2,:}  = x_y_z{i}(2,:);
    trajectory{i}{3,:}  = x_y_z{i}(3,:);
    trajectory{i}{4,:}  = angles(1,:);
    trajectory{i}{5,:}  = angles(2,:);
    trajectory{i}{6,:}  = angles(3,:);
end
end