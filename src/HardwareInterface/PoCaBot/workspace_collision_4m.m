% This source is for calculating the collision relationship between the 
% anything involved and the cables

% clear;
% The size of the outer frame
length_frame = 3.939-0.061;
width_frame = 3.934-0.066;
height_frame = 3.6425-0.3575;

% The size of the end-effector
length_end = 0.1;
width_end = 0.2;
height_end = 0.04;

% The size of the construction
length_bld = 0.9;
width_bld = 0.9;
height_bld = 0.6;

% The size of the brick
length_brick = 0.09;
width_brick = 0.05;
height_brick = 0.05;

% The distance between upper side of the brick being held by the gripper 
% and the bottom of the end effector
length_hand = 0.36;

% When we think about one side of the cube, the other side should be one
% brick higher than the working side.
length_hand_cast = length_hand - 0;


%% As the workspace is symmetic and we don't need to consider the upper four cables, we just take the cable 8 into account.
% The range of the pick up place
width_pickup_place = 0;
height_pickup_place = 0;

% The central coordinate of the pickup area
pickup_place_co = [0,0,-height_frame/2];

% The central coordinate of the construction
syms construction_cetre_co_x;
construction_centre_bottom_co = [construction_cetre_co_x; 0; -height_frame/2-0.0];

%% For premise 1
pick_up_point_8_co = [pickup_place_co(1); pickup_place_co(2);(height_pickup_place - height_frame/2 +length_hand)];
frame_vertice_8  = [-length_frame/2; width_frame/2;-height_frame/2];
pivot_const_1_co = [construction_centre_bottom_co(1)+length_bld/2;construction_centre_bottom_co(2)+width_bld/2;construction_centre_bottom_co(3)+height_bld];

line1 = pick_up_point_8_co - frame_vertice_8;
line1_slope_yx = line1(2)/line1(1);
a1 = width_frame/2/(-line1_slope_yx);
b1 = a1*width_bld/width_frame;

% The distance between the left side of the bld and frame;
d1 = a1-b1-length_bld;
% This distance should not be less than 1/4 of the whole length of the
% frame in case of overloading of the motor
ratio = d1/length_frame;
if(ratio > 0.25)
    fprintf('The place of the construction is reasonable. The ratio is %0.3f.\n',ratio);
else
    fprintf('[WARNING] The place of the construction is unreasonable. The ratio is %0.3f.\n',ratio);
end

% For the best 
construction_cetre_co_x = -length_frame/2 + d1 + length_bld/2;


%% For premise 2
place_point_co = [0;-construction_centre_bottom_co(2)-width_bld/2+width_end/2;construction_centre_bottom_co(3)+height_bld+length_hand_cast];
pivot_const_2_co = [0; construction_centre_bottom_co(2)+width_bld/2; construction_centre_bottom_co(3)+height_bld];
line2 = place_point_co - frame_vertice_8;
line2_slope_zy = line2(3)/line2(2);
d2 = double((width_frame-width_bld)/2*(-line2_slope_zy));
error_d2 = double(d2 - (pivot_const_2_co(3)+height_frame/2));
if(error_d2>=0)
    fprintf('The length of the gripper and the width of the frame both are reasonable. The surplus is %0.3f.\n',error_d2);
    fprintf('And the slope is %0.3f.\n',abs(double(line2_slope_zy)));
else
    fprintf('[WARNING] One of the length of the gripper and the width of the frame is unreasonable. The error height is %0.3f.\n',error_d2);
    fprintf('And the slope is %0.3f.\n',abs(double(line2_slope_zy)));
end

%% For premise 4
pick_up_point_2_co = [pickup_place_co(1)+length_end/2; -width_pickup_place/2+width_end/2;(height_pickup_place - height_frame/2 +length_hand)];
frame_vertice_2  = [length_frame/2; width_frame/2;-height_frame/2];

line4 = pick_up_point_2_co - frame_vertice_2;
line4_slope_yx = line4(2)/line4(1);
a4 = width_pickup_place/2 - pick_up_point_2_co(2);
c4 = a4/line4_slope_yx;
d4 = c4+length_end/2;
fprintf('The length of the delivery line should not be less than %0.3f.\n',d4);

%% For the length of the cables
frame_vertice_1 = [-length_frame/2; width_frame/2; height_frame/2];
frame_vertice_2 = [length_frame/2; width_frame/2; -height_frame/2];
frame_vertice_3 = [length_frame/2; width_frame/2; height_frame/2];
frame_vertice_4 = [length_frame/2; -width_frame/2; -height_frame/2];
frame_vertice_5 = [length_frame/2; -width_frame/2; height_frame/2];
frame_vertice_6 = [-length_frame/2; -width_frame/2; -height_frame/2];
frame_vertice_7 = [-length_frame/2; -width_frame/2; height_frame/2];
frame_vertice_8  = [-length_frame/2; width_frame/2;-height_frame/2];

height_attach = 0.3575;
ratio_safe_distance = 0.30;
length_in_holder = 0.21;
length_full_spool = 4.5;
% The distance above what the gripper is moving
height_safe = 0.1;
% For the upper two cables of the construction side 
pivot_shortest_1 = [(-length_frame/2 + d1 - length_end/2); (width_bld/2 + width_end/2); (-height_frame/2 + height_bld +length_hand + height_end + height_safe)];
pivot_longest_1 = [length_frame/2*(1-ratio_safe_distance);-width_frame/2*(1-ratio_safe_distance);-height_frame/2-height_attach+length_hand+ height_end];
delta_cable1 = norm(frame_vertice_1 - pivot_longest_1) - norm(frame_vertice_1 - pivot_shortest_1);
length_cable1 = (length_full_spool-delta_cable1)/2+norm(frame_vertice_1 - pivot_longest_1)+length_in_holder;

% For the upper two cables of the brick side
pivot_shortest_5 = [length_frame/2*(1-ratio_safe_distance)+length_end/2; -width_frame/2*(1-ratio_safe_distance)-width_end/2; -height_frame/2-height_attach+length_hand+ height_end+height_safe];
pivot_longest_5_1 = [(-length_frame/2 + d1 + length_end/2); (width_bld/2 - width_end/2); (-height_frame/2 - height_attach +length_hand+height_end)];
pivot_longest_5_2 = [length_frame/2*(1-ratio_safe_distance)+length_end/2; width_frame/2*(1-ratio_safe_distance)-width_end/2; -height_frame/2-height_attach+length_hand+ height_end+height_safe];
[~, index] = max([norm(frame_vertice_5 - pivot_longest_5_1),norm(frame_vertice_5 - pivot_longest_5_2)]);
if(index == 1)
    pivot_longest_5 = pivot_longest_5_1;
else
    pivot_longest_5 = pivot_longest_5_2;
end
delta_cable5 = norm(frame_vertice_5 - pivot_longest_5) - norm(frame_vertice_5 - pivot_shortest_5);
length_cable5 = (length_full_spool-delta_cable5)/2+norm(frame_vertice_5 - pivot_longest_5)+length_in_holder;

% For the lower two cables of the construction side
pivot_shortest_8 = pivot_shortest_1;
pivot_shortest_8(3) = -length_frame/2;
pivot_longest_8 = pivot_longest_1;
pivot_longest_8(3) = pivot_longest_1(3)-height_end;
delta_cable8 = norm(frame_vertice_8 - pivot_longest_8) - norm(frame_vertice_8 - pivot_shortest_8);
length_cable8 = (length_full_spool-delta_cable8)/2+norm(frame_vertice_8 - pivot_longest_8)+length_in_holder;

% For the lower two cables of the brick side
pivot_shortest_4 = pivot_shortest_5;
pivot_shortest_4(3) = -height_frame/2;
pivot_longest_4_1 = pivot_longest_5_1;
pivot_longest_4_1(3) = pivot_longest_5_1(3)-height_end;
pivot_longest_4_2 = pivot_longest_5_1;
pivot_longest_4_2(3) = -height_frame/2 + height_bld + length_hand;
pivot_longest_4_3 = pivot_longest_5_2;
pivot_longest_4_3(3) = -height_frame/2-height_attach+length_hand + height_safe;
pivot_longest_4_4 = pivot_longest_5_2;
pivot_longest_4_4(3) = -height_frame/2-height_attach+length_hand;
[~, index] = max([...
    norm(frame_vertice_4 - pivot_longest_4_1),...
    norm(frame_vertice_4 - pivot_longest_4_2),...
    norm(frame_vertice_4 - pivot_longest_4_3),...
    norm(frame_vertice_4 - pivot_longest_4_4)]);
switch index
    case 1
        pivot_longest_4 = pivot_longest_4_1;
    case 2
        pivot_longest_4 = pivot_longest_4_1;
    case 3
        pivot_longest_4 = pivot_longest_4_1;
    case 4
        pivot_longest_4 = pivot_longest_4_1;
end
delta_cable4 = norm(frame_vertice_4 - pivot_longest_4) - norm(frame_vertice_4 - pivot_shortest_4);
length_cable4 = (length_full_spool-delta_cable4)/2+norm(frame_vertice_4 - pivot_longest_4)+length_in_holder;

%% DRAWING
axis([-2,2,-2,2,-2,2]);hold on;
cube_ver1 = [-length_bld/2; width_bld/2; height_bld/2];
cube_ver2 = [ length_bld/2; width_bld/2;-height_bld/2];
cube_ver3 = [ length_bld/2; width_bld/2; height_bld/2];
cube_ver4 = [ length_bld/2;-width_bld/2;-height_bld/2];
cube_ver5 = [ length_bld/2;-width_bld/2; height_bld/2];
cube_ver6 = [-length_bld/2;-width_bld/2;-height_bld/2];
cube_ver7 = [-length_bld/2;-width_bld/2; height_bld/2];
cube_ver8 = [-length_bld/2; width_bld/2;-height_bld/2];
vert = [cube_ver1,cube_ver2,cube_ver3,cube_ver4,cube_ver5,cube_ver6,cube_ver7,cube_ver8]';
patch_move = [-length_frame/2 + d1 + length_bld/2;0;-height_frame/2 + height_bld/2];
vert = vert + ones(8,1)*patch_move';
fac = [8 1 3 2;2 3 5 4;4 5 7 6;6 7 1 8;1 7 5 3;6 8 2 4];
patch('Vertices',vert,'Faces',fac,'FaceVertexCData',hsv(8),'FaceColor','interp');

coordinate1 = [pivot_shortest_1,frame_vertice_1,pivot_longest_1]';
coordinate4 = [pivot_shortest_4,frame_vertice_4,pivot_longest_4]';
coordinate5 = [pivot_shortest_5,frame_vertice_5,pivot_longest_5]';
coordinate8 = [pivot_shortest_8,frame_vertice_8,pivot_longest_8]';
plot3(coordinate1(:,1),coordinate1(:,2),coordinate1(:,3));
plot3(coordinate4(:,1),coordinate4(:,2),coordinate4(:,3));
plot3(coordinate5(:,1),coordinate5(:,2),coordinate5(:,3));
plot3(coordinate8(:,1),coordinate8(:,2),coordinate8(:,3));
plot3(coordinate1(:,1),coordinate1(:,2),coordinate1(:,3), '*');
plot3(coordinate4(:,1),coordinate4(:,2),coordinate4(:,3), '*');
plot3(coordinate5(:,1),coordinate5(:,2),coordinate5(:,3), '*');
plot3(coordinate8(:,1),coordinate8(:,2),coordinate8(:,3), '*');