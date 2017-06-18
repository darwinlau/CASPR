% This source is for calculating the collision relationship between the 
% anything involved and the cables

clear;
% The size of the outer frame
length_frame = 1.69;
width_frame = 2.152;
height_frame = 2.0;

% The size of the end-effector
length_end = 0.1;
width_end = 0.2;
height_end = 0.04;

% The size of the construction
length_bld = 0.403;
width_bld = 0.5;
height_bld = 0.40;

% The size of the brick
length_brick = 0.09;
width_brick = 0.05;
height_brick = 0.05;

% The distance between upper side of the brick being held by the gripper 
% and the bottom of the end effector
length_hand = 0.20;

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
