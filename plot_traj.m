% script to plot the proposed trajectory
%
clc; clearvars; close all;
traj = [1 3 0.4; 0 2 0.8; 0 0 0];
x = []; y = []; z = [];
 for t = 0:0.2:2
    traj_xyz = traj*[t^2; t; 1];
    x = [x; traj_xyz(1)]
    y = [y; traj_xyz(2)]
    z = [z; traj_xyz(3)]
 end
figure
plot3(x,y,z)
grid on
axis([0 10 0 6 -0.5 0.5])
