clear all;clc;close all;
% CASPR_configuration.SetDevModelConfig(1);
% model_config = DevModelConfig('4_4_CDPR_planar');
% cable_set_id = 'original';
model_config = DevModelConfig('CU-Brick');
cable_set_id = 'demo_causewaybay';
% model_config = DevModelConfig('Traj_Spatial_Rotation');
% cable_set_id = '7cable';
% model_config    =    ModelConfig('BMArm');
% cable_set_id    =   'WORKING';
% cable_set_id = 'multilink';
modelObj        =   model_config.getModel(cable_set_id);

syms t;

q = cell(modelObj.numDofs,1);
%% Polynomial Example
% q{1,:} = [0.6 0.09 0.5 0.7 1];
% q{2,:} = [1 0];
% q{3,:} = [0];
% 
% q{1,:} = [5];
% q{2,:} = [1 0];
% q{3,:} = [0];

% q{1,:}  = [1 0];
% q{2,:}  = [0.5];
% q{3,:}  = [0];
% q{4,:}  = [0];

% q{1,:}  = [1.5];
% q{2,:}  = [1.5];
% q{3,:}  = [1.5];
% q{4,:}  = [1 0];
% q{5,:}  = [0];
% q{6,:}  = [0];

% q{1,:}  = [1 0];
% q{2,:}  = [1.5];
% q{3,:}  = [1.5];
% q{4,:}  = [-0.002 0.2 0];
% q{5,:}  = [0.0001 -0.002 0.2 0];
% q{6,:}  = [0.0004 -0.002 0.2];

q{1,:}  = [2.28571428571429];
q{2,:}  = [1];
q{3,:}  = [0.714285714285714];
q{4,:}  = [1 -3.1416];
q{5,:}  = [3.14160000000000];
q{6,:}  = [3.14160000000000];
%% Trigonometric Function Example

% q{1,:} = cos(8*t)*cos(t)+5;
% q{2,:} = sin(5*t)*cos(t)+0.8;
% q{3,:} = [];

% q{1,:}  = 3.14*sin(t);
% q{2,:}  = 0.1*cos(t)+0.5;
% q{3,:}  = 0.2*t;
% q{4,:}  = 0.1*t;
% q{5,:}  = 0;
% q{6,:}  = 0;



% q{1,:}  = 0.5*t*cos(2*t)  + 1.5;
% q{2,:}  = 0.5*t*sin(2*t) - 0.1;
% q{3,:}  = 0.2*t;
% q{4,:}  = 0.0001*t;


% q{1,:}  = cos(t)+1.5;
% q{2,:}  = 1.5*sin(2*t)+1;
% q{3,:}  = 1.5;
% q{4,:}  = 0;
% q{5,:}  = 0;
% q{6,:}  = 0;
% 
% q{1,:}  = cos(4*t)*cos(t)+1.5;
% q{2,:}  = sin(3*t)*cos(t)+1;
% q{3,:}  = 1.5;
% q{4,:}  = 0;
% q{5,:}  = 0;
% q{6,:}  = 0;

% q{1,:}  = cos(4*t)*cos(t)+1.5;
% q{2,:}  = sin(3*t)*cos(t)+1.2;
% q{3,:}  = sin(t)+1.5;
% q{4,:}  = 0;
% q{5,:}  = 0;
% q{6,:}  = 0;

q{1,:}  = cos(8*t)*cos(t)+1.5;
q{2,:}  = sin(5*t)*cos(t)+1.2;
q{3,:}  = 0.2*t+ 0.5;
q{4,:}  = 0;
q{5,:}  = 0;
q{6,:}  = 0;
%% Pose-Wise Example
%% data structure:
% q = [q_1 q_2 ... q_n], where q_i = [x_1 x_2 x_3 ... x_m time]

load('6_d_pose.mat');
trajectory_poses = poses;

%%
trajectory_type = 'Polynomials';
trajectory_type = 'Nonpolynomials';
trajectory_type = 'Poses';

if ~isequal(trajectory_type ,'Poses')
    tsim = PathVerification(modelObj,q,[],trajectory_type);
else
    tsim = PathVerification(modelObj,[],trajectory_poses,trajectory_type);
end

tsim.run();



