% script on trajectory verification for cable interference free conditions
% (IFC) and wrench closure condtions (WCC)
clc;  close all; warning off; clear all;

%% the model  
% 'CDPR_7c_6Dof'
model_config    =   DevModelConfig('CDPR_7c_6Dof'); 
cable_set_id    =   'original';
modelObj        =   model_config.getModel(cable_set_id);

%% Quaternions
degLow = 0;
degUp = 10;
q0_given = [cos(degLow/360*pi) sin(degLow/360*pi) 0 0]; % x-axis
q1_given = [cos(degUp/360*pi) sin(degUp/360*pi) 0 0];
q0 = quatnormalize(q0_given);
q1 = quatnormalize(q1_given);
quaternionDoFs = [q0; q1];

cosAgl = sum(q0 .* q1);
theta = acos(cosAgl);

% T\in [0, tan(theta/2)]
T_end = tan(theta/2);

%% translation polynomial trajectories 
cnt = 1;
for lambda = 0:pi/16:2*pi % circle
    if lambda ~= 2*pi
        %% circle area
        position0 = [2, 2, 2];
        position1 = [2+2*cos(lambda+pi/2), 2+2*sin(lambda+pi/2), 2];

        % QUADRATIC translation path [1] (x = c_x2 T^2 + c_x1*T + c_x0)
        T_insertPt = 0.5*T_end;
        insertPt = [2+1*cos(lambda+30*pi/180), 2+1*sin(lambda+30*pi/180), 2];
        % insertPt = [2+1*cos(lambda+150*pi/180), 2+1*sin(lambda+150*pi/180), 2];
        T_givenPts = [0 T_insertPt T_end];
        for i = 1:3
            val = [position0(i) insertPt(i) position1(i)];
            C_wrt_T(i,:) = polyfit(T_givenPts, val, 2);
        end
        startEndPts{cnt} = [position0; insertPt; position1];
        translationPolynomial{cnt} = C_wrt_T;
        clear C_wrt_T
        cnt = cnt+1;
    end
end
rayTrajectory = {quaternionDoFs, translationPolynomial};
DegofTranslation = size(startEndPts{1},1)-1;

%% workspace conditions
% required distance threshold for IFC
ds = 0.06;
rayCondType1 = TrajectoryRayConditionType.CABLE_INTERFERENCE_FREE;
rayCondType2 = TrajectoryRayConditionType.WRENCH_CLOSURE;
w_condition  = {TrajectoryRayConditionBase.CreateTrajectoryRayCondition(rayCondType1,0,ds)
                TrajectoryRayConditionBase.CreateTrajectoryRayCondition(rayCondType2,6,DegofTranslation)};
opt          = RayTrajectorySimulatorOptions(false,false);

%% Start the simulation
disp('Start Setup Simulation');
wsim            =   RayTrajectorySimulator(modelObj,rayTrajectory,opt);

%% Run the simulation
disp('Start Running Simulation');
wsim.run(w_condition,[])

%% plot the trajectory-based workspace
wsim.plotRayWorkspace2Cond(startEndPts,[1,2,3])
% print the string 'Initial'
text(position0(1)+0.2,position0(2),position0(3)+0.2,'Initial','FontSize',20)
% print the string 'Terminal'
text(position1(1)+0.2,position1(2),position1(3)+0.2,'Terminal','FontSize',20)
                    
