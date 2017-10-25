
clc;  close all; warning off

%%%          2 DoF VSD

% Set up the model 

% model_config    =   ModelConfig('2 DoF VSD');   %    spatial7cable   BMArm_paper   BMArm_paper
% cable_set_id    =   'basic';
% modelObj        =   model_config.getModel(cable_set_id);
% 
% 
% q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max; q_initial=modelObj.bodyModel.q_initial;  
% 
% nsegvar= [25;25];      % number of discritization on each axis. if the user desire to ignor discritization on one axis its corresponding discritiaztion number can be set to zero
% uGrid           =   RayGridGeneration(q_begin,q_end,q_initial,nsegvar); 
% 
% % Set up the workspace simulator
% 
% wsim            =   WorkspaceRayGeneration(modelObj,uGrid);
% wsim.run(2,0);    % the input can be empty or two digits. the first indicate the percentage of ray length which are ignored in workspace. 
%                   % the second one is the readmode. if readmode is one then the result of computation in the data file is read without repeating the computation    
% wsim.plotRayWorkspace([1,2]);     % the inout is a 1x2 or 1x3 vector of axes for plotting.
% gsim   =  GridGraphGeneration(modelObj,uGrid,wsim);  % RayGraphGeneration(modelObj,uGrid,wsim);
% gsim.run(1,0)       % the input can be empty or two digits. the first indicate weighted/unweighted graph. 0 means unweighted and 1 means weighted based on tension factor
%                     % the second one is the readmode. if readmode is one then the result of computation in the data file is read without repeating the computation    
% gsim.plotGraphWorkspace

%          4-4_CDPR_planar

% model_config    =   DevModelConfig('4-4_CDPR_planar');   %    spatial7cable   BMArm_paper   BMArm_paper
% cable_set_id    =   'original';
% modelObj        =   model_config.getModel(cable_set_id);
% 
% q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max; q_initial=modelObj.bodyModel.q_initial;
% nsegvar= [25;25;25];  
% uGrid           =   RayGridGeneration(q_begin,q_end,q_initial,nsegvar);
% 
% wsim            =   WorkspaceRayGeneration(modelObj,uGrid);
% wsim.run(0,0)            % the input can be empty or two digits. the first indicate the percentage of ray length which are ignored in workspace. 
% %                   % the second one is the readmode. if readmode is one then the result of computation in the data file is read without repeating the computation    
% wsim.plotRayWorkspace([1,2,3])
% 
% gsim   =  GridGraphGeneration(modelObj,uGrid,wsim);  %RayGraphGeneration(modelObj,uGrid,wsim);
% 
% gsim.run(0,0)       % the input can be empty or two digits. the first indicate weighted/unweighted graph. 0 means unweighted and 1 means weighted based on tension factor
%                     % the second one is the readmode. if readmode is one then the result of computation in the data file is read without repeating the computation    
% gsim.plotGraphWorkspace

%%%          BMArm_paper

% model_config    =   DevModelConfig('BMArm_paper');   %    spatial7cable     
% cable_set_id    =   'original';
% modelObj        =   model_config.getModel(cable_set_id);
% 
% q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max; q_initial=modelObj.bodyModel.q_initial;
% nsegvar= [30;30;30;0];  
% uGrid           =   RayGridGeneration(q_begin,q_end,q_initial,nsegvar);
% 
% wsim            =   WorkspaceRayGeneration(modelObj,uGrid);
% wsim.run(0,0)
% %wsim.plotRayWorkspace([1,2,3])
% gsim   =  GridGraphGeneration(modelObj,uGrid,wsim);  %RayGraphGeneration(modelObj,uGrid,wsim);
% gsim.run(1,0)       % the input can be empty or two digits. the first indicate weighted/unweighted graph. 0 means unweighted and 1 means weighted based on tension factor
%                     % the second one is the readmode. if readmode is one then the result of computation in the data file is read without repeating the computation    
% gsim.plotGraphWorkspace

%%%%        spatial7cable

% model_config    =   DevModelConfig('spatial7cable');   %    spatial7cable     
% cable_set_id    =   'original';
% modelObj        =   model_config.getModel(cable_set_id);
% 
% q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max; q_initial=modelObj.bodyModel.q_initial;
% nsegvar= [0;0;0;30;30;30];  
% uGrid           =   RayGridGeneration(q_begin,q_end,q_initial,nsegvar);
% 
% wsim            =   WorkspaceRayGeneration(modelObj,uGrid);
% wsim.run(3,0)
% wsim.plotRayWorkspace([4,5,6])









%% planar ones

% % %%          2 DoF VSD
% % 
% % model_config    =   ModelConfig('2 DoF VSD');   %    spatial7cable   BMArm_paper   BMArm_paper
% % cable_set_id    =   'basic';
% % modelObj        =   model_config.getModel(cable_set_id);
% % 
% % q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max; q_initial=modelObj.bodyModel.q_initial;
% % nsegvar= [5;5];  
% % uGrid           =   RayGridGeneration(q_begin,q_end,q_initial,nsegvar);
% % 
% % wsim            =   WorkspaceRayGeneration(modelObj,uGrid,[]);
% % wsim.run(0,0)
% % wsim.plotRayWorkspace([1,2])


% % %%%          4-4_CDPR_planar
% % 
% % model_config    =   DevModelConfig('4-4_CDPR_planar');   %    spatial7cable   BMArm_paper   BMArm_paper
% % cable_set_id    =   'original';
% % modelObj        =   model_config.getModel(cable_set_id);
% % 
% % q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max; q_initial=modelObj.bodyModel.q_initial;
% % nsegvar= [15;15;15];  
% % uGrid           =   RayGridGeneration(q_begin,q_end,q_initial,nsegvar);
% % 
% % % wsim            =   WorkspaceRayGeneration(modelObj,uGrid);
% % wsim            =   WorkspaceRayGeneration(modelObj,uGrid,[]);
% % wsim.run(0,0)
% % wsim.plotRayWorkspace([1,2,3])

%% Delta_planar
% model_config  = DevModelConfig('Delta_planar');
% cable_set_id  = 'original';
% modelObj      = model_config.getModel(cable_set_id);
% 
% q_begin       = modelObj.bodyModel.q_min;
% q_end         = modelObj.bodyModel.q_max; 
% q_initial     = modelObj.bodyModel.q_initial;
% nsegvar       = [5;5;5]; 
% safebuf      = 0.283;
% uGrid         =   RayGridGeneration(q_begin,q_end,q_initial,nsegvar);
% 
% wsim          =   WorkspaceRayGeneration(modelObj, uGrid, safebuf);
% wsim.run(0,0)
% wsim.plotRayWorkspace([1,2,3])

%% spatial ones

% % %%%        spatial7cable
% % 
% % model_config    =   DevModelConfig('spatial7cable');   %    spatial7cable     
% % cable_set_id    =   'original';
% % modelObj        =   model_config.getModel(cable_set_id);
% % 
% % q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max; q_initial=modelObj.bodyModel.q_initial;
% % % nsegvar= [5;5;5;0;0;0];  
% % nsegvar= [0;0;0;5;5;5];  
% % uGrid           =   RayGridGeneration(q_begin,q_end,q_initial,nsegvar);
% % 
% % % wsim            =   WorkspaceRayGeneration(modelObj,uGrid);
% % wsim            =   WorkspaceRayGeneration(modelObj,uGrid,0.03);
% % wsim.run(3,0)
% % % wsim.plotRayWorkspace([1,2,3])
% % wsim.plotRayWorkspace([4,5,6])


% % %% IFW_ghostModel
% % model_config  = DevModelConfig('IFW_ghostModel');
% % cable_set_id  = 'original';
% % modelObj      = model_config.getModel(cable_set_id);
% % 
% % q_begin       = modelObj.bodyModel.q_min;
% % q_end         = modelObj.bodyModel.q_max; 
% % q_initial     = modelObj.bodyModel.q_initial;
% % nsegvar       = [5;5;5;5;5;5]; 
% % safebuf       = 0.2; % due to 'safebuf>distance of two attachments on the end effector, so it's empty'
% % uGrid         =   RayGridGeneration(q_begin,q_end,q_initial,nsegvar);
% % 
% % wsim          =   WorkspaceRayGeneration(modelObj, uGrid, safebuf);
% % wsim.run(10e-4,0)
% % % wsim.run(0,0)
% % wsim.plotRayWorkspace([4,5,6])



%% Multilinks ones

% % %%          BMArm_paper
% % 
% % model_config    =   DevModelConfig('BMArm_paper');   %    spatial7cable     
% % cable_set_id    =   'original';
% % modelObj        =   model_config.getModel(cable_set_id);
% % 
% % q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max; q_initial=modelObj.bodyModel.q_initial;
% % nsegvar= [5;5;5;0];  
% % uGrid           =   RayGridGeneration(q_begin,q_end,q_initial,nsegvar);
% % 
% % % wsim            =   WorkspaceRayGeneration(modelObj,uGrid);
% % typeWS = 2;
% % wsim            =   WorkspaceRayGeneration(modelObj,uGrid, 0.1, typeWS);
% % wsim.run(0,0)
% % wsim.plotRayWorkspace([1,2,3])

%% MickMultiIFW
model_config  = DevModelConfig('MickMultiIFW');
cable_set_id  = 'original';
modelObj      = model_config.getModel(cable_set_id);

q_begin       = modelObj.bodyModel.q_min;
q_end         = modelObj.bodyModel.q_max; 
q_initial     = modelObj.bodyModel.q_initial;
nsegvar       = [0;5;5;5]; 
safebuf      = 0.8;
uGrid         =   RayGridGeneration(q_begin,q_end,q_initial,nsegvar);

typeWS = 2;
wsim          =   WorkspaceRayGeneration(modelObj, uGrid, safebuf,typeWS);
wsim.run(0,0)
wsim.plotRayWorkspace([1,2,3])

