
clc; clear;  close all; warning off

% % %%          2 DoF VSD
% % 
% % model_config    =   ModelConfig('2 DoF VSD');   %    spatial7cable   BMArm_paper   BMArm_paper
% % cable_set_id    =   'basic';
% % modelObj        =   model_config.getModel(cable_set_id);
% % 
% % q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max; q_initial=modelObj.bodyModel.q_initial;
% % nsegvar= [25;25];  
% % uGrid           =   RayGridGeneration(q_begin,q_end,q_initial,nsegvar);
% % 
% % wsim            =   WorkspaceRayGeneration(modelObj,uGrid,[]);
% % wsim.run(0,0)
% % wsim.plotRayWorkspace([1,2])


% % %%%          4-4_CDPR_planar
% % 
% % model_config    =   ModelConfig('4-4_CDPR_planar');   %    spatial7cable   BMArm_paper   BMArm_paper
% % cable_set_id    =   'original';
% % modelObj        =   model_config.getModel(cable_set_id);
% % 
% % q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max; q_initial=modelObj.bodyModel.q_initial;
% % nsegvar= [5;5;5];  
% % uGrid           =   RayGridGeneration(q_begin,q_end,q_initial,nsegvar);
% % 
% % % wsim            =   WorkspaceRayGeneration(modelObj,uGrid);
% % wsim            =   WorkspaceRayGeneration(modelObj,uGrid,[]);
% % wsim.run(0,0)
% % wsim.plotRayWorkspace([1,2,3])


% %% MickMultiIFW
% model_config  = ModelConfig('MickMultiIFW');
% cable_set_id  = 'original';
% modelObj      = model_config.getModel(cable_set_id);
% 
% q_begin       = modelObj.bodyModel.q_min;
% q_end         = modelObj.bodyModel.q_max; 
% q_initial     = modelObj.bodyModel.q_initial;
% nsegvar       = [0;5;5;5]; 
% safebuf      = 0.8;
% uGrid         =   RayGridGeneration(q_begin,q_end,q_initial,nsegvar);
% 
% wsim          =   WorkspaceRayGeneration(modelObj, uGrid, safebuf);
% wsim.run(0,0)
% wsim.plotRayWorkspace([1,2,3])

% %% IFW_ghostModel
% model_config  = ModelConfig('IFW_ghostModel');
% cable_set_id  = 'original';
% modelObj      = model_config.getModel(cable_set_id);
% 
% q_begin       = modelObj.bodyModel.q_min;
% q_end         = modelObj.bodyModel.q_max; 
% q_initial     = modelObj.bodyModel.q_initial;
% nsegvar       = [5;5;5;5;5;5]; 
% safebuf      = 0.4;
% uGrid         =   RayGridGeneration(q_begin,q_end,q_initial,nsegvar);
% 
% wsim          =   WorkspaceRayGeneration(modelObj, uGrid, safebuf);
% wsim.run(0,0)
% wsim.plotRayWorkspace([1,2,3])

% %% Delta_planar
% model_config  = ModelConfig('Delta_planar');
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


% % %%          BMArm_paper
% % 
% % model_config    =   ModelConfig('BMArm_paper');   %    spatial7cable     
% % cable_set_id    =   'original';
% % modelObj        =   model_config.getModel(cable_set_id);
% % 
% % q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max; q_initial=modelObj.bodyModel.q_initial;
% % nsegvar= [5;5;5;0];  
% % uGrid           =   RayGridGeneration(q_begin,q_end,q_initial,nsegvar);
% % 
% % % wsim            =   WorkspaceRayGeneration(modelObj,uGrid);
% % wsim            =   WorkspaceRayGeneration(modelObj,uGrid, 0.1);
% % wsim.run(0,0)
% % wsim.plotRayWorkspace([1,2,3])

%%%        spatial7cable

model_config    =   ModelConfig('spatial7cable');   %    spatial7cable     
cable_set_id    =   'original';
modelObj        =   model_config.getModel(cable_set_id);

q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max; q_initial=modelObj.bodyModel.q_initial;
% nsegvar= [5;5;5;0;0;0];  
nsegvar= [0;0;0;5;5;5];  
uGrid           =   RayGridGeneration(q_begin,q_end,q_initial,nsegvar);

% wsim            =   WorkspaceRayGeneration(modelObj,uGrid);
wsim            =   WorkspaceRayGeneration(modelObj,uGrid,0.03);
wsim.run(3,0)
% wsim.plotRayWorkspace([1,2,3])
wsim.plotRayWorkspace([4,5,6])










