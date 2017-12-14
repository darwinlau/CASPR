
clc;  close all; warning off

%%%          2 DoF VSD

% Set up the model 
% 
% model_config    =   ModelConfig('2 DoF VSD');   %    spatial7cable   BMArm_paper   BMArm_paper
% cable_set_id    =   'basic';
% modelObj        =   model_config.getModel(cable_set_id);


% q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max; q_initial=modelObj.bodyModel.q_initial;  
% nsegvar= [25;25];      % number of discritization on each axis. if the user desire to ignor discritization on one axis its corresponding discritiaztion number can be set to zero
% uGrid           =   RayGridGeneration(q_begin,q_end,q_initial,nsegvar); 

% Set up the workspace simulator

% wsim            =   WorkspaceRayGeneration(modelObj,uGrid);
% wsim.run(2,0);    % the input can be empty or two digits. the first indicate the percentage of ray length which are ignored in workspace. 
                  % the second one is the readmode. if readmode is one then the result of computation in the data file is read without repeating the computation    
% wsim.plotRayWorkspace([1,2]);     % the inout is a 1x2 or 1x3 vector of axes for plotting.
% gsim   =  RayGraphGeneration(modelObj,uGrid,wsim); 
% gsim.run(1,0)       % the input can be empty or two digits. the first indicate weighted/unweighted graph. 0 means unweighted and 1 means weighted based on tension factor
%                     % the second one is the readmode. if readmode is one then the result of computation in the data file is read without repeating the computation    
% gsim.plotGraphWorkspace
% start_pose=[0.05,0.1] ;  end_pose=[0.42,0.63] ;
% [shortpath,listgrid,matpathvar,TFvect]=gsim.Path_Generation(start_pose,end_pose);


%          4-4_CDPR_planar
% 
model_config    =   DevModelConfig('4_4_CDPR_planar');   %    spatial7cable   BMArm_paper   BMArm_paper
cable_set_id    =   'original';
modelObj        =   model_config.getModel(cable_set_id);
% 
q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max; q_initial=modelObj.bodyModel.q_initial;
nsegvar= [25;25;25];  
uGrid           =   RayGridGeneration(q_begin,q_end,q_initial,nsegvar);
% 
wsim            =   WorkspaceRayGeneration(modelObj,uGrid);
wsim.run(2,0)            % the input can be empty or two digits. the first indicate the percentage of ray length which are ignored in workspace the second one is the readmode. if readmode is one then the result of computation in the data file is read without repeating the computation    
wsim.plotRayWorkspace([1,2,3])
% 
% gsim   = RayGraphGeneration(modelObj,uGrid,wsim); 
% 
% gsim.run(1,0)       % the input can be empty or two digits. the first indicate weighted/unweighted graph. 0 means unweighted and 1 means weighted based on tension factor the second one is the readmode. if readmode is one then the result of computation in the data file is read without repeating the computation    
% gsim.plotGraphWorkspace
% 
% start_pose=[3,2,2*pi/180] ;  end_pose=[7,3,-4*pi/180] ;
% [shortpath,listgrid,matpathvar,TFvect]=gsim.Path_Generation(start_pose,end_pose);
% 


%%          BMArm_paper
% 
% model_config    =   DevModelConfig('BMArm_paper');   %    spatial7cable     
% cable_set_id    =   'original';
% modelObj        =   model_config.getModel(cable_set_id);
% 
% q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max; q_initial=modelObj.bodyModel.q_initial;
% nsegvar= [30;30;30;0];  
% uGrid           =   RayGridGeneration(q_begin,q_end,q_initial,nsegvar);
% 
% wsim            =   WorkspaceRayGeneration(modelObj,uGrid);
% wsim.run(0,1)   ;     % the input can be empty or two digits. the first indicate the percentage of ray length which are ignored in workspace the second one is the readmode. if readmode is one then the result of computation in the data file is read without repeating the computation    
% wsim.plotRayWorkspace([1,2,3])
% gsim   =  RayGraphGeneration(modelObj,uGrid,wsim);
% gsim.run(1,1)       % the input can be empty or two digits. the first indicate weighted/unweighted graph. 0 means unweighted and 1 means weighted based on tension factor
%                     % the second one is the readmode. if readmode is one then the result of computation in the data file is read without repeating the computation    
% gsim.plotGraphWorkspace
% start_pose=[-0.3,-0.55,-0.65,0] ;  end_pose=[-0.2,-0.6,-0.7,0] ;
% [shortpath,listgrid,matpathvar,TFvect]=gsim.Path_Generation(start_pose,end_pose);


%%%        spatial7cable

% model_config    =   DevModelConfig('spatial7cable');   %    spatial7cable     
% cable_set_id    =   'original';
% modelObj        =   model_config.getModel(cable_set_id);
% 
% q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max; q_initial=modelObj.bodyModel.q_initial;
% nsegvar= [25;25;25;25;0;0];  
% uGrid           =   RayGridGeneration(q_begin,q_end,q_initial,nsegvar);
% 
% 
% 
% wsim            =   WorkspaceRayGeneration(modelObj,uGrid);
% wsim.run(0,0) ;                    % the input can be empty or two digits. the first indicate the percentage of ray length which are ignored in workspace the second one is the readmode. if readmode is one then the result of computation in the data file is read without repeating the computation                   

% wsim.plotRayWorkspace([1,2,3])

% gsim   = RayGraphGeneration(modelObj,uGrid,wsim);
% gsim.run(0,0)       % the input can be empty or two digits. the first indicate weighted/unweighted graph. 0 means unweighted and 1 means weighted based on tension factor
                    %the second one is the readmode. if readmode is one then the result of computation in the data file is read without repeating the computation    
% gsim.plotGraphWorkspace
% start_pose=[0.25,0.4,0.5,-11.4212*pi/180,11.4212*pi/180,0*pi/180] ;  end_pose=[0.75,0.75,0.3,-11.4212*pi/180,11.4212*pi/180,11.42*pi/180] ;
% start_pose=[0.35,0.4,0.5,-10*pi/180,0*pi/180,0*pi/180] ;  end_pose=[0.6,0.7,0.7,7*pi/180,0*pi/180,0*pi/180] ;
% 
% [shortpath,listgrid,matpathvar,TFvect]=gsim.Path_Generation(start_pose,end_pose);




% figure
% 
% subplot(2,3,1);
% plot(matpathvar(:,1));
% subplot(2,3,2);
% plot(matpathvar(:,2));
% subplot(2,3,3);
% plot(matpathvar(:,3));
% subplot(2,3,4);
% plot(matpathvar(:,4));
% subplot(2,3,5);
% plot(matpathvar(:,5));
% subplot(2,3,6);
% plot(matpathvar(:,6));
% figure
% plot(TFvect);



