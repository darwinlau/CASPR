clc;  close all; warning off; clear all;

% Set up the model  
% '4_4_CDPR_planar'
segment_number = [20,40,60,80,100,200];
model_config    =   DevModelConfig('4_4_CDPR_planar'); 
cable_set_id    =   'original';
% Generate the model
modelObj        =   model_config.getModel(cable_set_id,ModelModeType.COMPILED);
q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max;

% Loop through the different segment numbers
for i = 1:6
    % Set the grid
    nsegvar = [segment_number(i) segment_number(i) segment_number(i)]';
    uGrid           =   UniformGrid(q_begin,q_end,(q_end-q_begin)./(nsegvar-1),'step_size');
    % Workspace settings and conditions
    w_condition     =   {WorkspaceRayConditionBase.CreateWorkspaceRayCondition(WorkspaceRayConditionType.WRENCH_CLOSURE,100/(segment_number(i)-1),modelObj)};
    opt             =   RayWorkspaceSimulatorOptions(false,false);
    % Start the simulation
    disp('Start Setup Simulation');
    wsim            =   RayWorkspaceSimulator(modelObj,uGrid,opt);

    % Run the simulation
    disp('Start Running Simulation');
    wsim.run(w_condition,[])
    complete_string = ['Simulation completed for discretisation: ',segment_number(i)];
    sprintf(complete_string)
    ray_string = ['Ray construction time: ',num2str(wsim.comp_time.ray_construction)];
    sprintf(ray_string)
    graph_string = ['Graph construction time: ',num2str(wsim.comp_time.graph_construction)];
    sprintf(graph_string)
    total_string = ['Total construction time: ',num2str(wsim.comp_time.total)];
    sprintf(total_string)
    clear wsim;
end
