clc
clear all
close all

model_config    =   ModelConfig('4-4_CDPR_planar');   %    spatial7cable   BMArm_paper   BMArm_paper
cable_set_id    =   'original';
modelObj        =   model_config.getModel(cable_set_id);



parfor it=1:20
    modelObj.update(rand(3,1), zeros(3,1), zeros(3,1),zeros(3,1))
end



