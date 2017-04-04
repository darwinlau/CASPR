% Script file to show how to simply load a robot
% 1) Set up the type of model:
model_config = ModelConfig('SEGESTA');
% 2) Get the model for a given cable configuration
cdpr = model_config.getModel(model_config.defaultCableSetId);
% 3) Update the pose of the robot
cdpr.update([0; 0; 0.5; 0; 0; 0], zeros(6,1), zeros(6,1), zeros(6,1));
% 4) Plot to see the robot
MotionSimulator.PlotFrame(cdpr, model_config.displayRange, model_config.viewAngle);
