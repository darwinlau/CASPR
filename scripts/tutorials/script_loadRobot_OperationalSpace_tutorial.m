% Script file to show how to simply load a robot
% 1) Set up the type of model:
model_config = ModelConfig('BMArm');
% 2) Get the model for a given cable configuration
cdpr = model_config.getModel(model_config.defaultCableSetId,model_config.defaultOperationalSetId);
% 3) Update the pose of the robot
cdpr.update([pi/4; 0; 0; -pi/4], zeros(4,1), zeros(4,1), zeros(4,1));
% 4) Plot to see the robot
MotionSimulatorBase.PlotFrame(cdpr, model_config.displayRange, model_config.viewAngle);
