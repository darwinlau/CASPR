% Script file to show how to simply load a robot
%
% Author        : Darwin LAU
% Created       : 2016
% Description    :

% Clear the variables, command window, and all windows
clear; clc; close all;

% Set up the type of model:
model_config = ModelConfig(ModelConfigType.M_SEGESTA);

% The SystemModel is created
cdpr = model_config.getModel(model_config.defaultCableSetId);

% Plot to see the robot
MotionSimulator.PlotFrame(cdpr, model_config.displayRange);
