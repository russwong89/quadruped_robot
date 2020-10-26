% Main file for simulation of Quadruped robot walking (kinematics only)
close all
clear
clc

% -------- PARAMETERS -------- %
% Robot params
ROBOT_RADIUS = 0.3;
ROBOT_THIGH_LENGTH = 0.5;
ROBOT_CALF_LENGTH = 0.5;

% Create robot model
robot = create_robot_body(ROBOT_RADIUS, [0, 0, 0], [0, ROBOT_THIGH_LENGTH, 0], [0, ROBOT_CALF_LENGTH, 0]);
showdetails(robot);

figure;
show(robot, homeConfiguration(robot));
