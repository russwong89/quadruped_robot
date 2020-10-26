% Main file for simulation of Quadruped robot walking (kinematics only)
close all
clear
clc

robot = create_robot_body(0.3, [0, 0, 0], [0, 0.5, 0], [0, 0.5, 0]);
showdetails(robot);

% Plot home configuration
figure;
show(robot, homeConfiguration(robot));
