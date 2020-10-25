% Main file for simulation of Quadruped robot walking (kinematics only)
close all
clear
clc

leg1 = create_robot_leg('1', [pi/4, pi/4, -pi/2], [0, 0, 0], [0, 0.5, 0], [0, 0.5, 0]);
showdetails(leg1);

% Plot home configuration
figure;
show(leg1, homeConfiguration(leg1));
