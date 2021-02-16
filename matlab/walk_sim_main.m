% Copyright (c) 2021 Russell Wong
%
% “Commons Clause” License Condition v1.0
%
% The Software is provided to you by the Licensor under the License, as defined
% below, subject to the following condition.
%
% Without limiting other conditions in the License, the grant of rights under
% the License will not include, and the License does not grant to you, the right
% to Sell the Software.
%
% For purposes of the foregoing, “Sell” means practicing any or all of the rights
% granted to you under the License to provide to third parties, for a fee or other
% consideration (including without limitation fees for hosting or consulting/
% support services related to the Software), a product or service whose value derives,
% entirely or substantially, from the functionality of the Software. Any license
% notice or attribution required by the License must also include this Commons Clause
% License Condition notice.
%
% Software: https://github.com/russwong89/quadruped_robot
%
% License: MIT License with Commons Clause License Condition
% (https://github.com/russwong89/quadruped_robot/blob/main/LICENSE.md)
%
% Licensor: Russell Wong (russwong89@gmail.com)

% Main file for simulation of Quadruped robot walking (kinematics only)
close all
clear
clc

% -------- PARAMETERS -------- %
% Robot params
ROBOT_RADIUS = 0.10106601718;
SHOULDER_TO_THIGH_TFORM = [0.0144, 0.0258, -0.0144];
THIGH_TO_CALF_TFORM = [0.0231, 0.0998, -0.0001];
CALF_TO_FOOT_TFORM = [-0.0373, 0.1700, 0.0496];
INITIAL_THIGH_ANGLE = pi/4 - atan(THIGH_TO_CALF_TFORM(3) / THIGH_TO_CALF_TFORM(2));
GROUND_DIST= 0.092; % Distance from ground to shoulder joint
% Calculate calf angle so that home configuration touches the ground
projected_calf_length_yz = sqrt(CALF_TO_FOOT_TFORM(2)*CALF_TO_FOOT_TFORM(2) + CALF_TO_FOOT_TFORM(3)*CALF_TO_FOOT_TFORM(3));
projected_thigh_height_yz = sqrt(THIGH_TO_CALF_TFORM(2)*THIGH_TO_CALF_TFORM(2) + THIGH_TO_CALF_TFORM(3)*THIGH_TO_CALF_TFORM(3))*sin(INITIAL_THIGH_ANGLE + atan(THIGH_TO_CALF_TFORM(3) / THIGH_TO_CALF_TFORM(2)));
calf_joint_height_above_ground = GROUND_DIST + SHOULDER_TO_THIGH_TFORM(3) + projected_thigh_height_yz;
INITIAL_CALF_ANGLE = -(INITIAL_THIGH_ANGLE + atan(THIGH_TO_CALF_TFORM(3) / THIGH_TO_CALF_TFORM(2)) + pi/2 - acos(calf_joint_height_above_ground / projected_calf_length_yz) + atan(CALF_TO_FOOT_TFORM(3) / CALF_TO_FOOT_TFORM(2)));

% Trajectory params
STRIDE_TYPE = StrideTypes.MOVE_FORWARD;
FULL_STRIDE_LENGTH = 0.07;
FULL_STRIDE_ARC_LENGTH = 0.02;
STEP_HEIGHT = 0.1;
FULL_STRIDE_TIME = 1;
TIME_DELTA = 0.025;
NUM_CYCLES = 3;

% Simulation Params
FPS = 30;

% Create robot model
robot = create_robot_body(ROBOT_RADIUS, SHOULDER_TO_THIGH_TFORM, THIGH_TO_CALF_TFORM, CALF_TO_FOOT_TFORM, INITIAL_THIGH_ANGLE, INITIAL_CALF_ANGLE);
showdetails(robot);

figure;
show(robot, homeConfiguration(robot));
hold on;
temp = get_pos(robot, homeConfiguration(robot), 'foot1');
% foot_rad = temp(1)*sqrt(2);
% ezplot(@(x,y) (x).^2 + (y).^2 - foot_rad^2);

% Create relative paths
[lift_path, drag_path] = create_foot_path(STRIDE_TYPE, FULL_STRIDE_LENGTH, FULL_STRIDE_ARC_LENGTH, ROBOT_RADIUS, STEP_HEIGHT, FULL_STRIDE_TIME, TIME_DELTA);

% Inverse Kinematics
q0 = homeConfiguration(robot);
num_dof = length(q0);
num_timesteps_per_path = size(lift_path, 1);
qs = zeros(NUM_CYCLES*2*num_timesteps_per_path, num_dof);
ik = robotics.InverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 1];
for i = 1:NUM_CYCLES
    % First half-cycle: lift legs 1 and 3
    foot1_start_pos = get_pos(robot, q0, 'foot1');
    foot2_start_pos = get_pos(robot, q0, 'foot2');
    foot3_start_pos = get_pos(robot, q0, 'foot3');
    foot4_start_pos = get_pos(robot, q0, 'foot4');
    q_init = q0; 
    for j = 1:num_timesteps_per_path
        % Define destination points for each foot
        if STRIDE_TYPE == StrideTypes.MOVE_FORWARD || STRIDE_TYPE == StrideTypes.MOVE_BACKWARD
            % Rectangular coordinates
            foot1_dest_pos = lift_path(j,:) + foot1_start_pos;
            foot2_dest_pos = drag_path(j,:) + foot2_start_pos;
            foot3_dest_pos = lift_path(j,:) + foot3_start_pos;
            foot4_dest_pos = drag_path(j,:) + foot4_start_pos;
        else
            % Cylindrical coordinates
            foot1_dest_pos = pol_plus_cart(lift_path(j,:), foot1_start_pos);
            foot2_dest_pos = pol_plus_cart(drag_path(j,:), foot2_start_pos);
            foot3_dest_pos = pol_plus_cart(lift_path(j,:), foot3_start_pos);
            foot4_dest_pos = pol_plus_cart(drag_path(j,:), foot4_start_pos);
        end

        % Run IK for each foot
        q_sol1 = ik('foot1', trvec2tform(foot1_dest_pos), weights, q_init); % indices 1:3 are relevant to foot1
        q_sol2 = ik('foot2', trvec2tform(foot2_dest_pos), weights, q_init); % indices 4:6 are relevant to foot2
        q_sol3 = ik('foot3', trvec2tform(foot3_dest_pos), weights, q_init); % indices 7:9 are relevant to foot3
        q_sol4 = ik('foot4', trvec2tform(foot4_dest_pos), weights, q_init); % indices 10:12 are relevant to foot4

        % Add solution to the qs solution matrix
        q_sol = [q_sol1(1:3), q_sol2(4:6), q_sol3(7:9), q_sol4(10:12)];
        idx = 2*num_timesteps_per_path*(i-1)+j;
        qs(idx,:) = q_sol;
        q_init = q_sol;
    end

    % Second half-cycle: lift legs 2 and 4
    foot1_start_pos = get_pos(robot, q_init, 'foot1');
    foot2_start_pos = get_pos(robot, q_init, 'foot2');
    foot3_start_pos = get_pos(robot, q_init, 'foot3');
    foot4_start_pos = get_pos(robot, q_init, 'foot4');
    for j = 1:num_timesteps_per_path
        % Define destination points for each foot
        if STRIDE_TYPE == StrideTypes.MOVE_FORWARD || STRIDE_TYPE == StrideTypes.MOVE_BACKWARD
            % Rectangular coordinates
            foot1_dest_pos = drag_path(j,:) + foot1_start_pos;
            foot2_dest_pos = lift_path(j,:) + foot2_start_pos;
            foot3_dest_pos = drag_path(j,:) + foot3_start_pos;
            foot4_dest_pos = lift_path(j,:) + foot4_start_pos;
        else
            % Cylindrical coordinates
            foot1_dest_pos = pol_plus_cart(drag_path(j,:), foot1_start_pos);
            foot2_dest_pos = pol_plus_cart(lift_path(j,:), foot2_start_pos);
            foot3_dest_pos = pol_plus_cart(drag_path(j,:), foot3_start_pos);
            foot4_dest_pos = pol_plus_cart(lift_path(j,:), foot4_start_pos);
%             foot4_dest_pos = [1, 2, 3, 4];
        end

        % Run IK for each foot
        q_sol1 = ik('foot1', trvec2tform(foot1_dest_pos), weights, q_init); % indices 1:3 are relevant to foot1
        q_sol2 = ik('foot2', trvec2tform(foot2_dest_pos), weights, q_init); % indices 4:6 are relevant to foot2
        q_sol3 = ik('foot3', trvec2tform(foot3_dest_pos), weights, q_init); % indices 7:9 are relevant to foot3
        q_sol4 = ik('foot4', trvec2tform(foot4_dest_pos), weights, q_init); % indices 10:12 are relevant to foot4

        % Add solution to the qs solution matrix
        q_sol = [q_sol1(1:3), q_sol2(4:6), q_sol3(7:9), q_sol4(10:12)];
        idx = 2*num_timesteps_per_path*(i-1)+j+num_timesteps_per_path;
        qs(idx,:) = q_sol;
        q_init = q_sol;
    end
end

save_qs_to_file(qs);

% Plotting and simulation
fps_rate = rateControl(FPS);
while true
    w = 0;
    while w == 0
        w = waitforbuttonpress;
    end
    init_campos = campos;
    for i = 1:size(qs,1)
        show(robot, qs(i,:), 'PreservePlot', false);
        hold on
        campos(init_campos);
%         ezplot(@(x,y) (x).^2 + (y).^2 - foot_rad^2)
        drawnow
        waitfor(fps_rate);
    end
    min(qs(:,2))
    max(qs(:,2))
    min(qs(:,5))
    max(qs(:,5))
    min(qs(:,8))
    max(qs(:,8))
    min(qs(:,11))
    max(qs(:,11))
    
end
