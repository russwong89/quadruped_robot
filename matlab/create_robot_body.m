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

% Good resource:
% https://www.mathworks.com/help/robotics/ug/build-a-robot-step-by-step.html
function robot_body = create_robot_body(body_to_shoulder_radius, shoulder_to_thigh_tform, thigh_to_calf_tform, calf_to_foot_tform, initial_thigh_angle, initial_calf_angle)
    robot_body = robotics.RigidBodyTree;
    robot_body.Gravity = [0, 0, -9.81];
    robot_body.DataFormat = 'row';

    % Home configurations(1) are negated because Z-axis rotation convention
    % is that theta = 0 aligns with Y-axis
    leg1 = create_robot_leg('1', [-pi/4, initial_thigh_angle, initial_calf_angle], [body_to_shoulder_radius*sin(pi/4), body_to_shoulder_radius*cos(pi/4), 0], shoulder_to_thigh_tform, thigh_to_calf_tform, calf_to_foot_tform);
    leg2 = create_robot_leg('2', [-3*pi/4, initial_thigh_angle, initial_calf_angle], [body_to_shoulder_radius*sin(3*pi/4), body_to_shoulder_radius*cos(3*pi/4), 0], shoulder_to_thigh_tform, thigh_to_calf_tform, calf_to_foot_tform);
    leg3 = create_robot_leg('3', [3*pi/4, initial_thigh_angle, initial_calf_angle], [body_to_shoulder_radius*sin(-3*pi/4), body_to_shoulder_radius*cos(-3*pi/4), 0], shoulder_to_thigh_tform, thigh_to_calf_tform, calf_to_foot_tform);
    leg4 = create_robot_leg('4', [pi/4, initial_thigh_angle, initial_calf_angle], [body_to_shoulder_radius*sin(-pi/4), body_to_shoulder_radius*cos(-pi/4), 0], shoulder_to_thigh_tform, thigh_to_calf_tform, calf_to_foot_tform);

    addSubtree(robot_body, 'base', subtree(leg1, 'base'));
    addSubtree(robot_body, 'base', subtree(leg2, 'base'));
    addSubtree(robot_body, 'base', subtree(leg3, 'base'));
    addSubtree(robot_body, 'base', subtree(leg4, 'base'));
end