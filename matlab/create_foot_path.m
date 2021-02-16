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

function [lift_path, drag_path] = create_foot_path(stride_type, stride_length, stride_arc_length, robot_radius, step_height, full_stride_time, time_delta)
    t = (time_delta:time_delta:full_stride_time/2)';
    switch stride_type
        case StrideTypes.MOVE_FORWARD
            % Parabolic path for lifting foot
            lift_path_x = t / t(end) * stride_length;
            lift_path_y = zeros(length(t), 1);
            lift_path_z = -4*step_height/(stride_length*stride_length)*(lift_path_x - stride_length/2).*(lift_path_x - stride_length/2) + step_height;
            % Linear path for dragging foot along ground
            drag_path_x = -t / t(end) * stride_length;
            drag_path_y = zeros(length(t), 1);
            drag_path_z = zeros(length(t), 1);
            lift_path = [lift_path_x, lift_path_y, lift_path_z];
            drag_path = [drag_path_x, drag_path_y, drag_path_z];
        case StrideTypes.MOVE_BACKWARD
            % Same as MOVE_FORWARD but reverse x direction
            lift_path_x = -t / t(end) * stride_length;
            lift_path_y = zeros(length(t), 1);
            lift_path_z = -4*step_height/(stride_length*stride_length)*(-lift_path_x - stride_length/2).*(-lift_path_x - stride_length/2) + step_height;
            drag_path_x = t / t(end) * stride_length;
            drag_path_y = zeros(length(t), 1);
            drag_path_z = zeros(length(t), 1);
            lift_path = [lift_path_x, lift_path_y, lift_path_z];
            drag_path = [drag_path_x, drag_path_y, drag_path_z];
        case StrideTypes.ROTATE_CCW
            % define in cylindrical coordinates
            lift_path_theta = t / t(end) * stride_arc_length / robot_radius;
            lift_path_rho = zeros(length(t), 1);
            lift_path_z = -4*step_height/(stride_arc_length*stride_arc_length)*(lift_path_theta*robot_radius - stride_arc_length/2).*(lift_path_theta*robot_radius - stride_arc_length/2) + step_height;
            drag_path_theta = -t / t(end) * stride_arc_length / robot_radius;
            drag_path_rho = zeros(length(t), 1);
            drag_path_z = zeros(length(t), 1);
            lift_path = [lift_path_theta, lift_path_rho, lift_path_z];
            drag_path = [drag_path_theta, drag_path_rho, drag_path_z];
        case StrideTypes.ROTATE_CW
            % Same as ROTATE_CCW but reverse theta
            lift_path_theta = -t / t(end) * stride_arc_length / robot_radius;
            lift_path_rho = zeros(length(t), 1);
            lift_path_z = -4*step_height/(stride_arc_length*stride_arc_length)*(-lift_path_theta*robot_radius - stride_arc_length/2).*(-lift_path_theta*robot_radius - stride_arc_length/2) + step_height;
            drag_path_theta = t / t(end) * stride_arc_length / robot_radius;
            drag_path_rho = zeros(length(t), 1);
            drag_path_z = zeros(length(t), 1);
            lift_path = [lift_path_theta, lift_path_rho, lift_path_z];
            drag_path = [drag_path_theta, drag_path_rho, drag_path_z];
        otherwise
            disp('ERROR: unhandled stride_type')
            return
    end
end