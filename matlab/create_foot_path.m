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