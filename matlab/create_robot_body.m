% https://www.mathworks.com/help/robotics/ug/build-a-robot-step-by-step.html
function robot_body = create_robot_body(body_to_shoulder_radius, shoulder_to_thigh_tform, thigh_to_calf_tform, calf_to_foot_tform)
    robot_body = robotics.RigidBodyTree;
    robot_body.Gravity = [0, 0, -9.81];
    robot_body.DataFormat = 'row';

    % Home configurations(1) are negated because Z-axis rotation convention
    % is that theta = 0 aligns with Y-axis
    leg1 = create_robot_leg('1', [-pi/4, pi/4, -pi/2], [body_to_shoulder_radius*sin(pi/4), body_to_shoulder_radius*cos(pi/4), 0], shoulder_to_thigh_tform, thigh_to_calf_tform, calf_to_foot_tform);
    leg2 = create_robot_leg('2', [-3*pi/4, pi/4, -pi/2], [body_to_shoulder_radius*sin(3*pi/4), body_to_shoulder_radius*cos(3*pi/4), 0], shoulder_to_thigh_tform, thigh_to_calf_tform, calf_to_foot_tform);
    leg3 = create_robot_leg('3', [3*pi/4, pi/4, -pi/2], [body_to_shoulder_radius*sin(-3*pi/4), body_to_shoulder_radius*cos(-3*pi/4), 0], shoulder_to_thigh_tform, thigh_to_calf_tform, calf_to_foot_tform);
    leg4 = create_robot_leg('4', [pi/4, pi/4, -pi/2], [body_to_shoulder_radius*sin(-pi/4), body_to_shoulder_radius*cos(-pi/4), 0], shoulder_to_thigh_tform, thigh_to_calf_tform, calf_to_foot_tform);

    addSubtree(robot_body, 'base', subtree(leg1, 'base'));
    addSubtree(robot_body, 'base', subtree(leg2, 'base'));
    addSubtree(robot_body, 'base', subtree(leg3, 'base'));
    addSubtree(robot_body, 'base', subtree(leg4, 'base'));
end