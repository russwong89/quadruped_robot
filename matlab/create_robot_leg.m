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

function robot_leg = create_robot_leg(index, home_positions, base_to_shoulder_tform, shoulder_to_thigh_tform, thigh_to_calf_tform, calf_to_foot_tform)
    robot_leg = robotics.RigidBodyTree;
    robot_leg.Gravity = [0, 0. -9.81];
    robot_leg.DataFormat = 'row';

    % Body rotating around z-axis
    shoulder = robotics.RigidBody(append('shoulder', index));
    shoulder_joint = robotics.Joint(append('shoulder_joint', index), 'revolute');
    shoulder_joint.HomePosition = home_positions(1);
    shoulder_joint_tform = trvec2tform(base_to_shoulder_tform);
    setFixedTransform(shoulder_joint, shoulder_joint_tform);
    shoulder_joint.JointAxis = [0, 0, 1];
    shoulder.Joint = shoulder_joint;
    addBody(robot_leg, shoulder, 'base');

    % First leg segment
    thigh = robotics.RigidBody(append('thigh', index));
    thigh_joint = robotics.Joint(append('thigh_joint', index), 'revolute');
    thigh_joint.HomePosition = home_positions(2);
    thigh_joint_tform = trvec2tform(shoulder_to_thigh_tform);
    setFixedTransform(thigh_joint, thigh_joint_tform);
    thigh_joint.JointAxis = [1, 0, 0];
    thigh.Joint = thigh_joint;
    addBody(robot_leg, thigh, append('shoulder', index));

    % Second leg segment
    calf = robotics.RigidBody(append('calf', index));
    calf_joint = robotics.Joint(append('calf_joint', index), 'revolute');
    calf_joint.HomePosition = home_positions(3);
    calf_joint_tform = trvec2tform(thigh_to_calf_tform);
    setFixedTransform(calf_joint, calf_joint_tform);
    calf_joint.JointAxis = [1, 0, 0];
    calf.Joint = calf_joint;
    addBody(robot_leg, calf, append('thigh', index));

    % Foot (end effeector)
    foot = robotics.RigidBody(append('foot', index));
    setFixedTransform(foot.Joint, trvec2tform(calf_to_foot_tform));
    addBody(robot_leg, foot, append('calf', index));
end
