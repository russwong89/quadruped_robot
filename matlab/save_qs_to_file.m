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

function save_qs_to_file(qs)
    % Convert from rad to deg
    qs = rad2deg(qs);

    % Copy initial angles to the beginning
    q_init = qs(end,:);
    qs = [q_init; qs];

    % Convert to servo angle reference frame
    % Set all angles to zero-referenced
    qs = qs - qs(1,:);
    % Reverse directions of joint angles as needed
    qs(:,1) = qs(:,1)*-1;
    qs(:,4) = qs(:,4)*-1;
    qs(:,7) = qs(:,7)*-1;
    qs(:,10) = qs(:,10)*-1;

    % Add joint angle offsets
    qs(:,1) = qs(:,1)+90;
    qs(:,2) = qs(:,2)+90;
    qs(:,3) = qs(:,3)+225+q_init(3);
    qs(:,4) = qs(:,4)+90;
    qs(:,5) = qs(:,5)+90;
    qs(:,6) = qs(:,6)+225+q_init(6);
    qs(:,7) = qs(:,7)+90;
    qs(:,8) = qs(:,8)+90;
    qs(:,9) = qs(:,9)+225+q_init(9);
    qs(:,10) = qs(:,10)+90;
    qs(:,11) = qs(:,11)+90;
    qs(:,12) = qs(:,12)+225+q_init(12);

    % Round to nearest integer
    qs = round(qs);
    writematrix(qs, 'temp.csv');
end