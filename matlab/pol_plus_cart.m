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

function ret = pol_plus_cart(pol, cart)
    [th2, rh2, z2] = cart2pol(cart(1), cart(2), cart(3));
    [x1, y1, z1] = pol2cart(pol(1) + th2, pol(2) + rh2, pol(3) + z2);
    ret = [x1, y1, z1];
end