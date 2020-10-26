function ret = get_pos(robot, q, body_name)
    T = getTransform(robot, q, body_name);
    ret = (T(1:3, 4))';
end