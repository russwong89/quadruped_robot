function save_qs_to_file(qs)
    % Convert from rad to deg
    qs = radtodeg(qs);

    % Copy initial angles to the beginning
    q_init = qs(end,:);
    qs = [q_init; qs];

    % Convert to servo angle reference frame
    % Set all angles to zero-referenced
    qs = qs - qs(1,:);
    % Reverse directions of joint angles as needed
    qs(:,1) = qs(:,1)*-1;

    % Add joint angle offsets
    qs(:,1) = qs(:,1)+90;
    qs(:,2) = qs(:,2)+90;
    qs(:,3) = qs(:,3)+225+q_init(3);

    % Round to nearest integer
    qs = round(qs);
    writematrix(qs, 'qs.csv');
end