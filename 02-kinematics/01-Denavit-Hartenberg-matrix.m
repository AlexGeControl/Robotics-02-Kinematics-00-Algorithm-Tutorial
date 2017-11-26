function A = compute_dh_matrix(r, alpha, d, theta)
    % Compute Denavit-Hartenburg rigid motion matrix given its params
    %
    % theta: rotation along z_i-1 from x_i-1 to x_i
    % d:     translation along z_i-1 from origin frame i-1 to frame i
    % alpha: rotation along x_i from z_i-1 to z_i
    % r:     translation along x_i from origin of frame i-1 to frame i
    %

    % a. rotation theta:
    rot_z_theta = [
        cos(theta),-sin(theta), 0, 0;
        sin(theta), cos(theta), 0, 0;
                 0,          0, 1, 0;
                 0,          0, 0, 1
    ];
    % b. translation d:
    trans_z_d = [
             eye(3), [0, 0, d]';
        zeros(1, 3), 1
    ];
    % c. translation a:
    trans_x_r = [
             eye(3), [r, 0, 0]';
        zeros(1, 3), 1
    ];
    % d. rotation alpha:
    rot_x_alpha = [
        1,          0,           0, 0;
        0, cos(alpha), -sin(alpha), 0;
        0, sin(alpha),  cos(alpha), 0;
        0,          0,           0, 1;
    ];

    A = rot_z_theta * trans_z_d * trans_x_r * rot_x_alpha;
end
