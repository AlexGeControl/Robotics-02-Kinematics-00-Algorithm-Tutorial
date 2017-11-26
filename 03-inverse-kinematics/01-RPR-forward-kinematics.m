function [ pos, R ] = RPR_fk( theta1, d2, theta3 )
    % Compute coordinates of intermediate frame origins and end-effector reference points
    % inside world frame, given:
    %
    % a. joint values: theta1, d2, theta3
    %
    % Kinematics in Denavit-Hartenburg conventions are as follos:
    %
    % Link,    r,   alpha,  d,          theta
    %    1,    0, -3/4*pi,  a,         theta1
    %    2,    0, -1/2*pi, d2,        -1/2*pi
    %    3,    0, +1/2*pi,  0, +1/4*pi+theta3
    %
    % After that
    % 1. +1/2*pi rotation along z
    % 2. b translation along z
    % should be applied before reaching final end-effector frame

    % Initialize reference points & posit:
    pos = zeros(4, 3);
    R = eye(3);

    % Arm specifications:
    a = 10;
    b =  5;

    % A_01:
    A_01 = compute_dh_matrix(0, -3/4*pi,  a,         theta1);
    % A_12:
    A_12 = compute_dh_matrix(0, -1/2*pi, d2,        -1/2*pi);
    % A_23:
    A_23 = compute_dh_matrix(0, +1/2*pi,  0, +1/4*pi+theta3);

    % a. origin of frame 0 is already set:
    % b. origin of frame 1:
    T = A_01;
    pos( 2, :) = (T(1:3,4) / T(4,4))';
    % c. origin of frame 2:
    T = T * A_12;
    pos( 3, :) = (T(1:3,4) / T(4,4))';
    % d. origin of frame 3:
    T = T * A_23 * [
      0, -1, 0, 0;
      1,  0, 0, 0;
      0,  0, 1, b;
      0,  0, 0, 1;
    ];
    pos( 4, :) = (T(1:3,4) / T(4,4))';

    R = T(1:3, 1:3);
end

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
