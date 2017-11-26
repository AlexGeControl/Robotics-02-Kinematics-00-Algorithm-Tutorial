function [ pos ] = lynx_fk( theta1, theta2, theta3, theta4, theta5, g )
    % Compute coordinates of intermediate frame origins and end-effector reference points
    % inside world frame, given:
    %
    % a. joint angles: theta1, theta2, theta3, theta4, theta5
    % b. distance between gripper pads: g
    %
    % Kinematics in Denavit-Hartenburg conventions are as follos:
    %
    % Link,    r, alpha, d, theta
    %    1,    0, -pi/2, a, theta1
    %    2,    b,     0, 0, -pi/2+theta2
    %    3,    c,     0, 0, +pi/2+theta3
    %    4,    0, -pi/2, 0, -pi/2+theta4
    %    5,    0,     0, d, theta5
    %
    % The output must contain 10 positions of various points along the robot arm as specified
    % in the question.

    % Initialize reference points:
    pos = zeros(10, 3);

    % Arm specifications:
    a = 3.000;
    b = 5.750;
    c = 7.375;
    d = 4.125;
    e = 1.125;

    % A_01:
    A_01 = compute_dh_matrix(0, -pi/2, a, theta1      );
    % A_12:
    A_12 = compute_dh_matrix(b,     0, 0, -pi/2+theta2);
    % A_23:
    A_23 = compute_dh_matrix(c,     0, 0, +pi/2+theta3);
    % A_34:
    A_34 = compute_dh_matrix(0, -pi/2, 0, -pi/2+theta4);
    % A_45:
    A_45 = compute_dh_matrix(0,     0, d, theta5      );

    T = A_01;
    pos( 2, :) = (T(1:3,4) / T(4,4))';
    T = T * A_12;
    pos( 3, :) = (T(1:3,4) / T(4,4))';
    T = T * A_23;
    pos( 4, :) = (T(1:3,4) / T(4,4))';
    T = T * A_34;
    pos( 5, :) = (T(1:3,4) / T(4,4))';
    T = T * A_45;

    X = [
        0, 0, -e, 1;
      g/2, 0, -e, 1;
     -g/2, 0, -e, 1;
      g/2, 0,  0, 1;
     -g/2, 0,  0, 1;
    ]';

    X_world = T * X;

    pos( 6, :) = (X_world(1:3, 1) / X_world(4, 1))';
    pos( 7, :) = (X_world(1:3, 2) / X_world(4, 2))';
    pos( 8, :) = (X_world(1:3, 3) / X_world(4, 3))';
    pos( 9, :) = (X_world(1:3, 4) / X_world(4, 4))';
    pos(10, :) = (X_world(1:3, 5) / X_world(4, 5))';
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
