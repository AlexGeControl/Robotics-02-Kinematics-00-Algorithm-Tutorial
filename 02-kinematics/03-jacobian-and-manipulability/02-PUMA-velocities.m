function [ v06, w06 ] = puma_velocities( thetas, thetadot )
    % Compute end-effector velocity at given configuration
    % Kinematics in Denavit-Hartenburg conventions are as follos:
    %
    % Link,    r, alpha, d, theta
    %    1,    0, -pi/2, a, theta1
    %    2,    b,     0, 0, -pi/2+theta2
    %    3,    c,     0, 0, +pi/2+theta3
    %    4,    0, -pi/2, 0, -pi/2+theta4
    %    5,    0,     0, d, theta5
    %
    % The inputs are as follows:
    % 1. thetas: The joint angles of the robot in radians - 1x5 matrix
    % 2. thetadot: The rate of change of joint angles of the robot in radians/sec - 1x5 matrix
    %
    % The output has 2 parts:
    % v05 - The linear velocity of frame 5 with respect to frame 0, expressed in frame 0.
    % w05 - The angular velocity of frame 5 with respect to frame 0, expressed in frame 0.
    % They are both 1x3 matrices of the form [x y z] for a vector xi + yj + zk

    % Initialize outputs:
    v06 = zeros(1, 3);
    w06 = zeros(1, 3);

    % Robot specifications:
    a = 13.0;
    b =  2.5;
    c =  8.0;
    d =  2.5;
    e =  8.0;
    f =  2.5;

    % Initialize transform:
    T = eye(4);
    z0 = [0,0,1]'; p0 = zeros(3,1);
    % q1:
    T = T * compute_dh_matrix( 0, +pi/2, +a, thetas(1,1));
    z1 = T(1:3,3); p1 = T(1:3,4);
    % q2:
    T = T * compute_dh_matrix(+c,     0, -b, thetas(1,2));
    z2 = T(1:3,3); p2 = T(1:3,4);
    % q3:
    T = T * compute_dh_matrix( 0, -pi/2, -d, thetas(1,3));
    T(1:3, 4) = T(1:3, 4) + e*T(1:3,3);
    z3 = T(1:3,3); p3 = T(1:3,4);
    % q4:
    T = T * compute_dh_matrix( 0, +pi/2,  0, thetas(1,4));
    z4 = T(1:3,3); p4 = T(1:3,4);
    % q5:
    T = T * compute_dh_matrix( 0, -pi/2,  0, thetas(1,5));
    z5 = T(1:3,3); p5 = T(1:3,4);
    % q6:
    T = T * compute_dh_matrix( 0,     0,  f, thetas(1,6));
    z6 = T(1:3,3); p6 = T(1:3,4);

    % Initialize Jacobian
    Jv = [cross(z0, (p6-p0)), cross(z1, (p6-p1)), cross(z2, (p6-p2)), cross(z3, (p6-p3)), cross(z4, (p6-p4)), cross(z5, (p6-p5))];
    Jw = [z0, z1, z2, z3, z4, z5];

    % Calculate speed:
    v06 = (Jv * thetadot')';
    w06 = (Jw * thetadot')';
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
