function [pos, R] = puma_fk(theta1, theta2, theta3, theta4, theta5, theta6)
    % Compute coordinates of end-effector reference point inside world frame, given:
    %
    % a. joint variables: theta1, theta2, theta3, theta4, theta5, theta6
    %
    % Kinematics in Denavit-Hartenburg conventions are as follos:
    %
    % Wrist center:
    % Link,     r, alpha, d, theta
    %    1,     0, +pi/2, +a, theta1
    %    2,    +c,     0, -b, theta2
    %    3,     0, -pi/2, -d, theta3
    %
    % [ cos(theta1)*cos(theta2)*cos(theta3) - cos(theta1)*sin(theta2)*sin(theta3), -sin(theta1), - cos(theta1)*cos(theta2)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2), 8*cos(theta1)*cos(theta2) - 5*sin(theta1)]
    % [ cos(theta2)*cos(theta3)*sin(theta1) - sin(theta1)*sin(theta2)*sin(theta3),  cos(theta1), - cos(theta2)*sin(theta1)*sin(theta3) - cos(theta3)*sin(theta1)*sin(theta2), 5*cos(theta1) + 8*cos(theta2)*sin(theta1)]
    % [                         cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2),            0,                           cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3),                        8*sin(theta2) + 13]
    % [                                                                         0,            0,                                                                           0,                                         1]
    %
    % With additional +e translation along z3 axis
    %
    % [ cos(theta1)*cos(theta2)*cos(theta3) - cos(theta1)*sin(theta2)*sin(theta3), -sin(theta1), - cos(theta1)*cos(theta2)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2), -5*sin(theta1) + 8*cos(theta1)*cos(theta2) - 8*cos(theta1)*cos(theta2)*sin(theta3) - 8*cos(theta1)*sin(theta2)*cos(theta3)]
    % [ cos(theta2)*cos(theta3)*sin(theta1) - sin(theta1)*sin(theta2)*sin(theta3),  cos(theta1), - cos(theta2)*sin(theta1)*sin(theta3) - cos(theta3)*sin(theta1)*sin(theta2), +5*cos(theta1) + 8*sin(theta1)*cos(theta2) - 8*sin(theta1)*cos(theta2)*sin(theta3) - 8*sin(theta1)*sin(theta2)*cos(theta3)]
    % [                         cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2),            0,                           cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3),                                                 8*sin(theta2) + 8*cos(theta2)*cos(theta3) - 8*sin(theta2)*sin(theta3) + 13]
    % [                                                                         0,            0,                                                                           0,                                                                                                                          1]
    %
    % Wrist orientation:
    % Z-Y-Z rotation
    % With additional +f translation along z6 axis
    % Link,     r, alpha, d, theta
    %    4,     0, +pi/2,  0, theta4
    %    5,     0, -pi/2,  0, theta5
    %    6,     0,     0,  f, theta6
    %
    % [ cos(theta4)*cos(theta5)*cos(theta6) - sin(theta4)*sin(theta6), - cos(theta6)*sin(theta4) - cos(theta4)*cos(theta5)*sin(theta6), -cos(theta4)*sin(theta5), -(5*cos(theta4)*sin(theta5))/2]
    % [ cos(theta4)*sin(theta6) + cos(theta5)*cos(theta6)*sin(theta4),   cos(theta4)*cos(theta6) - cos(theta5)*sin(theta4)*sin(theta6), -sin(theta4)*sin(theta5), -(5*sin(theta4)*sin(theta5))/2]
    % [                                       cos(theta6)*sin(theta5),                                        -sin(theta5)*sin(theta6),              cos(theta5),              (5*cos(theta5))/2]
    % [                                                             0,                                                               0,                        0,                              1]
    %
    % The output must contain:
    % a. End effector position of the robot arm
    % b. The rotation matrix representing the rotation from frame 6 to frame 0
    %
    % [ (cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3))*(sin(theta4)*sin(theta6) - cos(theta4)*cos(theta5)*cos(theta6)) - sin(theta1)*(cos(theta4)*sin(theta6) + cos(theta5)*cos(theta6)*sin(theta4)) - cos(theta6)*sin(theta5)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)), (cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3))*(cos(theta6)*sin(theta4) + cos(theta4)*cos(theta5)*sin(theta6)) - sin(theta1)*(cos(theta4)*cos(theta6) - cos(theta5)*sin(theta4)*sin(theta6)) + sin(theta5)*sin(theta6)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)), sin(theta1)*sin(theta4)*sin(theta5) - cos(theta5)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)) + cos(theta4)*sin(theta5)*(cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3))]
    % [ (sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1))*(sin(theta4)*sin(theta6) - cos(theta4)*cos(theta5)*cos(theta6)) + cos(theta1)*(cos(theta4)*sin(theta6) + cos(theta5)*cos(theta6)*sin(theta4)) - cos(theta6)*sin(theta5)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)), (sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1))*(cos(theta6)*sin(theta4) + cos(theta4)*cos(theta5)*sin(theta6)) + cos(theta1)*(cos(theta4)*cos(theta6) - cos(theta5)*sin(theta4)*sin(theta6)) + sin(theta5)*sin(theta6)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)), cos(theta4)*sin(theta5)*(sin(theta1)*sin(theta2)*sin(theta3) - cos(theta2)*cos(theta3)*sin(theta1)) - cos(theta1)*sin(theta4)*sin(theta5) - cos(theta5)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))]
    % [                                                                                                                               cos(theta6)*sin(theta5)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - (sin(theta4)*sin(theta6) - cos(theta4)*cos(theta5)*cos(theta6))*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)),                                                                                                                             - (cos(theta6)*sin(theta4) + cos(theta4)*cos(theta5)*sin(theta6))*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) - sin(theta5)*sin(theta6)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)),                                                                                       cos(theta5)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - cos(theta4)*sin(theta5)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2))]
    %
    % Initialize results:
    pos = zeros(1, 3);
    R = eye(3);
    frame_origins = zeros(5, 3);

    % Configuration params:
    a = 13.0;
    b =  2.5;
    c =  8.0;
    d =  2.5;
    e =  8.0;
    f =  2.5;

    % a. Frame 1:
    A_01 = compute_dh_matrix( 0, +pi/2, +a, theta1);
    T = A_01;
    frame_origins(2,:) = (T(1:3,4)/T(4,4))';
    % b. Frame 2:
    A_12 = compute_dh_matrix(+c,     0, -b, theta2);
    T = T * A_12;
    frame_origins(3,:) = (T(1:3,4)/T(4,4))';
    % c. Frame 3:
    A_23 = compute_dh_matrix( 0, -pi/2, -d, theta3);
    T = T * A_23;
    T(1:3, 4) = T(1:3, 4) + e*T(1:3,3);
    frame_origins(4,:) = (T(1:3,4)/T(4,4))';
    % d. Frame 4:
    A_34 = compute_dh_matrix( 0, +pi/2,  0, theta4);
    A_45 = compute_dh_matrix( 0, -pi/2,  0, theta5);
    A_56 = compute_dh_matrix( 0,     0,  f, theta6);
    T = T * A_34 * A_45 * A_56;
    frame_origins(5,:) = (T(1:3,4)/T(4,4))';

    pos = frame_origins(5,:);
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
