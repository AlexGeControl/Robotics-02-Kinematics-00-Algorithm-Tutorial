function [ ik_sol ] = puma_ik( x, y, z, R )
    % Compute inverse kinematic for PUMA
    %
    % The input to the function will be
    % 1. The position of the end effector (in inches) in the world frame
    % 2. The rotation matrix R_60 as described in the question.
    %
    % The output must be the joint angles of the robot
    % to achieve the desired end effector position and orientation.

    % Initialize inverse kinematics solution:
    ik_sol = ones(1, 6);

    % PUMA configuration params:
    a = 13.0;
    b =  2.5;
    c =  8.0;
    d =  2.5;
    e =  8.0;
    f =  2.5;

    % Step 1: Decouple wrist orientation with wrist center position:
    c_x = x - f*R(1,3);
    c_y = y - f*R(2,3);
    c_z = z - f*R(3,3);

    % Step 2: Coordinates of wrist center in frame 1:
    c_x_1 = sqrt(round(c_x^2+c_y^2-(b+d)^2,7));
    c_y_1 = b + d;

    % Step 3: Calculate theta3:
    theta3 = pi/2-acos((e^2+c^2-(c_z-a)^2-c_x^2-c_y^2+(b+d)^2) / (2*e*c));
    c3 = cos(theta3); s3 = sin(theta3);

    % Step 4: Calculate theta2:
    A2 = [
        c-e*s3,  -e*c3;
          e*c3, c-e*s3;
    ];
    b2 = [
        c_x_1;
        c_z - a;
    ];
    v2 = A2\b2;
    theta2 = atan2(v2(2,1), v2(1,1));
    c2 = cos(theta2); s2 = sin(theta2);

    % Step 5: Calculate theta1:
    A1 = [
        +c_x_1, -c_y_1;
        +c_y_1, +c_x_1;
    ];
    b1 = [
        c_x;
        c_y;
    ];
    v1 = A1\b1;
    theta1 = atan2(v1(2,1), v1(1,1));
    c1 = cos(theta1); s1 = sin(theta1);

    % Step 6: Calculate orientation matrix:
    c1 = cos(theta1); s1 = sin(theta1);
    R_30 = [
        [ c1*c2*c3 - c1*s2*s3, -s1, - c1*c2*s3 - c1*c3*s2];
        [ c2*c3*s1 - s1*s2*s3,  c1, - c2*s1*s3 - c3*s1*s2];
        [       c2*s3 + c3*s2,   0,         c2*c3 - s2*s3];
    ];
    R_63 = R_30'*R;

    % Step 7: Calculate ZYZ thetas:
    theta4 = atan2(-R_63(2,3),-R_63(1,3));
    theta5 = +acos(+R_63(3,3));
    theta6 = atan2(-R_63(3,2),+R_63(3,1));
    %theta4 = atan2(+R_63(2,3),+R_63(1,3));
    %theta5 = -acos(+R_63(3,3));
    %theta6 = atan2(+R_63(3,2),-R_63(3,1));

    ik_sol = [theta1, theta2, theta3, theta4, theta5, theta6];
end
