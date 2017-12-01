function [ ik_sol ] = RPR_ik( x, y, z, R )
  % Compute inverse kinematics for RPR robot arm
  %
  % The input to the function will be:
  % 1. The position of the end effector (in inches) in the world frame
  % 2. The rotation matrix R_03
  %
  % The output must be the joint angles and extensions of the robot
  % to achieve the end effector position and orientation.

  % Initialize inverse kinematics:
  ik_sol = ones(1, 3);

  % RPR params:
  a = 10;
  b = 5;

  % a. theta3:
  theta3 = atan2(R(3,3), -R(3,2));
  % b. d2:
  d2 = sqrt(2) * (a + b*sin(theta3) - z);
  % c. theta1:
  factor = (d2*sin(pi/4) + b*cos(theta3))
  theta1 = atan2(y/factor,x/factor) -1/2*pi;

  % Assign inverse kinematics solution:
  ik_sol(1,1) = theta1;
  ik_sol(1,2) = d2;
  ik_sol(1,3) = theta3;

end
