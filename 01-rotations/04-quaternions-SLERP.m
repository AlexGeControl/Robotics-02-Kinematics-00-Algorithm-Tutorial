function [ q_int ] = quat_slerp( q0, q1, steps )
    % QUAT_SLERP Perform SLERP between two quaternions and return the intermediate quaternions
    %   Usage: [ q_int ] = quat_slerp( q0, q1, steps )
    %   Inputs:
    %       q0 is the quaternion representing the starting orientation, 1x4 matrix
    %       q1 is the quaternion representing the final orientation, 1x4 matrix
    %       steps is the number of intermediate quaternions required to be returned, integer value
    %       The first step is q0, and the last step is q1
    %   Output:
    %       q_int contains q0, steps-2 intermediate quaternions, q1
    %       q_int is a (steps x 4) matrix

    % Calculate the angle between the two quaternions:
    product = dot(q0, q1); factor = sign(product);

    omega = factor * acos(factor * product);

    % Generate interpolation points:
    t = linspace(0, 1, steps)';

    q_int = 1.0/sin(omega)*(sin((1.0 - t)*omega)*q0 + sin(t*omega) * factor * q1);
end
