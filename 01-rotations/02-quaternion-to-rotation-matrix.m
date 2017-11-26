% In this script, you need to convert the quaternion to a rotation matrix
% Your final solution for the rotation matrix must be stored in R
% The quaternion for Qs + Qx i + Qy j + Qz k, is represented in matrix form as [Qs, Qx, Qy, Qz]

task2_solution = zeros(3*20, 3);

% Test code for 20 different quaternions
for i = 1:20

    quat = genQuaternion(); % Generates a random quaternion
    % quat is a 1x4 matrix of the form [Qs, Qx, Qy, Qz]

    R = (quat(1)^2-dot(quat(2:4), quat(2:4)))*eye(3)+2*quat(1)*[0, -quat(4), quat(3); quat(4), 0, -quat(2); -quat(3), quat(2), 0]+2*quat(2:4).'*quat(2:4);

    %% Storing answer for auto-grading
    if ~isequal(size(R), [3 3])
        error('R must be a 3x3 matrix')
    end

    task2_solution(((i-1)*3)+1:((i-1)*3)+3, :) = R;
end
