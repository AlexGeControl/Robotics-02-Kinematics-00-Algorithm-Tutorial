% In this function, you need to convert the rotation matrix R into axis-angle form

function [axang] = rotm2axang(R)
    % The trace of rotation matrix R equals 2*cos(theta) + 1:
    theta = acos((trace(R)-1)/2);

    % r(3,2) - r(2,3) = 2*sin(theta)*x;
    % r(1,3) - r(3,1) = 2*sin(theta)*y;
    % r(2,1) - r(1,2) = 2*sin(theta)*z;
    tolerance = 1e-6;

    % If theta is 0, R = I, set axis as zero vector belongs to R3
    if abs(theta) < tolerance
        axang = [NaN, NaN, NaN, theta];
    elseif abs(theta - pi) < tolerance
        if abs(R(1,1)+1) > tolerance
            x1 = -sqrt((R(1,1)+1)/2); x2 = sqrt((R(1,1)+1)/2);
            axang = [
                x1, R(2,1)/(2*x1), R(3,1)/(2*x1), theta;
                x2, R(2,1)/(2*x2), R(3,1)/(2*x2), theta;
            ];
        elseif abs(R(2,2)+1) > tolerance
            y1 = -sqrt((R(2,2)+1)/2); y2 = sqrt((R(2,2)+1)/2);
            axang = [
                R(1,2)/(2*y1), y1, R(3,2)/(2*y1), theta;
                R(1,2)/(2*y2), y2, R(3,2)/(2*y2), theta;
            ];
        else
            z1 = -sqrt((R(3,3)+1)/2); z2 = sqrt((R(3,3)+1)/2);
            axang = [
                R(1,3)/(2*z1), R(2,3)/(2*z1), z1, theta;
                R(1,3)/(2*z2), R(2,3)/(2*z2), z2, theta;
            ];
        end
    else
        axang = [1.0/(2*sin(theta))*[R(3,2) - R(2,3), R(1,3) - R(3,1), R(2,1) - R(1,2)], theta];
    end
end
