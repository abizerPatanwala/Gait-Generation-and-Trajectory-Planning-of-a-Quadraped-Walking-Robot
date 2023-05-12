function [jointAngles] = IK(fxb, fzb, l1, l2, prevAngle)
    jointAngles = zeros(2,1);
    jointAngles1 = zeros(2,1);
    jointAngles2 = zeros(2,1);
    l_dash = sqrt(fxb^2 + fzb^2);
    jointAngles1(2) = pi - acos((l1^2 + l2^2 - l_dash^2) / (2*l1*l2));
    jointAngles1(1) = atan(fxb/fzb) - acos((l_dash^2 + l1^2 -l2^2) / (2*l_dash*l1));
    jointAngles2(2) = acos((l1^2 + l2^2 - l_dash^2) / (2*l1*l2)) - pi;
    jointAngles2(1) = atan(fxb/fzb) + acos((l_dash^2 + l1^2 -l2^2) / (2*l_dash*l1));
    if norm(jointAngles1 - prevAngle) < norm(jointAngles2 - prevAngle)
        jointAngles = jointAngles1;
    else
        jointAngles = jointAngles2;
    end
end