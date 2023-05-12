function jointAngleVel = calcAngVel(jointAngles, Vfxb, Vfzb, l1, l2)
    theta1 = jointAngles(1);
    theta2 = jointAngles(2);
    Jacob = [-l1*cos(theta1) - l2*cos(theta1 + theta2), -l2*cos(theta1 + theta2);
             l1*sin(theta1) + l2*sin(theta1 + theta2), l2*sin(theta1 + theta2)];
    %disp(Jacob)
    vel = [Vfxb; Vfzb];
    jointAngleVel = pinv(Jacob)*vel;
end