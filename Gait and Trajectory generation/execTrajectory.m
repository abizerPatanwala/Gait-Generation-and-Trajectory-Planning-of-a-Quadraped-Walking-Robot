function[jointAngles, jointAngVel, fxb, fzb, HipPos] = execTrajectory(InitHipPos, v_b, fxg, fzg, Vfxg, Vfzg,...
                                                      l1, l2, transferTime, timeStep, initTransfAngles)
times = 0:timeStep:transferTime;
jointAngles = zeros(2, length(times));
jointAngVel = zeros(2, length(times));
fxb = zeros(1, length(times));
fzb = zeros(1, length(times));
HipPos = zeros(2, length(times));
for i = 1:length(times)
    HipPos(1,i) = InitHipPos(1) + v_b*times(i);
    HipPos(2,i) = InitHipPos(2);
    fxb(i) = fxg(i) - HipPos(1,i);
    fzb(i) = fzg(i) - HipPos(2,i);
    if i > 1
        jointAngles(:,i) = IK(fxb(i), fzb(i), l1, l2, jointAngles(:,i-1));
    else
        jointAngles(:,i) = IK(fxb(i), fzb(i), l1, l2, initTransfAngles');
    end
    Vfxb = Vfxg(i) - v_b;
    Vfzb = Vfzg(i);
    jointAngVel(:,i) = calcAngVel(jointAngles(:,i), Vfxb, Vfzb, l1, l2);
end
end