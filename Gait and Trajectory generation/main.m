%% SECTION 1: Robot Parameters
% Below 6 parameters are for dreamwalker. Uncomment this to get the trajectories for dreamwalker.
%Comment the parameters of petoi dog.
% l1 = 5.5; %cm
% l2 = 6.5; %cm
% l = 18.9; %cm
% w = 16.2; %cm
%initJointTransfer = [0, 0.5235];
%initJointSupport =  [-0.4952, 0.2411];

%Below 6 parameters are for petoi dog. Comment this set of parameters when getting trajectories 
%for dreamwalker
l1 = 4.6; %cm
l2 = 4.8; %cm
l = 10.5; %cm
w = 9.7; %cm
initJointTransfer = [(0/180)*pi, (20/180)*pi];
initJointSupport = [-0.3315, 0.2037];

InitFootToHipTransfer = [l2*sin(initJointTransfer(1) + initJointTransfer(2)) + l1*sin(initJointTransfer(1)),...
                l2*cos(initJointTransfer(1) + initJointTransfer(2)) + l1*cos(initJointTransfer(1))];
h = InitFootToHipTransfer(2);


InitHipToFootSupport = [-l2*sin(initJointSupport(1) + initJointSupport(2)) - l1*sin(initJointSupport(1)),...
                0, -l2*cos(initJointSupport(1) + initJointSupport(2)) - l1*cos(initJointSupport(1))];

%% SECTION 2: Gait and Trajectory Planning Parameters
b = 0.75; 
v_b = 5; %cm
stride = 5; %cm
cycleTime = stride / v_b;
transferTime = (1-b)*cycleTime;
supportTime = b*cycleTime;


%% SECTION3: Gait Planning and Trajectory 
timeStep = 0.01;
legPhases = gaitGeneration(b);

zMax = InitFootToHipTransfer(2) / 10.0;
[fxgTrans, fzgTrans, VfxgTrans, VfzgTrans, ~] = generateTrajectory(stride, zMax, timeStep, transferTime);

[jointAnglesTrans, jointAngVelTrans, fxbTrans, fzbTrans, hipPosTrans] = execTrajectory(InitFootToHipTransfer, v_b, fxgTrans, fzgTrans, VfxgTrans, VfzgTrans,...
                                                      l1, l2, transferTime, timeStep, initJointTransfer);

bodyFrame = [0,0,0];
hip1B = [l/2,  w/2, h];
hip2B = [l/2, -w/2, h];
hip3B = [-l/2, w/2, h];
hip4B = [-l/2, -w/2, h];
hip1W = hip1B + bodyFrame;
hip2W = hip2B + bodyFrame;
hip3W = hip3B + bodyFrame;
hip4W = hip4B + bodyFrame;

foot1B = hip1B + InitHipToFootSupport;
foot2B = hip2B + InitHipToFootSupport;
foot3B = hip3B + InitHipToFootSupport;
foot4B = hip4B + InitHipToFootSupport;
foot1W = foot1B + bodyFrame;
foot2W = foot2B + bodyFrame;
foot3W = foot3B + bodyFrame;
foot4W = foot4B + bodyFrame;

times = 0:timeStep:supportTime;

% jointAnglesSupport = zeros(2, length(times));
% hipPosSupport(1,:) = linspace(-InitHipToFootSupport(1), InitFootToHipTransfer(1), length(times));
% hipPosSupport(2,:) = linspace(-InitHipToFootSupport(3), InitFootToHipTransfer(2), length(times));
% jointAnglesSupport(1,:) = linspace(initJointSupport(1), initJointTransfer(1), length(times)); 
% for i = 1 : length(times)
%     if i ~= 1
%         bodyFrame = bodyFrame + v_b*(times(i) - times(i-1))*[1,0,0];
%     else
%         bodyFrame = bodyFrame + v_b*times(i)*[1,0,0];
%     end
%     hip1W = hip1B + bodyFrame;
%     l_dash = sqrt((foot1W(1) - hip1W(1))^2 + (foot1W(3) - hip1W(3))^2);
%     jointAnglesSupport(2,i)  = acos((l_dash^2 - l1^2 - l2^2) / (2*l1*l2));
% end
jointAnglesSupport = zeros(2, length(times));
footx = InitHipToFootSupport(1);
footz = InitHipToFootSupport(3);
for i = 1 : length(times)
    if i ~= 1
        footx = -v_b*(times(i) - times(i-1)) + footx;
    else
        footx = -v_b*times(i) + footx;
    end 
    if i > 1
        jointAnglesSupport(:,i) = IK(footx, footz, l1, l2, jointAnglesSupport(:,i-1));
    else
        jointAnglesSupport(:,i) = IK(footx, footz, l1, l2, initJointSupport');
    end
end

%% SECTION 4: Position and velocity plots for each leg during one cycle time
times = 0:timeStep:supportTime;
times1 = 0:timeStep:transferTime;
times3 = 0:timeStep:cycleTime;
offset1s = 1;
offset2s = round((7/9) * length(times));
offset3s = round((4/9) * length(times));
offset = [offset1s, offset2s, offset3s];
fx = [];
fz = [];
fxv = [];
fzv = [];
JA = [];
JA2 = [];
JAV = [];
JAV2 = [];
temp = zeros(8, (cycleTime/timeStep) + 1);
legCount = 0;
for l = 1:3
    legCount = legCount + 1;
    fx = [];
    fz = [];
    fxv = [];
    fzv = [];
    JA = [];
    JA2 = [];
    JAV = [];
    JAV2 = [];
    if l == 1
        footlW = foot1W;
    end
    if l == 2
        footlW = foot2W;
    end
    if l == 3
        footlW = foot3W;
    end
    for i = offset(l):length(times)
        fx = [fx footlW(1)];
        fz = [fz footlW(3)];
        fxv = [fxv 0];
        fzv = [fzv 0];
        JA = [JA jointAnglesSupport(1,i)];
        JA2 = [JA2 jointAnglesSupport(2,i)];
    end
    JAV = [JAV jointAngVelTrans(1,1)];
    JAV2 = [JAV2 jointAngVelTrans(2,1)];
    for i = 2:length(times1)
        posx = fxgTrans(i) + footlW(1);
        posz = fzgTrans(i) + footlW(3);
        fx = [fx posx];
        fz = [fz posz];
        fxv = [fxv VfxgTrans(i)];
        fzv = [fzv VfzgTrans(i)];
        JA = [JA jointAnglesTrans(1,i)];
        JA2 = [JA2 jointAnglesTrans(2,i)];
        JAV = [JAV jointAngVelTrans(1,i)];
        JAV2 = [JAV2 jointAngVelTrans(2,i)];
    end
    footx = fx(end);
    footz = fz(end);
    for i = 1: offset(l) - 1
        fx = [fx footx];
        fz = [fz footz];
        fxv = [fxv 0];
        fzv = [fzv 0];
        JA = [JA jointAnglesSupport(1,i)];
        JA2 = [JA2 jointAnglesSupport(2,i)];
    end
    temp(legCount, :) = JA;
    legCount = legCount + 1;
    temp(legCount, :) = JA2;
    figure(l)
    subplot(2,2,1)
    plot(times3,fx)
    title("x foot pos for 1 cycle time(leg "+ string(l) + ")")
    subplot(2,2,2)
    plot(times3,fz)
    title("z foot pos for 1 cycle time (leg "+ string(l) + ")")
    subplot(2,2,3)
    plot(times3,fxv)
    title("x foot val for 1 cycle time (leg "+ string(l) + ")")
    subplot(2,2,4)
    plot(times3,fzv)
    title("z foot vel for 1 cycle time (leg "+ string(l) + ")")
    figure(l+4)
    subplot(2,2,1)
    plot(times3, JA)
    title("Hip joint Angle for 1 cycle time(leg "+ string(l) + ")")
    subplot(2,2,2)
    plot(times3, JA2)
    title("Knee joint Angle for 1 cycle time(leg "+ string(l) + ")")
    subplot(2,2,3)
    t = times1 + legPhases(l)*cycleTime;
    plot( t, JAV)
    title("Hip joint vel for transfer phase(leg "+ string(l) + ")")
    subplot(2,2,4)
    t = times1 + legPhases(l)*cycleTime;
    plot( t, JAV2)
    title("Knee joint vel for transfer phase (leg "+ string(l) + ")")
   
end

fx = [];
fz = [];
fxv = [];
fzv = [];
JA = [];
JA2 = [];
JAV = [];
JAV2 = [];
for i = 1:length(times1)
    posx = fxgTrans(i) + foot4W(1);
    posz = fzgTrans(i) + foot4W(3);
    fx = [fx posx];
    fz = [fz posz];
    fxv = [fxv VfxgTrans(i)];
    fzv = [fzv VfzgTrans(i)];
    JA = [JA jointAnglesTrans(1,i)];
    JA2 = [JA2 jointAnglesTrans(2,i)];
    JAV = [JAV jointAngVelTrans(1,i)];
    JAV2 = [JAV2 jointAngVelTrans(2,i)];
end
footx = fx(end);
footz = fz(end);
for i = 2: length(times)
    fx = [fx footx];
    fz = [fz footz];
    fxv = [fxv 0];
    fzv = [fzv 0];
    JA = [JA jointAnglesSupport(1,i)];
    JA2 = [JA2 jointAnglesSupport(2,i)];
end
legCount = legCount + 1;
temp(legCount, :) = JA;
legCount = legCount + 1;
temp(legCount, :) = JA2;
l = 4;
hold on
figure(l)
subplot(2,2,1)
plot(times3,fx)
title("x foot pos for 1 cycle time(leg "+ string(l) + ")")
subplot(2,2,2)
plot(times3,fz)
title("z foot pos for 1 cycle time (leg "+ string(l) + ")")
subplot(2,2,3)
plot(times3,fxv)
title("x foot vel for 1 cycle time (leg "+ string(l) + ")")
subplot(2,2,4)
plot(times3,fzv)
title("z foot val for 1 cycle time (leg "+ string(l) + ")")

figure(l+4)
subplot(2,2,1)
plot(times3, JA)
title("Hip joint Angle for 1 cycle time(leg "+ string(l) + ")")
subplot(2,2,2)
plot(times3, JA2)
title("Knee joint Angle for 1 cycle time(leg "+ string(l) + ")")
subplot(2,2,3)
t = times1 + legPhases(l)*cycleTime;
plot( t, JAV)
title("Hip joint vel for transfer phase(leg "+ string(l) + ")")
subplot(2,2,4)
t = times1 + legPhases(l)*cycleTime;
plot( t, JAV2)
title("Knee joint vel for transfer phase (leg "+ string(l) + ")")
JointAnglesTotal = zeros(8, (cycleTime/timeStep) + 1);
JointAnglesTotal(1,:) = temp(1,:);
JointAnglesTotal(2,:) = temp(3,:);
JointAnglesTotal(3,:) = temp(7,:);
JointAnglesTotal(4,:) = temp(5,:);
JointAnglesTotal(5,:) = temp(2,:);
JointAnglesTotal(6,:) = temp(4,:);
JointAnglesTotal(7,:) = temp(8,:);
JointAnglesTotal(8,:) = temp(6,:);

writematrix(round(rad2deg(JointAnglesTotal')), 'jointAnglesTotal.csv')

%% SECTION 5 : Animation of the leg in swing phase

close all;
figure(10), hold on
link2 = plot([0,0],[0, 0], 'r-', 'LineWidth', 4 );
link1 = plot([0,0],[0, 0 ], 'b-', 'LineWidth',4 );
drawnow
times1 = 0:timeStep:transferTime;
for i = 1:length(times1)
    hipx = InitFootToHipTransfer(1) + v_b*times1(i);
    hipz = InitFootToHipTransfer(2);
    set(link1,'xdata',[hipx, hipx - l1*sin(jointAnglesTrans(1,i))],'ydata',[hipz, hipz - l1*cos(jointAnglesTrans(1,i))]); 
    set(link2,'xdata',[fxgTrans(i), fxgTrans(i) + l2*sin(jointAnglesTrans(1,i) + jointAnglesTrans(2,i)) ],'ydata',[fzgTrans(i), fzgTrans(i) + l2*cos(jointAnglesTrans(1,i) + jointAnglesTrans(2,i))]);
    title('Leg in swing phase');
    xlim([-12,12]), ylim([-12,12])
   
    xlabel("X")
    ylabel("Z")
    drawnow
    pause(1)
end

%% SECTION 6: Animation of the robot

close all
animationTime = 10;
simTimes = 0: timeStep:animationTime;
times = 0:timeStep:transferTime;

figure(11), hold on
bodyFrame = [0,0,0];
hip1B = [l/2,  w/2, h];
hip2B = [l/2, -w/2, h];
hip3B = [-l/2, w/2, h];
hip4B = [-l/2, -w/2, h];
hip1W = hip1B + bodyFrame;
hip2W = hip2B + bodyFrame;
hip3W = hip3B + bodyFrame;
hip4W = hip4B + bodyFrame;

foot1B = hip1B + InitHipToFootSupport;
foot2B = hip2B + InitHipToFootSupport;
foot3B = hip3B + InitHipToFootSupport;
foot4B = hip4B + InitHipToFootSupport;
foot1W = foot1B + bodyFrame;
foot2W = foot2B + bodyFrame;
foot3W = foot3B + bodyFrame;
foot4W = foot4B + bodyFrame;




link24 = plot3([hip4W(1), hip2W(1)],[hip4W(2), hip2W(2)],[h,h],  'g-', 'LineWidth', 4);
link12 = plot3([hip1W(1), hip2W(1)],[hip1W(2), hip2W(2)],[h,h],  'g-', 'LineWidth', 4);
link13 = plot3([hip1W(1), hip3W(1)],[hip1W(2), hip3W(2)],[h,h],  'g-', 'LineWidth', 4);
link34 = plot3([hip3W(1), hip4W(1)],[hip3W(2), hip4W(2)],[h,h],  'g-', 'LineWidth', 4);



leg1link2 = plot3([foot1W(1), foot1W(1) + l2*sin(initJointSupport(1) + initJointSupport(2))], [w/2, w/2], [foot1W(3), foot1W(3) + l2*cos(initJointSupport(1) + initJointSupport(2))], 'r-', 'LineWidth', 4 );
leg1link1 = plot3([hip1W(1), hip1W(1) - l1*sin(initJointSupport(1))],[w/2, w/2], [hip1W(3), hip1W(3) - l1*cos(initJointSupport(1))], 'b-', 'LineWidth',4 );
leg2link2 = plot3([foot2W(1), foot2W(1) + l2*sin(initJointSupport(1) + initJointSupport(2))], [-w/2, -w/2], [foot2W(3), foot2W(3) + l2*cos(initJointSupport(1) + initJointSupport(2))], 'r-', 'LineWidth', 4 );
leg2link1 = plot3([hip2W(1), hip2W(1) - l1*sin(initJointSupport(1))],[-w/2, -w/2], [hip2W(3), hip2W(3) - l1*cos(initJointSupport(1))], 'b-', 'LineWidth',4 );
leg3link2 = plot3([foot3W(1), foot3W(1) + l2*sin(initJointSupport(1) + initJointSupport(2))], [w/2, w/2], [foot3W(3), foot3W(3) + l2*cos(initJointSupport(1) + initJointSupport(2))], 'r-', 'LineWidth', 4 );
leg3link1 = plot3([hip3W(1), hip3W(1) - l1*sin(initJointSupport(1))],[w/2, w/2], [hip3W(3), hip3W(3) - l1*cos(initJointSupport(1))], 'b-', 'LineWidth',4 );
leg4link2 = plot3([foot4W(1), foot4W(1) + l2*sin(initJointSupport(1) + initJointSupport(2))], [-w/2, -w/2], [foot4W(3), foot4W(3) + l2*cos(initJointSupport(1) + initJointSupport(2))], 'r-', 'LineWidth', 4 );
leg4link1 = plot3([hip4W(1), hip4W(1) - l1*sin(initJointSupport(1))],[-w/2, -w/2], [hip4W(3), hip4W(3) - l1*cos(initJointSupport(1))], 'b-', 'LineWidth',4 );
drawnow
countTrans = [0, 0, 0, 0];
countSupport = [0, 0, 0, 0];
currAngle = zeros(2, 4);
for i = 1:length(simTimes)
    inCycleTime = rem(simTimes(i), cycleTime);
    normalizedTime = inCycleTime / cycleTime;
    
    for j = 1:4
        if legPhases(j) == normalizedTime || countTrans(j) > 0
            countTrans(j) = countTrans(j) + 1;
            countSupport(j) = 0;
        end
        if countTrans(j) == 0
            countSupport(j) = countSupport(j) + 1;
            countTrans(j) = 0;
        end
    end

    if i ~= 1
        bodyFrame = bodyFrame + v_b*(simTimes(i) - simTimes(i-1))*[1,0,0];
    else
        bodyFrame = bodyFrame + v_b*simTimes(i)*[1,0,0];
    end
    %disp(simTimes(i))
    hip1W = hip1B + bodyFrame;
    hip2W = hip2B + bodyFrame;
    hip3W = hip3B + bodyFrame;
    hip4W = hip4B + bodyFrame;  
    %foot1W = foot1B + bodyFrame;
    %foot2W = foot2B + bodyFrame;
    %foot3W = foot3B + bodyFrame;
    %foot4W = foot4B + bodyFrame;
    


    set(link24,'xdata',[hip4W(1), hip2W(1)],'ydata',[hip4W(2), hip2W(2)], 'zdata', [h, h]); 
    set(link12,'xdata',[hip1W(1), hip2W(1)],'ydata',[hip1W(2), hip2W(2)], 'zdata', [h, h]); 
    set(link13,'xdata',[hip1W(1), hip3W(1)],'ydata',[hip1W(2), hip3W(2)], 'zdata', [h, h]); 
    set(link34,'xdata',[hip3W(1), hip4W(1)],'ydata',[hip3W(2), hip4W(2)], 'zdata', [h, h]); 
    if countSupport(1) > 0
        currAngle(:,1) = [jointAnglesSupport(1, countSupport(1)); jointAnglesSupport(2,countSupport(1))];
        set(leg1link1,'xdata',[hip1W(1), hip1W(1) - l1*sin(currAngle(1,1))],'zdata',[hip1W(3), hip1W(3) - l1*cos(currAngle(1,1))], 'ydata', [w/2, w/2]); 
        set(leg1link2,'xdata',[foot1W(1), hip1W(1) - l1*sin(currAngle(1,1)) ],'zdata',[foot1W(3), hip1W(3) - l1*cos(currAngle(1,1))], 'ydata', [w/2, w/2]);
        if countSupport(1) == length(jointAnglesSupport)
                countSupport(1) = 0;
        end
    else 
        foot1W = hip1W + [fxbTrans(countTrans(1)), w/2, fzbTrans(countTrans(1))];
        currAngle(:,1) = [jointAnglesTrans(1, countTrans(1)); jointAnglesTrans(2,countTrans(1))];
        set(leg1link1,'xdata',[hip1W(1), hip1W(1) - l1*sin(currAngle(1,1))],'zdata',[hip1W(3), hip1W(3) - l1*cos(currAngle(1,1))], 'ydata', [w/2, w/2]); 
        set(leg1link2,'xdata',[foot1W(1), foot1W(1) + l2*sin(currAngle(1,1) + currAngle(2,1)) ],'zdata',[foot1W(3), foot1W(3) + l2*cos(currAngle(1,1) + currAngle(2,1))], 'ydata', [w/2, w/2]);
        if countTrans(1) == length(jointAnglesTrans)
                countTrans(1) = 0;
        end
    end 
   if countSupport(2) > 0
        currAngle(:,2) = [jointAnglesSupport(1, countSupport(2)); jointAnglesSupport(2,countSupport(2))];
        set(leg2link1,'xdata',[hip2W(1), hip2W(1) - l1*sin(currAngle(1,2))],'zdata',[hip2W(3), hip2W(3) - l1*cos(currAngle(1,2))], 'ydata', [-w/2, -w/2]); 
        set(leg2link2,'xdata',[foot2W(1), hip2W(1) - l1*sin(currAngle(1,2)) ],'zdata',[foot2W(3), hip2W(3) - l1*cos(currAngle(1,2))], 'ydata', [-w/2, -w/2]);
        if countSupport(2) == length(jointAnglesSupport)
                countSupport(2) = 0;
        end
    else 
        foot2W = hip2W + [fxbTrans(countTrans(2)), -w/2, fzbTrans(countTrans(2))];
        currAngle(:,2) = [jointAnglesTrans(1, countTrans(2)); jointAnglesTrans(2,countTrans(2))];
        set(leg2link1,'xdata',[hip2W(1), hip2W(1) - l1*sin(currAngle(1,2))],'zdata',[hip2W(3), hip2W(3) - l1*cos(currAngle(1,2))], 'ydata', [-w/2, -w/2]); 
        set(leg2link2,'xdata',[foot2W(1), foot2W(1) + l2*sin(currAngle(1,2) + currAngle(2,2)) ],'zdata',[foot2W(3), foot2W(3) + l2*cos(currAngle(1,2) + currAngle(2,2))], 'ydata', [-w/2, -w/2]);
        if countTrans(2) == length(jointAnglesTrans)
                countTrans(2) = 0;
        end
   end
   if countSupport(3) > 0
        currAngle(:,3) = [jointAnglesSupport(1, countSupport(3)); jointAnglesSupport(2,countSupport(3))];
        set(leg3link1,'xdata',[hip3W(1), hip3W(1) - l1*sin(currAngle(1,3))],'zdata',[hip3W(3), hip3W(3) - l1*cos(currAngle(1,3))], 'ydata', [w/2, w/2]); 
        set(leg3link2,'xdata',[foot3W(1), hip3W(1) - l1*sin(currAngle(1,3)) ],'zdata',[foot3W(3), hip3W(3) - l1*cos(currAngle(1,3))], 'ydata', [w/2, w/2]);
        if countSupport(3) == length(jointAnglesSupport)
                countSupport(3) = 0;
        end
    else 
        foot3W = hip3W + [fxbTrans(countTrans(3)), -w/2, fzbTrans(countTrans(3))];
        currAngle(:,3) = [jointAnglesTrans(1, countTrans(3)); jointAnglesTrans(2,countTrans(3))];
        set(leg3link1,'xdata',[hip3W(1), hip3W(1) - l1*sin(currAngle(1,3))],'zdata',[hip3W(3), hip3W(3) - l1*cos(currAngle(1,3))], 'ydata', [w/2, w/2]); 
        set(leg3link2,'xdata',[foot3W(1), foot3W(1) + l2*sin(currAngle(1,3) + currAngle(2,3)) ],'zdata',[foot3W(3), foot3W(3) + l2*cos(currAngle(1,3) + currAngle(2,3))], 'ydata', [w/2, w/2]);
        if countTrans(3) == length(jointAnglesTrans)
                countTrans(3) = 0;
        end
   end
   if countSupport(4) > 0
        currAngle(:,4) = [jointAnglesSupport(1, countSupport(4)); jointAnglesSupport(2,countSupport(4))];
        set(leg4link1,'xdata',[hip4W(1), hip4W(1) - l1*sin(currAngle(1,4))],'zdata',[hip4W(3), hip4W(3) - l1*cos(currAngle(1,4))], 'ydata', [-w/2, -w/2]); 
        set(leg4link2,'xdata',[foot4W(1), hip4W(1) - l1*sin(currAngle(1, 4)) ],'zdata',[foot4W(3), hip4W(3) - l1*cos(currAngle(1,4))], 'ydata', [-w/2, -w/2]);
        if countSupport(4) == length(jointAnglesSupport)
                countSupport(4) = 0;
        end
    else 
        foot4W = hip4W + [fxbTrans(countTrans(4)), -w/2, fzbTrans(countTrans(4))];
        currAngle(:,4) = [jointAnglesTrans(1, countTrans(4)); jointAnglesTrans(2,countTrans(4))];
        set(leg4link1,'xdata',[hip4W(1), hip4W(1) - l1*sin(currAngle(1,4))],'zdata',[hip4W(3), hip4W(3) - l1*cos(currAngle(1,4))], 'ydata', [-w/2, -w/2]); 
        set(leg4link2,'xdata',[foot4W(1), foot4W(1) + l2*sin(currAngle(1,4) + currAngle(2,4)) ],'zdata',[foot4W(3), foot4W(3) + l2*cos(currAngle(1,4) + currAngle(2,4))], 'ydata', [-w/2, -w/2]);
        if countTrans(4) == length(jointAnglesTrans)
                countTrans(4) = 0;
        end
    end
    xlim([-50 50]), ylim([-50 50]), zlim([-50 50]);
    title('Walking Gait Animation');
    xlabel('X') 
    ylabel('Y') 
    zlabel('Z')
    drawnow
    pause(0.3)
end




    



