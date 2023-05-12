clear; close; clc;

m = matfile('JointAnglesTotal.mat');
k = m.JointanglesTotal;

limb_angles1 = k(1,:);
knee_angles1 = k(2,:);


limb_angles2 = k(3,:);
knee_angles2 = k(4,:);


limb_angles3 = k(5,:);
knee_angles3 = k(6,:);


limb_angles4 = k(7,:);
knee_angles4 = k(8,:);
NoC = 10;

% ROS Setup
rosinit;


limb1_angle = rospublisher('/dreamwalker/limb_joint1_position_controller/command');
limb2_angle = rospublisher('/dreamwalker/limb_joint2_position_controller/command');
limb3_angle = rospublisher('/dreamwalker/limb_joint3_position_controller/command');
limb4_angle = rospublisher('/dreamwalker/limb_joint4_position_controller/command');


knee1_angle = rospublisher('/dreamwalker/knee_joint1_position_controller/command');
knee2_angle = rospublisher('/dreamwalker/knee_joint2_position_controller/command');
knee3_angle = rospublisher('/dreamwalker/knee_joint3_position_controller/command');
knee4_angle = rospublisher('/dreamwalker/knee_joint4_position_controller/command');

shoulder1_angle = rospublisher('/dreamwalker/shoulder_joint1_position_controller/command');
shoulder2_angle = rospublisher('/dreamwalker/shoulder_joint2_position_controller/command');
shoulder3_angle = rospublisher('/dreamwalker/shoulder_joint3_position_controller/command');
shoulder4_angle = rospublisher('/dreamwalker/shoulder_joint4_position_controller/command');

limb1 = rosmessage(limb1_angle);
limb2 = rosmessage(limb2_angle);
limb3 = rosmessage(limb3_angle);
limb4 = rosmessage(limb4_angle);



knee1 = rosmessage(knee1_angle);
knee2 = rosmessage(knee2_angle);
knee3 = rosmessage(knee3_angle);
knee4 = rosmessage(knee4_angle);


shoulder1 = rosmessage(shoulder1_angle);
shoulder2 = rosmessage(shoulder2_angle);
shoulder3 = rosmessage(shoulder3_angle);
shoulder4 = rosmessage(shoulder4_angle);



client = rossvcclient('/gazebo/set_model_configuration');
req = rosmessage(client);
req.ModelName = 'dreamwalker_dummy';
req.UrdfParamName = 'robot_description';
resp = call(client,req,'Timeout',3);

shoudler1.Data = 0;

shoudler2.Data = 0;


shoudler3.Data = 0;


shoudler4.Data = 0;

send(shoulder1_angle, shoulder1);
pause(0.1);

send(shoulder2_angle, shoulder2);
pause(0.1);

send(shoulder3_angle, shoulder3);
pause(0.1);

send(shoulder4_angle, shoulder4);
pause(0.1);



knee1.Data = -knee_angles1(1);
limb1.Data = -limb_angles1(1);

knee2.Data = -knee_angles2(1);
limb2.Data = -limb_angles2(1);

knee3.Data = -knee_angles3(1);
limb3.Data = -limb_angles3(1);


knee4.Data = -knee_angles4(1);
limb4.Data = -limb_angles4(1);

send(limb1_angle, limb1);
pause(0.01)
send(knee1_angle, knee1);

pause(0.01);

send(limb2_angle, limb2);
pause(0.01)
send(knee2_angle, knee2);

pause(0.01);

send(limb3_angle, limb3);
pause(0.01)
send(knee3_angle, knee3);

pause(0.01);

send(limb4_angle, limb4);
pause(0.01)
send(knee4_angle, knee4);

pause(0.01);
tic;
t = 0;
for j = 1:NoC
    disp('here')
    for i = 1:size(limb_angles1')
    limb1.Data = -limb_angles1(i);
    knee1.Data = -knee_angles1(i);

    limb2.Data = -limb_angles2(i);
    knee2.Data = -knee_angles2(i);

    limb3.Data = -limb_angles3(i);
    knee3.Data = -knee_angles3(i);

        
    limb4.Data = -limb_angles4(i);
    knee4.Data = -knee_angles4(i);

    
    send(limb1_angle, limb1);
    pause(0.01)
    send(knee1_angle, knee1);
    pause(0.01)

    
    send(limb2_angle, limb2);
    pause(0.01)
    send(knee2_angle, knee2);
    pause(0.01)


    send(limb3_angle, limb3);
    pause(0.01)
    send(knee3_angle, knee3);
    pause(0.01)

    send(limb4_angle, limb4);
    pause(0.01)
    send(knee4_angle, knee4);
    pause(0.01)
    end
end
% 
% % knee4.Data = -knee_angles(1);
% % limb4.Data = -limb_angles(1);
% % 
% % send(limb4_angle, limb4);
% % send(knee4_angle, knee4);
% % 
% % pause(3);
% 
% 
% % tau1.Data = 0;
% % tau2.Data = 0;
% % send(j1_effort,tau1);
% % send(j2_effort,tau2);
% % subplot(3,2,1);
% % 
% % plot(T,theta1);
% % title('theta1 vs time');
% % 
% % subplot(3,2,2);
% % plot(T,theta2);
% % title('theta2 vs time');
% % 
% % subplot(3,2,3);
% % 
% % plot(T,Tau1);
% % title('torque1 vs time');
% % 
% % subplot(3,2,4);
% % plot(T,Tau2);
% % title('torque2 vs time');
% % 
% % subplot(3,2,5);
% % 
% % plot(T,d1);
% % title('dtheta1 vs time');
% % 
% % subplot(3,2,6);
% % plot(T,d2);
% % title('dtheta2 vs time');

rosshutdown;

