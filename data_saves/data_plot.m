close all
clear all
clc

fileID = fopen('2018_02_02_zed_verification_1.txt','r');
data = textscan(fileID,'%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f');

t = cell2mat(data(1,1))'; % timestamp
x_z = cell2mat([data(1,2),data(1,3),data(1,4)])'; % zed position
q_z = cell2mat([data(1,5),data(1,6),data(1,7),data(1,8)])'; % zed orientation [x y z w]

x_v = cell2mat([data(1,9),data(1,10),data(1,11)])'; % vicon position
q_v = cell2mat([data(1,12),data(1,13),data(1,14),data(1,15)])'; % vicon orientation [x y z w]

R_z_to_v = [1 0 0; 0 0 -1; 0 1 0]; % rotation from zed to vicon

for i = 1:length(x_z)
    x_zv(:,i) = R_z_to_v*x_z(:,i); % zed position in vicon frame
    
    R_v(:,:,i) = quat_to_rot(q_v(:,i)); % vicon rotation 
    R_zv(:,:,i) = R_z_to_v*quat_to_rot(q_z(:,i)); % zed rotation
    
    w_v(:,i) = rot_to_eangles(R_v(:,:,i));
    w_zv(:,i) = rot_to_eangles(R_zv(:,:,i));
end


figure
subplot(3,1,1)
hold on
plot(x_v(1,:))
plot(x_zv(1,:))
title('Position')
legend('Vicon','Zed')
subplot(3,1,2)
hold on
plot(x_v(2,:))
plot(x_zv(2,:))
subplot(3,1,3)
hold on
plot(x_v(3,:))
plot(x_zv(3,:))

figure
subplot(3,1,1)
hold on
plot(w_v(1,:))
plot(w_zv(1,:))
title('Orientation')
legend('Vicon','Zed')
subplot(3,1,2)
hold on
plot(w_v(2,:))
plot(w_zv(2,:))
subplot(3,1,3)
hold on
plot(w_v(3,:))
plot(w_zv(3,:))