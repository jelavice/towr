clc;

addpath('matlab_rosbag-0.4-linux64');
clear rosbag_wrapper;
clear ros.Bag;
clear all
close all


% Using load() lets you auto-complete filepaths.
prefix = '/home/jelavice/Documents/catkin_workspaces/towr_ws/src/xpp/xpp_examples/bags';
bagName = '/m545.bag';

filename = [prefix bagName];

bag = ros.Bag.load(filename);
bag.info()

topic1 = '/xpp/joint_des';
msgs = bag.readAll({topic1});

fprintf('Read %i messages\n', length(msgs));

[msgs, meta] = bag.readAll(topic1);
fprintf('Got %i messages, first one at time %f\n', ...
    length(msgs), meta{1}.time.time);

bag.resetView(topic1);
count = 0;
while bag.hasNext()
    [msg, meta] = bag.read();
    count = count + 1;
end

%exctract the motion of the base
[msgs, meta] = bag.readAll(topic1); % Messages are structs
accessor = @(pose) pose.base.pose.position;
[xyzBase] = ros.msgs2mat(msgs, accessor); % Convert struct to 3-by-N matrix of linear position
xyzBase = xyzBase';
time = cellfun(@(x) x.time.time, meta); % Get timestamps

%extract positions for all four end-effectors
accessor = @(feet) (feet.ee_motion(1).pos);
[xyzLF] = (ros.msgs2mat(msgs, accessor))';
accessor = @(feet) (feet.ee_motion(2).pos);
[xyzRF] = (ros.msgs2mat(msgs, accessor))';
accessor = @(feet) (feet.ee_motion(3).pos);
[xyzLH] = (ros.msgs2mat(msgs, accessor))';
accessor = @(feet) (feet.ee_motion(4).pos);
[xyzRH] = (ros.msgs2mat(msgs, accessor))';

%extract the quaternion
accessor = @(pose) pose.base.pose.orientation;
[q] = (ros.msgs2mat(msgs, accessor))';
%swap the columns because matlab has a [w x y z] quaternion representation
temp = q(:,1);
q(:,1) = q(:,4);
q(:,4) = temp;
euler = rad2deg(quat2eul(q)); 


%exctract the velocities for the feet
accessor = @(feet) (feet.ee_motion(1).vel);
[velLF] = (ros.msgs2mat(msgs, accessor))';
accessor = @(feet) (feet.ee_motion(2).vel);
[velRF] = (ros.msgs2mat(msgs, accessor))';
accessor = @(feet) (feet.ee_motion(3).vel);
[velLH] = (ros.msgs2mat(msgs, accessor))';
accessor = @(feet) (feet.ee_motion(4).vel);
[velRH] = (ros.msgs2mat(msgs, accessor))';

%exctract the joint positions in radinas
accessor = @(feet) (feet.limb_joints(1).position);
[jointsLF] = (ros.msgs2mat(msgs, accessor))';
accessor = @(feet) (feet.limb_joints(2).position);
[jointsRF] = (ros.msgs2mat(msgs, accessor))';
accessor = @(feet) (feet.limb_joints(3).position);
[jointsLH] = (ros.msgs2mat(msgs, accessor))';
accessor = @(feet) (feet.limb_joints(4).position);
[jointsRH] = (ros.msgs2mat(msgs, accessor))';
accessor = @(feet) (feet.limb_joints(5).position);
[jointsBoom] = (ros.msgs2mat(msgs, accessor))';

%% make base and feet xy plots


%now plot them in the x-y plane
figure
plot(xyzLF(:,1), xyzLF(:,2), 'Linewidth', 2)
hold on
plot(xyzRF(:,1), xyzRF(:,2), 'Linewidth', 2)
plot(xyzLH(:,1), xyzLH(:,2), 'Linewidth', 2)
plot(xyzRH(:,1), xyzRH(:,2), 'Linewidth', 2)
plot(xyzBase(:,1), xyzBase(:,2), 'LineWidth', 2);
legend('LF', 'RF', 'LH', 'RH', 'Base')
grid on
title('Position of the base and the feets')

%% make base xyz and rpy plot vs time

figure
subplot(2,1,1)
plot(time, euler(:,1), 'Linewidth',2);
hold on
plot(time, euler(:,2), 'Linewidth',2);
plot(time, euler(:,3), 'Linewidth',2);
legend('roll', 'pitch', 'yaw');
grid on
title('roll pitch yaw of the base');

subplot (2,1,2)
plot(time, xyzBase(:,1), 'Linewidth',2);
hold on
plot(time, xyzBase(:,2), 'Linewidth',2);
plot(time, xyzBase(:,3), 'Linewidth',2);
legend('x', 'y', 'z');
grid on
title('positions of the base');


%% make all the feets position plots
figure
subplot (2,2,1)
plot(time, xyzLF(:,1), 'Linewidth',2);
hold on
plot(time, xyzLF(:,2), 'Linewidth',2);
plot(time, xyzLF(:,3), 'Linewidth',2);
legend('x', 'y', 'z');
grid on
title('LF positions')

subplot (2,2,2)
plot(time, xyzRF(:,1), 'Linewidth',2);
hold on
plot(time, xyzRF(:,2), 'Linewidth',2);
plot(time, xyzRF(:,3), 'Linewidth',2);
legend('x', 'y', 'z');
grid on
title('RF positions')

subplot (2,2,3)
plot(time, xyzLH(:,1), 'Linewidth',2);
hold on
plot(time, xyzLH(:,2), 'Linewidth',2);
plot(time, xyzLH(:,3), 'Linewidth',2);
legend('x', 'y', 'z');
grid on
title('LH positions')

subplot (2,2,4)
plot(time, xyzRH(:,1), 'Linewidth',2);
hold on
plot(time, xyzRH(:,2), 'Linewidth',2);
plot(time, xyzRH(:,3), 'Linewidth',2);
legend('x', 'y', 'z');
grid on
title('RH positions')

%% make all the feets velocities plots

figure
subplot (2,2,1)
plot(time, velLF(:,1), 'Linewidth',2);
hold on
plot(time, velLF(:,2), 'Linewidth',2);
plot(time, velLF(:,3), 'Linewidth',2);
legend('x', 'y', 'z');
grid on
title('LF velocities')

subplot (2,2,2)
plot(time, velRF(:,1), 'Linewidth',2);
hold on
plot(time, velRF(:,2), 'Linewidth',2);
plot(time, velRF(:,3), 'Linewidth',2);
legend('x', 'y', 'z');
grid on
title('RF velocities')

subplot (2,2,3)
plot(time, velLH(:,1), 'Linewidth',2);
hold on
plot(time, velLH(:,2), 'Linewidth',2);
plot(time, velLH(:,3), 'Linewidth',2);
legend('x', 'y', 'z');
grid on
title('LH velocities')

subplot (2,2,4)
plot(time, velRH(:,1), 'Linewidth',2);
hold on
plot(time, velRH(:,2), 'Linewidth',2);
plot(time, velRH(:,3), 'Linewidth',2);
legend('x', 'y', 'z');
grid on
title('RH velocities')

%% plot positions for all the joints for each ee

figure
subplot(5,1,1)
plot(time, jointsLF(:,1), 'Linewidth',2);
hold on
plot(time, jointsLF(:,2), 'Linewidth',2);
plot(time, jointsLF(:,3), 'Linewidth',2);
legend('HAA', 'HFE', 'STEER');
grid on
title('LF joint positions')


subplot(5,1,2)
plot(time, jointsRF(:,1), 'Linewidth',2);
hold on
plot(time, jointsRF(:,2), 'Linewidth',2);
plot(time, jointsRF(:,3), 'Linewidth',2);
legend('HAA', 'HFE', 'STEER');
grid on
title('RF joint positions')

subplot(5,1,3)
plot(time, jointsLH(:,1), 'Linewidth',2);
hold on
plot(time, jointsLH(:,2), 'Linewidth',2);
plot(time, jointsLH(:,3), 'Linewidth',2);
legend('HFE', 'HAA', 'STEER');
grid on
title('LH joint positions')

subplot(5,1,4)
plot(time, jointsRH(:,1), 'Linewidth',2);
hold on
plot(time, jointsRH(:,2), 'Linewidth',2);
plot(time, jointsRH(:,3), 'Linewidth',2);
legend('HFE', 'HAA', 'STEER');
grid on
title('RH joint positions')

subplot(5,1,5)
plot(time, jointsBoom(:,1), 'Linewidth',2);
hold on
plot(time, jointsBoom(:,2), 'Linewidth',2);
plot(time, jointsBoom(:,3), 'Linewidth',2);
plot(time, jointsBoom(:,4), 'Linewidth',2);
legend('TURN', 'BOOM', 'DIPPER', 'TELE');
grid on
title('Boom joint positions')



