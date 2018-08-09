clc;

addpath('matlab_rosbag-0.4-linux64');
clear rosbag_wrapper;
clear ros.Bag;
clear all
close all


% Using load() lets you auto-complete filepaths.
filename = '/home/jelavice/Documents/catkin_workspaces/towr_ws/src/towr/towr_trajectory.bag';
filename = '/home/jelavice/.ros/towr_trajectory.bag';
bag = ros.Bag.load(filename);
bag.info()

topic1 = '/xpp/state_des';
topic2 = '/xpp/terrain_info';
msgs = bag.readAll({topic1, topic2});

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

%robotCartesian = bag.definition('xpp_msgs/RobotStateCartesian')

% Setting 'raw' to true shows the original message definition with comments
% raw = true;
% raw_twist_definition = bag.definition('geometry_msgs/Twist', raw)

% When it's unambiguous, you can drop the package name.  You can also get
% definitions for messages defined in other messages;
% geometry_msgs/Quaternion comes from geometry_msgs/Twist
%quaternion_definition = bag.definition('Quaternion', true)

%% Convert velocity messages to a matrix to plot linear speed
[msgs, meta] = bag.readAll(topic1); % Messages are structs
accessor = @(pose) pose.base.pose.position;
[xyz] = ros.msgs2mat(msgs, accessor); % Convert struct to 3-by-N matrix of linear position
xyz = xyz';
times = cellfun(@(x) x.time.time, meta); % Get timestamps

% Make xy plot in the plane 
subplot(2,1,1)
plot(times,xyz(:,1), 'linewidth', 2)
hold on
plot(times, xyz(:,2), 'linewidth',2);
legend('vx', 'vy')
grid on
subplot(2,1,2)
plot(xyz(:,1), xyz(:,2), 'LineWidth', 2);
grid on

%get the quaternion
accessor = @(pose) pose.base.pose.orientation;
[q] = (ros.msgs2mat(msgs, accessor))';
%swap the columns because matlab has a [w x y z] quaternion representation
temp = q(:,1);
q(:,1) = q(:,4);
q(:,4) = temp;

ypr = quat2eul(q,'ZYX');
clear temp;
figure
subplot(3,1,1)
plot(times, ypr(:,3), 'LineWidth', 2);
title('roll')
grid on
subplot(3,1,2)
plot(times, ypr(:,2), 'LineWidth', 2);
title('pitch')
grid on
subplot(3,1,3)
plot(times, ypr(:,1), 'LineWidth', 2);
title('yaw')
grid on

%plot the trajectory of all 4 end-effectors
accessor = @(feet) (feet.ee_motion(1).pos);
[xyzLF] = (ros.msgs2mat(msgs, accessor))';
accessor = @(feet) (feet.ee_motion(2).pos);
[xyzRF] = (ros.msgs2mat(msgs, accessor))';
accessor = @(feet) (feet.ee_motion(3).pos);
[xyzLH] = (ros.msgs2mat(msgs, accessor))';
accessor = @(feet) (feet.ee_motion(4).pos);
[xyzRH] = (ros.msgs2mat(msgs, accessor))';

%now plot them in the x-y plane
figure
plot(xyzLF(:,1), xyzLF(:,2), 'Linewidth', 2)
hold on
plot(xyzRF(:,1), xyzRF(:,2), 'Linewidth', 2)
plot(xyzLH(:,1), xyzLH(:,2), 'Linewidth', 2)
plot(xyzRH(:,1), xyzRH(:,2), 'Linewidth', 2)
grid on

%plot the velocities trajectory of all 4 end-effectors
accessor = @(feet) (feet.ee_motion(1).vel);
[velLF] = (ros.msgs2mat(msgs, accessor))';
accessor = @(feet) (feet.ee_motion(2).vel);
[velRF] = (ros.msgs2mat(msgs, accessor))';
accessor = @(feet) (feet.ee_motion(3).vel);
[velLH] = (ros.msgs2mat(msgs, accessor))';
accessor = @(feet) (feet.ee_motion(4).vel);
[velRH] = (ros.msgs2mat(msgs, accessor))';
figure
subplot(4,1,1)
plot(times, velLF(:,1), 'Linewidth',2)
hold on
plot(times, velLF(:,2), 'Linewidth',2)
legend('vx', 'vy');
grid on

title('LF velocity')
subplot(4,1,2)
plot(times, velRF(:,1), 'Linewidth',2)
hold on
plot(times, velRF(:,2), 'Linewidth',2)
legend('vx', 'vy');
grid on

title('RF velocity')
subplot(4,1,3)
plot(times, velLH(:,1), 'Linewidth',2)
hold on
plot(times, velLH(:,2), 'Linewidth',2)
legend('vx', 'vy');
grid on

title('LH velocity')
subplot(4,1,4)
plot(times, velRH(:,1), 'Linewidth',2)
hold on
plot(times, velRH(:,2), 'Linewidth',2)
legend('vx', 'vy');
title('RH velocity')
grid on


%plot the wheel angles
accessor = @(feet) (feet.wheel_angles(1));
[headingLF] = (ros.msgs2mat(msgs, accessor))';
accessor = @(feet) (feet.wheel_angles(2));
[headingRF] = (ros.msgs2mat(msgs, accessor))';
accessor = @(feet) (feet.wheel_angles(3));
[headingLH] = (ros.msgs2mat(msgs, accessor))';
accessor = @(feet) (feet.wheel_angles(4));
[headingRH] = (ros.msgs2mat(msgs, accessor))';
figure
subplot(4,1,1)
plot(times, headingLF, 'Linewidth',2)
grid on
title('LF heading')
subplot(4,1,2)
plot(times, headingRF, 'Linewidth',2)
grid on
title('RF heading')
subplot(4,1,3)
plot(times, headingLH, 'Linewidth',2)
grid on
title('LH heading')
subplot(4,1,4)
plot(times, headingRH, 'Linewidth',2)
grid on
title('RH heading')

% 
% 
% %plot the velocities trajectory of all 4 end-effectors
% accessor = @(forces) (forces.ee_forces(:,1));
% [fLF] = (ros.msgs2mat(msgs, accessor))';
% accessor = @(forces) (forces.ee_forces(:,2));
% [fRF] = (ros.msgs2mat(msgs, accessor))';
% accessor = @(forces) (forces.ee_forces(:,3));
% [fLH] = (ros.msgs2mat(msgs, accessor))';
% accessor = @(forces) (forces.ee_forces(:,4));
% [fRH] = (ros.msgs2mat(msgs, accessor))';
% 
% figure
% subplot(4,1,1)
% plot(times, fLF(:,1), 'Linewidth',2)
% hold on
% plot(times, fLF(:,2), 'Linewidth',2)
% legend('fx', 'fy');
% grid on
% title('LF forces')
% 
% 
% subplot(4,1,2)
% plot(times, fRF(:,1), 'Linewidth',2)
% hold on
% plot(times, fRF(:,2), 'Linewidth',2)
% legend('fx', 'fy');
% grid on
% title('RF forces')
% 
% subplot(4,1,3)
% plot(times, fLH(:,1), 'Linewidth',2)
% hold on
% plot(times, fLH(:,2), 'Linewidth',2)
% legend('fx', 'fy');
% grid on
% title('LH forces')
% 
% subplot(4,1,4)
% plot(times, fRH(:,1), 'Linewidth',2)
% hold on
% plot(times, fRH(:,2), 'Linewidth',2)
% legend('fx', 'fy');
% grid on
% title('RH forces')

%plot the positions trajectory of all 4 end-effectors


figure
subplot(4,1,1)
plot(times, xyzLF(:,1), 'Linewidth',2)
hold on
plot(times, xyzLF(:,2), 'Linewidth',2)
legend('x', 'y');
grid on
title('LF pos')


subplot(4,1,2)
plot(times, xyzRF(:,1), 'Linewidth',2)
hold on
plot(times, xyzRF(:,2), 'Linewidth',2)
legend('x', 'y');
grid on
title('RF pos')

subplot(4,1,3)
plot(times, xyzLH(:,1), 'Linewidth',2)
hold on
plot(times, xyzLH(:,2), 'Linewidth',2)
legend('x', 'y');
grid on
title('LH pos')

subplot(4,1,4)
plot(times, xyzRH(:,1), 'Linewidth',2)
hold on
plot(times, xyzRH(:,2), 'Linewidth',2)
legend('x', 'y');
grid on
title('RH pos')











