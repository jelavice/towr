clc;

addpath('matlab_rosbag-0.4-linux64');
clear rosbag_wrapper;
clear ros.Bag;
clear all
close all


% Using load() lets you auto-complete filepaths.
filename = '/home/jelavice/Documents/catkin_workspaces/towr_ws/src/towr/towr_trajectory.bag';
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
plot(xyz(:,1), xyz(:,2), 'LineWidth', 2);
ylim([-2 2]);
xlim([-2 2]);
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
%get the quaternion
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














