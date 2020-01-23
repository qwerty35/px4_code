clear;
clc;
close all;

filename = '2020-01-10-18-09-11.bag';
bag = rosbag(filename);

% /mavros/local_position/pose
bSel_lp = select(bag, 'Topic', '/mavros/local_position/pose');
local_pose = readMessages(bSel_lp, 'DataFormat', 'struct');
X_lp = cellfun(@(m) double(m.Pose.Position.X), local_pose);
Y_lp = cellfun(@(m) double(m.Pose.Position.Y), local_pose);
Z_lp = cellfun(@(m) double(m.Pose.Position.Z), local_pose);
[R_lp, P_lp, Yaw_lp] = cellfun(@(m) quat2angle([double(m.Pose.Orientation.X) ...
                                                double(m.Pose.Orientation.Y) ...
                                                double(m.Pose.Orientation.Z) ...
                                                double(m.Pose.Orientation.W)]), local_pose);
T_lp = cellfun(@(m) double(m.Header.Stamp.Sec), local_pose);

% /mavros/setpoint_position/local
bSel_sp = select(bag, 'Topic', '/mavros/setpoint_position/local');
setpoint_pose = readMessages(bSel_sp, 'DataFormat', 'struct');
X_sp = cellfun(@(m) double(m.Pose.Position.X), setpoint_pose);
Y_sp = cellfun(@(m) double(m.Pose.Position.Y), setpoint_pose);
Z_sp = cellfun(@(m) double(m.Pose.Position.Z), setpoint_pose);
[R_sp, P_sp, Yaw_sp] = cellfun(@(m) quat2angle([double(m.Pose.Orientation.X) ...
                                                double(m.Pose.Orientation.Y) ...
                                                double(m.Pose.Orientation.Z) ...
                                                double(m.Pose.Orientation.W)]), setpoint_pose);
T_sp = cellfun(@(m) double(m.Header.Stamp.Sec), setpoint_pose);

% /mocap_node/cf1/pose
bSel_vi = select(bag, 'Topic', '/mavros/vision_pose/pose');
vision_pose = readMessages(bSel_vi, 'DataFormat', 'struct');
X_vi = cellfun(@(m) double(m.Pose.Position.X), vision_pose);
Y_vi = cellfun(@(m) double(m.Pose.Position.Y), vision_pose);
Z_vi = cellfun(@(m) double(m.Pose.Position.Z), vision_pose);
[R_vi, P_vi, Yaw_vi] = cellfun(@(m) quat2angle([double(m.Pose.Orientation.X) ...
                                                double(m.Pose.Orientation.Y) ...
                                                double(m.Pose.Orientation.Z) ...
                                                double(m.Pose.Orientation.W)]), vision_pose);
T_vi = cellfun(@(m) double(m.Header.Stamp.Sec), vision_pose);

% Time scaling
t0 = min([T_lp(1), T_sp(1), T_vi(1)]);
T_lp = T_lp - t0;
T_sp = T_sp - t0;
T_vi = T_vi - t0;

% Roll scaling
R_vi = R_vi - (R_vi > pi*0.9) * 2*pi;

% Plot data
figure;
subplot(3,2,1)
hold on
plot(T_lp, X_lp)
plot(T_sp, X_sp)
plot(T_vi, X_vi)
hold off
title('X')
legend('local','setpoint', 'vision');

subplot(3,2,2)
hold on
plot(T_lp, R_lp)
plot(T_sp, R_sp)
plot(T_vi, R_vi)
hold off
title('Roll')
legend('local','setpoint', 'vision');

subplot(3,2,3)
hold on
plot(T_lp, Y_lp)
plot(T_sp, Y_sp)
plot(T_vi, Y_vi)
hold off
title('Y')
legend('local','setpoint', 'vision');

subplot(3,2,4)
hold on
plot(T_lp, P_lp)
plot(T_sp, P_sp)
plot(T_vi, P_vi)
hold off
title('Pitch')
legend('local','setpoint', 'vision');

subplot(3,2,5)
hold on
plot(T_lp, Z_lp)
plot(T_sp, Z_sp)
plot(T_vi, Z_vi)
hold off
title('Z')
legend('local','setpoint', 'vision');

subplot(3,2,6)
hold on
plot(T_lp, Yaw_lp)
plot(T_sp, Yaw_sp)
plot(T_vi, Yaw_vi)
hold off
title('Yaw')
legend('local','setpoint', 'vision');