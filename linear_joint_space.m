clear all; close all; clc;
% theta d a alpha
DH_params = [0,3,3,pi/4;
             pi/4,2,2,0;
             0,3,1,pi/4;
             0,2,0,0;];
robot = SerialLink(DH_params);
% 
targets = [3,3,5;
    -2,4,1;
    -3,3,6;
    -1,2,5;];
% T0 = robot.fkine([0,0,0,0]);
% targets = [0,0,0;
%     7,-6,7;
%     8,-5,4;
%     6,-3,5;
%     4,1,4]
% targets(1,:) = T0.t+1;
q = [0,0,0,0];
waypoints = zeros(size(targets,1), 4);
step = 100;
Qs = zeros(size(targets,1) * step, 4);
k = 1;
for j = 1:size(targets,1)
    target = targets(j,:);
    T = eye(4);
    T(1:3,4) = target;
    prev_q = q;
    [q,err,exitflag] = robot.ikunc(T);
    Qs_waypoint = zeros(step,4);
    for i = 1:4
        Qs_waypoint(:,i) = linspace(prev_q(i),q(i),step);
    end
    st_idx = (j-1)*step+1;
    end_idx = st_idx + step - 1;
    Qs(st_idx:end_idx,:) = Qs_waypoint;
end
for i = 1:size(targets,1)
    plot_sphere(targets(i,:),1,'yellow');
end
robot.plot(Qs);
% plot_sphere([3,3,5],1,'yellow');