clear all; close all; clc;
% theta d a alpha
DH_params = [0,3,3,pi/4;
             pi/4,2,2,0;
             0,3,1,pi/4;
             0,2,0,0;];
robot = SerialLink(DH_params);

T0 = robot.fkine([0,0,0,0]);
% targets = [0,0,0;
%     7,-6,6;
%     8,-5,5;
%     6,-3,6;
%     4,1,3;]
targets = [0,0,0;
    8,-1.46,6.93;
    7.615,-0.451,8.024;
    6.421,3.132,8.361;
    2.735,4.817,9.684;];
targets(1,:) = T0.t;

q = [0,0,0,0];
waypoints = zeros(size(targets,1), 4);
step = 50;
Qs = zeros((size(targets,1)-1) * step, 4);
Ps = zeros((size(targets,1)-1) * step, 3);
for j = 1:size(targets,1)-1
    Ps_waypoint = zeros(step,3);
    for x = 1:3
        Ps_waypoint(:,x) = linspace(targets(j,x),targets(j+1,x),step);
    end
    st_idx = (j-1)*step+1;
    end_idx = st_idx + step - 1;
    Ps(st_idx:end_idx,:) = Ps_waypoint;
end
alpha = 3;
I = eye(3);
L = 0.1;
for i = 1:size(Qs,1)-1
    cur_H = robot.fkine(Qs(i,:));
%     del_p = Ps(i+1,:) - Ps(i,:);
    err_p = Ps(i,:)' - cur_H.t;
    J_p = robot.jacob0(Qs(i,:));
    J_p = J_p(1:3,:);
    Qs(i+1,:) = Qs(i,:) + 0.4 * (J_p' * inv(J_p * J_p' + L^2 * I) * err_p)';
%     Qs(i+1,:) = Qs(i,:) + 0.1 * (J_p' * inv(J_p * J_p') * err_p)';
end
for i = 1:size(targets,1)
    plot_sphere(targets(i,:),1,'yellow');
end
robot.plot(Qs);