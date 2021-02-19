function IK_traj(robot,dof,targets_old)
T0 = robot.fkine(zeros(1,dof));
targets = zeros(size(targets_old,1)+1,3);
targets(2:end,:) = targets_old;
targets(1,:) = T0.t;
q = zeros(1,dof);
waypoints = zeros(size(targets,1), 4);
step = 50;
Qs = zeros((size(targets,1)-1) * step, dof);
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
    plot_sphere(targets(i,:),0.2,'yellow');
end
robot.plot(zeros(1,dof));
pause(4);
robot.plot(Qs);
end

