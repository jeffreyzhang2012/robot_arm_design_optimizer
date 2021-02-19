function [best_DH,min_c] = SA_optimizer(targets,DH,dof)
DH_new = DH;
robot = SerialLink(DH);
cost_parent = cost_fn(robot,targets);
T = cost_parent * 0.2;
j = 0;
alpha = 2;
max_iter = 1000;
% costs = [];
j = 1;
min_c = 1e10;
num_iter = 10;
while T >= 0 && j < num_iter
    T = T - 1;
    c0 = cost_fn(robot,targets);
%     if j == 1
%         fprintf('Initial c: %f\n',c0);
%     end
    del_q = (rand(dof,3) - 0.5);
    DH_new(:,2:end) = DH(:,2:end) + alpha * del_q;
    robot_new = SerialLink(DH_new);
    c_new = cost_fn(robot_new,targets);
    delta_c = c_new - c0;
    p = 1/(1+exp(delta_c/abs(T)));
%     fprintf('del c: %f, p: %f\n',delta_c,p);
    if c_new < min_c
        min_c = c_new;
        best_DH = DH_new;
    end
    if p > rand
        c0 = c_new;
        robot = robot_new;
        DH = DH_new;
%         costs(j) = c0;
        j = j + 1;
    end
end
% costs = costs(1:j-1);
% costs = min(costs)
end