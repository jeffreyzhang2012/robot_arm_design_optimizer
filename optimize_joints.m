clear all; close all; clc;
% theta d a alpha
num_start = 10;
num_iter = 40;
alpha0 = 5;
delta = 0.01;
dof = 3;
% targets = [2.5, 2,  1;
%      2.2,-1,1.5;
%      2.4,-3,  2;
%      2.3, 0,  1.6;];
 targets = [4, 2,  1;
     2.2,2,4;
     -2,3,  3;
     -3, 0,  1.6;];
min_cost = 10000;
best_parent = rand_DH(dof,1,0.2); %useless
for i = 1:num_start
    DH = rand_DH(dof,1,0.2);
    DH_parent = DH;
    robot = SerialLink(DH);
    cost_parent = cost_fn(robot,targets);
    j = 0;
    grad = zeros(3,3); 
    delta_c = 1;
    alpha = alpha0;
    costs = zeros(num_iter,1);
    while j < num_iter && abs(delta_c) > 0.01
        j = j + 1;
        grad = zeros(3,3); 
        c0 = cost_fn(robot,targets);
        if j == 1
            fprintf('Initial c: %f\n',c0);
        end
        for dx = 1:3
            for dy = 1:dof
                DH_del_dydx = DH;
                DH_del_dydx(dy,dx+1) = DH_del_dydx(dy,dx+1) + delta;
                robot_del = SerialLink(DH_del_dydx);
                grad(dy,dx) = (cost_fn(robot_del,targets) - c0);
            end
        end
        DH(:,2:end) = DH(:,2:end) - alpha * grad;
        robot = SerialLink(DH);
        c = cost_fn(robot,targets);
        delta_c = c - c0;
        if delta_c > 0
            DH(:,2:end) = DH(:,2:end) + alpha * grad;
            robot = SerialLink(DH);
            alpha = 0.5 * alpha;
        end
        costs(j) = c;
    end
    figure; plot(costs(1:j));
    xlabel('Descent Iteration'); ylabel('Cost');pause(0.1);
    fprintf('Final c: %f\n',c0);
    if c0 < min_cost
        min_cost = c0;
        best_parent_cost = cost_parent;
        fprintf('New lowest\n');
        min_DH = DH;
        best_parent = DH_parent;
    end
end
best_parent
min_DH
best_parent_cost
min_cost
robot = SerialLink(min_DH);
IK_traj(robot,dof,targets);