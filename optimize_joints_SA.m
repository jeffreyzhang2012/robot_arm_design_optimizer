clear all; close all; clc;
% theta d a alpha
num_start = 5;
num_iter = 40;
alpha0 = 0.5;
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
    DH_new = DH;
    DH_parent = DH;
    robot = SerialLink(DH);
    cost_parent = cost_fn(robot,targets);
    T = cost_parent * 0.2;
    j = 0;
    grad = zeros(3,3); 
    delta_c = 1;
    alpha = alpha0;
    costs = zeros(num_iter,1);
    j = 1;
    while T >= 0
        T = T - 0.015;
        grad = zeros(3,3); 
        c0 = cost_fn(robot,targets);
        if j == 1
            fprintf('Initial c: %f\n',c0);
        end
        del_q = (rand(dof,3) - 0.5);
        DH_new(:,2:end) = DH(:,2:end) + alpha * del_q;
        robot_new = SerialLink(DH_new);
        c_new = cost_fn(robot_new,targets);
        delta_c = c_new - c0;
        p = 1/(1+exp(delta_c/T));
        fprintf('del c: %f, p: %f\n',delta_c,p);
        if p > rand
            c0 = c_new;
            robot = robot_new;
            DH = DH_new;
            costs(j) = c0;
            j = j + 1;
        end
    end
    figure; plot(costs(1:j-1));
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