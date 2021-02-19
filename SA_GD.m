clear all; close all; clc

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
seed = rand_DH(dof,1,0.2);
R = SerialLink(seed);
while cost_fn(R,targets) >= 25
    seed = rand_DH(dof,1,0.2);
    R = SerialLink(seed);
end
[DH, costs_SA] = SA_optimizer(targets, seed, dof);
[DH, costs_GD] = GD_optimizer(targets, DH, dof, 80);
[DH_GDonly, costs_GD_only] = GD_optimizer(targets, seed, dof, 80);
robot = SerialLink(DH);
% IK_traj(robot,dof,targets);
costs = [costs_SA',costs_GD'];
figure; plot(costs);
title('Simulated Annealing + Gradient Descent');
xlabel('Descent Iteration'); ylabel('Cost');pause(0.1);
fprintf('Final c: %f\n',min(costs));
line([size(costs_SA,1),size(costs_SA,1)],[min(costs),max(costs)])

figure; plot(costs_GD_only');
title('Gradient Descent Only');
xlabel('Descent Iteration'); ylabel('Cost');pause(0.1);
fprintf('Final gd only c: %f\n',min(costs_GD_only));
