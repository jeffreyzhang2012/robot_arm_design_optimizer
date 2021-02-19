clear all; close all; clc;
% theta d a alpha
num = 14; uniform = 1; cx = 0; cy = 4.5; cz = 1; sigx = 10; sigy = 2; sigz = 5;
targets = rand_targets(num,uniform,cx,cy,cz,sigx,sigy,sigz);
num_tests = 30;
avgs = zeros(5,1);
population = 20;
dof = 5;
best_cost = 1e4;
% for i = 1:num_tests
tic
while toc < 900
    toc
    DHs = zeros(5,4,population);
    for i = 1:population
        DHs(:,:,i) = rand_DH(dof,1,0.2);
    end
    [DH,cost] = GA_optimizer(targets,DHs,10,population,0,1);
    cost = min(cost)
%     avg = avg + cost;
    if cost < best_cost
        best_DH = DH;
        best_cost = cost;
%         best_parent = parent;
    end
end
robot = SerialLink(DH);
IK_traj(robot,dof,targets);
robot_parent = SerialLink(best_parent);
IK_traj(robot_parent,dof,targets);