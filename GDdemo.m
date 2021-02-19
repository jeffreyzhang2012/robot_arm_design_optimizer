clear all; close all; clc

num = 14; uniform = 1; cx = 0; cy = 4.5; cz = 1; sigx = 10; sigy = 2; sigz = 5;
targets = rand_targets(num,uniform,cx,cy,cz,sigx,sigy,sigz);
num_tests = 20;
avgs = zeros(5,1);
population = 20;
dof = 5;
best_cost = 1e4;
tic
j = 0
avg = 0;
while toc < 900
    DH_init = rand_DH(dof,2/dof,0.3);
    parent = DH_init;
%         [DH,cost] = GA_optimizer_time(targets,DHs,10,population,0,1, T);
    [DH,cost] = GD_optimizer(targets,DH_init,dof,10);
    cost = min(cost)
    avg = avg + cost;
    j = j + 1;
    if cost < best_cost
        best_DH = DH;
        best_cost = cost;
        best_parent = parent;
    end
end
avg = avg/j
robot = SerialLink(DH);
IK_traj(robot,dof,targets);
robot_parent = SerialLink(best_parent);
IK_traj(robot_parent,dof,targets);