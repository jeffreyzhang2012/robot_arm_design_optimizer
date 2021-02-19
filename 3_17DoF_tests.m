clear all; close all; clc;

num = 7; uniform = 1; cx = 0; cy = 1.5; cz = 1; sigx = 1; sigy = 0.5; sigz = 0.1;
targets = rand_targets(num,uniform,cx,cy,cz,sigx,sigy,sigz);
num_tests = 10;
avgs = zeros(5,1);
population = 10;
T = 30;
for dof = 3:7
    avg = 0;
    best_cost = 1e5;
    best_DH = 0;
    for i = 1:num_tests
        DHs = zeros(dof,4,population);
        for i = 1:population
            DHs(:,:,i) = rand_DH(dof,2/dof,0.5);
        end
%         [DH,cost] = GA_optimizer_time(targets,DHs,10,population,0,1, T);
        [DH,cost] = GD_optimizer(targets,DHs(:,:,i),dof,20);
        cost = min(cost);
        avg = avg + cost;
        if cost < best_cost
            best_DH = DH;
            best_cost = cost;
        end
    end
    avg = avg / num_tests;
    avgs(dof - 2) = avg;
end
figure;
plot([3:7],avgs);
xlabel('DoF'); ylabel('Average Cost'); 
% title('Genetic Algorithm: DoF Test');
title('Gradient Descent: DoF Test');