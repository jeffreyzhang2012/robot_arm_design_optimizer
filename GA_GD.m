clear all; close all; clc

cost_far_avg = 0;
cost_near_avg = 0;
cost_GD_avg = 0;
for k = 1:1
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
    population = 10;
    DHs = zeros(3,4,population);
    for i = 1:population
        DHs(:,:,i) = rand_DH(dof,1,0.2);
    end
%     [DH_near,cost_near] = GA_optimizer(targets,DHs,10,population,1);
    [DH_far,cost_far] = GA_optimizer(targets,DHs,10,population,0);
    cost_far_avg = cost_far_avg + cost_far;
    best_gd_one = 1e5; 
    for i = 1:population
        [DH_gd, cost_GD_i] = GD_optimizer(targets,DHs(:,:,i),dof,20);
        if min(cost_GD_i) < best_gd_one
            best_gd_one = min(cost_GD_i);
        end
    end
    cost_GD_avg = cost_GD_avg + best_gd_one;
    best_sagd_one = 1e5;
    for i = 1:population
        [DH_sagd, cost_SAGD_i] = GD_optimizer(targets,DH_far,dof,20);
        if cost_SAGD_i < best_sagd_one
            best_sagd_one = cost_SAGD_i;
        end
    end
end
% cost_far_avg = cost_far_avg / 10
% cost_near_avg = cost_near_avg / 10
% cost_GD_avg = cost_GD_avg / 10
best_sagd_one