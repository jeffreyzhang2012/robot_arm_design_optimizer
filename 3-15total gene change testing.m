clear all; close all; clc

cost_change_avg = 0;
cost_nochange_avg = 0;
population = 10;
trials = 20;
generation = 10;
costs_change = zeros(trials,1);
costs_nochange = zeros(trials,1);
for k = 1:trials
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
    DHs = zeros(3,4,population);
    for i = 1:population
        DHs(:,:,i) = rand_DH(dof,1,0.2);
    end
    [DH_change,cost_change] = GA_optimizer(targets,DHs,generation,population,0,1);
    [DH_nochange,cost_nochange] = GA_optimizer(targets,DHs,generation,population,0,0);
    costs_change(k) = cost_change;
    costs_nochange(k) = cost_nochange;
end
figure;
plot(costs_change);
hold on;
plot(costs_nochange);
legend('Total Gene Change','Average Gene');
xlabel('Trial number'); ylabel('Cost');
title(strcat('Total Gene Change = ', num2str(mean(costs_change)), ' Average Gene = ',num2str(mean(costs_nochange))));