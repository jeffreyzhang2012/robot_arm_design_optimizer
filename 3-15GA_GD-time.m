clear all; close all; clc

cost_change_avg = 0;
cost_nochange_avg = 0;
population = 10;
trials = 10;
generation = 10;
costs_ga = zeros(trials,1);
costs_gd = zeros(trials,1);
time_ga = zeros(trials,1);
time_gd = zeros(trials,1);
T = 200;
start = tic;
for k = 1:trials
    num_start = 5;
    num_iter = 40;
    alpha0 = 0.5;
    delta = 0.01;
    dof = 3;
    targets = [4, 2,  1;
         2.2,2,4;
         -2,3,  3;
         -3, 0,  1.6;];
    population = 10;
    DHs = zeros(3,4,population);
    for i = 1:population
        DHs(:,:,i) = rand_DH(dof,1,0.2);
    end
    DHs_more = zeros(3,4,population*3);    
    for i = 1:population*3
        DHs_more(:,:,i) = rand_DH(dof,1,0.2);
    end
    ga_st = toc(start);
    [DH_ga,cost_ga] = GA_optimizer_time(targets,DHs,10,population,0,1, T);
    ga_end = toc(start);
    time_ga(k) = ga_end - ga_st;
    costs_ga(k) = cost_ga;
    best_gd_one = 1e5; 
    gd = tic;
    i = 1;
    gd_st = toc(start);
    while toc(gd) < T * 0.6
        [DH_gd, cost_GD_i] = GD_optimizer(targets,DHs_more(:,:,i),dof,20);
        if min(cost_GD_i) < best_gd_one
            best_gd_one = min(cost_GD_i);
        end
        i = i + 1;
    end
    gd_end = toc(start);
    time_gd(k) = gd_end - gd_st;
    costs_gd(k) = best_gd_one;
    k
end
figure;
subplot(2,1,1);
plot(costs_ga);
hold on;
plot(costs_gd);
legend('Genetic Algorithm','Gradient Descent');
xlabel('Trial number'); ylabel('Cost');
title(strcat('Genetic Algorithm = ', num2str(mean(costs_ga)), ' Gradient Descent = ',num2str(mean(costs_gd))));

subplot(2,1,2);
plot(time_ga);
hold on;
plot(time_gd);
legend('Genetic Algorithm','Gradient Descent');
xlabel('Trial number'); ylabel('Time');
title(strcat('Genetic Algorithm = ', num2str(mean(time_ga)), ' Gradient Descent = ',num2str(mean(time_gd))));