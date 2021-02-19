function [best_DH, best_cost] = GA_optimizer(targets,DHs,num_iter,population,nearest,total_gene)
pop_cost = zeros(population,1);
for i = 1:population
    robot = SerialLink(DHs(:,:,i));
    pop_cost(i) = cost_fn(robot,targets);
end
offsprings = zeros(size(DHs,1),size(DHs,2),population/2);
off_costs = zeros(population/2,1);
i = 0;
while i <= num_iter
    i = i + 1;
    mate_sequence = randperm(population);
    for j = 1:2:population
        if ~nearest
            gene_select = rand(size(DHs(:,:,1))) > 0.5;
            if total_gene
                DH_off = DHs(:,:,mate_sequence(j)).*gene_select + DHs(:,:,mate_sequence(j+1)).*(1-gene_select); % total gene change
            else
                DH_off = (DHs(:,:,mate_sequence(j)) + DHs(:,:,mate_sequence(j)))./2; % average gene change
            end
%         else
%             DH_off = (DHs(:,:,j)+DHs(:,:,j+1))./2;
        end
        DH_off_sub = DH_off(:,2:end);
        DH_off(:,2:end) = abs(DH_off_sub * (1 + (rand(3,3)-0.5)*(rand(3,3)>0.7)));
        offsprings(:,:,(j+1)/2) = DH_off;
        off_costs((j+1)/2) = cost_fn(SerialLink(DH_off),targets);
    end
    total_costs = [pop_cost; off_costs];
    total_DHs = cat(3,DHs, offsprings);
    [sorted, order] = sort(total_costs);
    DHs = total_DHs(:,:,order(1:population));
    pop_cost = sorted(1:population);
end
best_DH = DHs(:,:,1);
best_cost = pop_cost(1);
end