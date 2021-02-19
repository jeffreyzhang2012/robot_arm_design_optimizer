function [DH,costs] = GD_optimizer(targets,DH,dof,num_iter)
robot = SerialLink(DH);
j = 0;
delta_c = 1;
delta = 0.05;
alpha = 0.5;
costs = zeros(num_iter,1);
while j < num_iter && abs(delta_c) > 0.1
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
    c = cost_fn(robot,targets)
    delta_c = c - c0;
    if delta_c > 0
        DH(:,2:end) = DH(:,2:end) + alpha * grad;
        robot = SerialLink(DH);
        alpha = 0.5 * alpha;
    end
    costs(j) = c;
end
costs = costs(1:j)

end