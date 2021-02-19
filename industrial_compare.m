clear all; close all; clc

num = 14; uniform = 1; cx = 0; cy = 4.5; cz = 1; sigx = 10; sigy = 2; sigz = 5;
targets = rand_targets(num,uniform,cx,cy,cz,sigx,sigy,sigz);
num_tests = 10;
avgs = zeros(5,1);
dof = 6;
DH_kuka  = [0,0.675,0.26,-pi/2;
    0,0,0.68,0;
    0,0,0.035,-pi/2;
    0,0.67,0,pi/2;
    0,0,0,-pi/2;
    0,.115,0,0]; % kuka
DH_ur  = [0,0.089,0,pi/2;
    0,0,-0.425,0;
    0,0,-0.39225,0;
    0,0.10915,0,pi/2;
    0,0.09465,0,-pi/2;
    0,0.0823,0,0]; % ur5
DH_kuka(:,2:3) = DH_kuka(:,2:3) * 4;
DH_ur(:,2:3) = DH_ur(:,2:3) * 16;
% DH_mother = DH_ur;
DH_mother = DH_kuka
DH = DH_mother;

% for i = -10:10
%     DH(:,2:3) = DH_mother(:,2:3) * (1+i*0.001);
%     DH_mother
%     cost_fn(SerialLink(DH),targets)
%     cost_fn(SerialLink(DH_mother),targets)
%     i
% end
num_iter = 20;
delta = 0.01; alpha = 0.002;
% DH_del = DH_mother;
% DH_del(:,1) = 0; DH_del(:,4) = 0;
% DH_del(:,2:3) = DH_del(:,2:3) + DH_mother(:,2:3) * delta;
robot = SerialLink(DH);
c = cost_fn(robot,targets);
for i = 1:num_iter
    DH_del = DH;
    DH_del(:,2:3) = DH_del(:,2:3) * 1.01;
    grad = cost_fn(SerialLink(DH_del),targets) - c;
    DH(:,2:3) = DH(:,2:3) * (1-alpha*grad)
%     DH = DH - alpha * grad * DH_del;
    robot = SerialLink(DH);
    c_new = cost_fn(robot,targets);
    delta_c = c - c_new;
    c = c_new
    if delta_c < 0
        DH(:,2:3) = DH(:,2:3)/((1-alpha*grad));
        robot = SerialLink(DH);
        alpha = 0.5 * alpha;
    end
end

robot = SerialLink(DH);
IK_traj(robot,6,targets);