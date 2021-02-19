clear all; close all; clc;
num = 7; uniform = 1; cx = 0; cy = 1.5; cz = 1; sigx = 1; sigy = 0.5; sigz = 0.1;
targets = rand_targets(num,uniform,cx,cy,cz,sigx,sigy,sigz);
plot3(targets(:,1),targets(:,2),targets(:,3),'.')
axis equal; xlabel('X'); ylabel('Y'); zlabel('Z'); 
if uniform
    title(strcat('Uniform Distribution: Center = (', num2str(cx),', ', num2str(cy),', ', num2str(cz),'), Spread = (',num2str(sigx),', ', num2str(sigy),', ', num2str(sigz),')'));
else
    title(strcat('Normal Distribution: Center = (', num2str(cx),', ', num2str(cy),', ', num2str(cz),'), Spread = (',num2str(sigx),', ', num2str(sigy),', ', num2str(sigz),')'));
end