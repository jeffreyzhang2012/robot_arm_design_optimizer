function targets = rand_targets(num,uniform,cx,cy,cz,sigx,sigy,sigz)
% uniform vs normally distributed
% uniform--sig becomes range
targets = zeros(num,3);
if uniform
    targets(:,1) = (rand(num,1) - 0.5) * sigx + cx;
    targets(:,2) = (rand(num,1) - 0.5) * sigy + cy;
    targets(:,3) = (rand(num,1) - 0.5) * sigz + cz;
else
    targets(:,1) = randn(num,1) * sigx + cx;
    targets(:,2) = randn(num,1) * sigy + cy;
    targets(:,3) = randn(num,1) * sigz + cz;
end
end