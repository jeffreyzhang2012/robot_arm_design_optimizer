function DH = rand_DH(dof, len_range, len_sd)
% theta d a alpha
DH = zeros(dof,4);
DH(:,2) = abs(randn(dof,1)*len_sd + len_range);
DH(:,3) = abs(randn(dof,1)*len_sd + len_range);
% DH(:,1) = (rand(dof,1)-0.5) * 2 * pi;
DH(:,4) = (rand(dof,1)-0.5) * 2 * pi;
end