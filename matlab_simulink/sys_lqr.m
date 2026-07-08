% LQR反馈矩阵拟合
clear;close all;clc;

L0s = 0.1:0.01:0.5;

for step = 1:length(L0s)
    Ks(:,:,step) = get_k_length(L0s(step));
end

K = sym('K',[2 6]);
syms L0;
i = 0;
for x = 1:2
    for y = 1:6
        i = i+1;
        p(i,:) = polyfit(L0s,reshape(Ks(x,y,:),1,length(L0s)),3);
        K(x,y) = p(i,1) * L0^3 + p(i,2) * L0^2 + p(i,3) *L0 + p(i,4);
    end
end

vpa(subs(K,L0,0.25))