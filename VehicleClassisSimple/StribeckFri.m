function T_fri = StribeckFri(Fn,dx)
%% 已知条件
mu_s = 0.02; %静摩擦系数
mu_d = 0.018; %动摩擦系数
vs = 0.001; %初始过程的速度m/s
vss = 0.005; %stribeck速度m/s
delta = 2; %stribeck系数
R_kin = 30/1000; %主销平均摩擦直径m
%% 求解
Fc = -mu_d * Fn * sign(dx);
Fs = -mu_s * Fn * sign(dx);
if abs(dx) <= vs
    Ff = -mu_s * Fn * (dx/vs); %静摩擦力的近似表示
else
    Ff = Fc + (Fs - Fc) * exp(-(dx/vss)^delta); %动摩擦力与速度方向相反
end
T_fri = Ff * R_kin;
