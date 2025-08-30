function T_fri = StribeckFri(Fn,dx)
%% ��֪����
mu_s = 0.02; %��Ħ��ϵ��
mu_d = 0.018; %��Ħ��ϵ��
vs = 0.001; %��ʼ���̵��ٶ�m/s
vss = 0.005; %stribeck�ٶ�m/s
delta = 2; %stribeckϵ��
R_kin = 30/1000; %����ƽ��Ħ��ֱ��m
%% ���
Fc = -mu_d * Fn * sign(dx);
Fs = -mu_s * Fn * sign(dx);
if abs(dx) <= vs
    Ff = -mu_s * Fn * (dx/vs); %��Ħ�����Ľ��Ʊ�ʾ
else
    Ff = Fc + (Fs - Fc) * exp(-(dx/vss)^delta); %��Ħ�������ٶȷ����෴
end
T_fri = Ff * R_kin;
