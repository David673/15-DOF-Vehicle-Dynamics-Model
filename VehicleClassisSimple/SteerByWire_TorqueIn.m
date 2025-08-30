function [ddx_rack,T_assist, dddelta] = SteerByWire_TorqueIn(Twz1,Twz2,dx_rack,fz1,fz2,delta,x_rack,Tin,ddelta)
Isw = 0.02;%ת������ת������ܹ�����kg*m2��0.02
K_feedback = 15/300;
ColDamping = 3*0.002*57.3;%ת���������ϵ��Nm*s/rad,0.002*57.3


H = 0.3;%HΪ��Ϊ�趨�Ĳ������������������ĵ��ƽ�о��롾m��
ceq = 0.002*57.3; % ת��ϵͳ���᡾N/(m/s)��0.002
Meq = 10*2;%��Ч������kg��10
Tfri1 = StribeckFri(fz1,dx_rack);%ת��ϵͳĦ����Nm��
Tfri2 = StribeckFri(fz2,dx_rack);%ת��ϵͳĦ����Nm��

ratio_S = 44.12/1000/(2*pi);%���ֳ����ٱȡ�m/rad��
K_assist = 3000/(4*pi/180);%����ϵ����N/rad��
T_assist = H * K_assist * (delta - x_rack/ratio_S);%�������ء�Nm��


%ת��ϵͳ����ѧ���̡�m/s2��
Tfeedback = K_feedback * T_assist;
dddelta = 1/Isw * (Tin - Tfeedback - ColDamping * ddelta);

ddx_rack = (Twz1 + Twz2 - ceq * dx_rack * H + Tfri1 + Tfri2 + T_assist)/(Meq * H);