function [ddx_rack,T_assist] = SteerByWire(Twz1,Twz2,dx_rack,fz1,fz2,delta,x_rack)

H = 0.3;%HΪ��Ϊ�趨�Ĳ������������������ĵ��ƽ�о��롾m��
ceq = 0.002*57.3; % ת��ϵͳ����N/(m/s)
Meq = 10*2;%��Ч����kg
Tfri1 = 1*StribeckFri(fz1,dx_rack);%ת��ϵͳĦ��
Tfri2 = 1*StribeckFri(fz2,dx_rack);%ת��ϵͳĦ��

ratio_S = 44.12/1000/(2*pi);%���ֳ����ٱ�m/rad
K_assist = 3000/(4*pi/180);%����ϵ��N/rad
T_assist = H * K_assist * (delta - x_rack/ratio_S);%��������
%ת��ϵͳ����ѧ����

ddx_rack = (Twz1 + Twz2 - ceq * dx_rack * H + Tfri1 + Tfri2 + T_assist)/(Meq * H);