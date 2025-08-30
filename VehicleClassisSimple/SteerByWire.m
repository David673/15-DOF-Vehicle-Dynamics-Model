function [ddx_rack,T_assist] = SteerByWire(Twz1,Twz2,dx_rack,fz1,fz2,delta,x_rack)

H = 0.3;%H为人为设定的参数，齿条与主销中心点的平行距离【m】
ceq = 0.002*57.3; % 转向系统阻尼N/(m/s)
Meq = 10*2;%等效质量kg
Tfri1 = 1*StribeckFri(fz1,dx_rack);%转向系统摩擦
Tfri2 = 1*StribeckFri(fz2,dx_rack);%转向系统摩擦

ratio_S = 44.12/1000/(2*pi);%齿轮齿条速比m/rad
K_assist = 3000/(4*pi/180);%助力系数N/rad
T_assist = H * K_assist * (delta - x_rack/ratio_S);%助力力矩
%转向系统动力学方程

ddx_rack = (Twz1 + Twz2 - ceq * dx_rack * H + Tfri1 + Tfri2 + T_assist)/(Meq * H);