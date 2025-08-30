function [ddx_rack,T_assist, dddelta] = SteerByWire_TorqueIn(Twz1,Twz2,dx_rack,fz1,fz2,delta,x_rack,Tin,ddelta)
Isw = 0.02;%转向盘与转向管柱总惯量【kg*m2】0.02
K_feedback = 15/300;
ColDamping = 3*0.002*57.3;%转向管柱阻尼系数Nm*s/rad,0.002*57.3


H = 0.3;%H为人为设定的参数，齿条与主销中心点的平行距离【m】
ceq = 0.002*57.3; % 转向系统阻尼【N/(m/s)】0.002
Meq = 10*2;%等效质量【kg】10
Tfri1 = StribeckFri(fz1,dx_rack);%转向系统摩擦【Nm】
Tfri2 = StribeckFri(fz2,dx_rack);%转向系统摩擦【Nm】

ratio_S = 44.12/1000/(2*pi);%齿轮齿条速比【m/rad】
K_assist = 3000/(4*pi/180);%助力系数【N/rad】
T_assist = H * K_assist * (delta - x_rack/ratio_S);%助力力矩【Nm】


%转向系统动力学方程【m/s2】
Tfeedback = K_feedback * T_assist;
dddelta = 1/Isw * (Tin - Tfeedback - ColDamping * ddelta);

ddx_rack = (Twz1 + Twz2 - ceq * dx_rack * H + Tfri1 + Tfri2 + T_assist)/(Meq * H);