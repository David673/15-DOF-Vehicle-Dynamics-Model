function [delta1, delta2] = KAckermann(x_rack)
% 转向齿条位移为x，假设转向齿条右移为【正】,转向齿条右移时轮胎也右转
% 输入为m，输出为rad
x = x_rack * 1000;
delta1 = (7e-06*x^3 - 0.001 * x^2 + 0.4408 * x)*pi/180;
delta2 = (7e-06*x^3 + 0.001 * x^2 + 0.4408 * x)*pi/180;