function [delta1, delta2] = KAckermann(x_rack)
% ת�����λ��Ϊx������ת���������Ϊ������,ת���������ʱ��̥Ҳ��ת
% ����Ϊm�����Ϊrad
x = x_rack * 1000;
delta1 = (7e-06*x^3 - 0.001 * x^2 + 0.4408 * x)*pi/180;
delta2 = (7e-06*x^3 + 0.001 * x^2 + 0.4408 * x)*pi/180;