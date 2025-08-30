function [AngleR,V] = RelativeAirVelocity(Vcar,Vwind,Cga)
%Vcar,Vwind�ֱ�Ϊȫ��ϵ�³����ٶ��������������λ��m/s��
%���AngleR��rad����Ϊ��Է������г�����ļн�
%���V��m/s����Ϊ��Է��ٵĴ�С
Vrg = Vcar - Vwind;
Vra = Cga\Vrg;
[AngleR,V] = cart2pol(Vra(1),Vra(2));