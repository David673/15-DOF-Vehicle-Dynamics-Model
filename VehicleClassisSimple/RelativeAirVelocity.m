function [AngleR,V] = RelativeAirVelocity(Vcar,Vwind,Cga)
%Vcar,Vwind分别为全局系下车辆速度与风速向量，单位【m/s】
%输出AngleR【rad】，为相对风速与行车方向的夹角
%输出V【m/s】，为相对风速的大小
Vrg = Vcar - Vwind;
Vra = Cga\Vrg;
[AngleR,V] = cart2pol(Vra(1),Vra(2));