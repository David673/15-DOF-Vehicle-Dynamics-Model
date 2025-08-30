function [Fx,Fy,Fz,Mx,My,Mz,X,Y,Z] = Aerodynamics(V,Angle)
%% 参数
% 输入为车辆与空气之间的相对速度大小V，单位【m/s】;行车方向与相对风速方向之间的夹角Angle，单位【rad】
% 输出为力【N】，力矩【Nm】，空气阻力参考点X轴坐标，单位【m】
X = 1455/1000;%【m】
Y = 0;%【m】
Z = 0;%【m】
A = 2.2;%【m2】
L = 2910/1000;%【m】一般取轴距
D = 1.206;%【kg/m3】
%% 计算
CFx = 0.0805 * abs(Angle)^3 - 0.4088 * abs(Angle)^2 + 0.2515 * abs(Angle) + 0.2938;
CFy = 0.5 * sin(Angle);
CFz = -0.0392 * abs(Angle)^3 + 0.1597 * abs(Angle)^2 - 0.0455 * abs(Angle) + 0.1811;

CMx = 0.3 * sin(Angle);
CMy = 0.0457 * abs(Angle)^3 - 0.2353 * abs(Angle)^2 + 0.1606 * abs(Angle) + 0.1465;
CMz =  -0.05 * sin(Angle);


Q = D * V * V / 2;
Fx = -CFx * A * Q;
Fy = -CFy * A * Q;
Fz = -CFz * A * Q;
Mx = -CMx * A * L * Q;
My = CMy * A * L * Q;
Mz = CMz * A * L * Q;


