clear variables;
%% 参数初始化 initializing for parameters

% a（m）：质心到前轴的距离
% b（m）：质心到后轴的距离
% cf（m）：前轮距
% cr（m）：后轮距
% df（m）：前弹簧间距
% dr（m）：后弹簧间距
% zf（m）：整备状态下质心与前悬轮心的垂直方向间距
% zr（m）：整备状态下质心与后悬轮心的垂直方向间距
a = 1.015;
b = 2.910-a;
cf = 1.675;
cr = 1.675;
dsf = 1.575;
dsr = 1.575;
zf = 0.115;
zr = 0.115;
% 
% CA1（rad）：左前轮主销后倾角
% IA1（rad）：左前轮主销内倾角
% LO1（m）：左前轮lateral offset @center,暂不考虑
% XC1（m）：左前轮x coordinate @center，暂不考虑
% psio1（rad）：左前轮初始前束角
% thto1（rad）：左前轮初始外倾角
CA1 = 3*pi/180;  CA2 = 3*pi/180;  CA3 = 0;  CA4 = 0;
IA1 = 13.5*pi/180;  IA2 = 13.5*pi/180;  IA3 = 0;  IA4 = 0;
psio1 = 0; psio2 = 0; psio3 = 0; psio4 = 0;
thto1 = 0;thto2 = 0;thto3 = 0;thto4 = 0;

%% ---调参数 parameter adjust
% Ktz1（N/m）：左前轮轮胎刚度
% Ktx1（_）：左前轮无量纲纵向刚度
% Kty1（/rad）：左前轮侧偏刚度
% RllRes1（m）：左前轮滚动阻力系数
% width1（m）：左前轮拖距
ScaleKtx1= 1*1;
%%%%%%%%%%%
ScaleKty= 1*1;%%%%【】【】【KEY】【】【】【】【】【
ScaleWidth = 1*1;
ScaleFri = 1*1;%100
%%%%%%%%%%%

Ktz1 = 240000;Ktz2 = 240000;Ktz3 = 240000;Ktz4 = 240000;
Ktx1 = ScaleKtx1*25;Ktx2 = ScaleKtx1*25;Ktx3 = ScaleKtx1*25;Ktx4 = ScaleKtx1*25;
Kty1 = ScaleKty*15;Kty2 = ScaleKty*15;Kty3 = ScaleKty*15;Kty4 = ScaleKty*15;
RllRes1 = ScaleFri*0.00001;RllRes2 = ScaleFri*0.00001;RllRes3 = ScaleFri*0.00001;RllRes4 = ScaleFri*0.00001;
width1 = ScaleWidth*0.032;width2 = ScaleWidth*0.032;width3 = ScaleWidth*0.032;width4 = ScaleWidth*0.032;

% Ixx,Iyy,Izz,Ixy,Ixz,Iyz（kg*m^2）：车辆惯量与惯性积
% Ix,Iy,Iz,Ixym,Ixzm,Iyzm（kg*m^2）：车身惯量与惯性积
% m（kg）：簧上质量
Ixx = 536.6;
Iyy = 1536.7;
Izz = 1536.7;
Ixy = 0;
Ixz = 0;%-110
Iyz = 0;
m = 1270;

% 
% Is（kg*m^2）：转向系统等效转动惯量，暂不考虑
% 
% g（m/s^2）：重力加速度
g = 9.8;

% 
% Rf1（m）：左前轮自由半径
% k1（N/m）：左前弹簧刚度
% c1（N/（m/s）^-1）：左前减震阻尼
% R1：左前弹簧压缩量/左前轮心行程,没有用上
% I1（kg*m^2）：左前轮转动惯量（绕转轴）
% m1（kg）：左前簧下质量
ScaleK = 1;
ScaleC = -1;%%%%%%%【关键】【关键】【关键】【关键】【关键】【关键】

Rf1 = 0.316;Rf2 = 0.316;Rf3 = 0.316;Rf4 = 0.316;
k1 = ScaleK*27000;k2 = ScaleK*27000;k3 = ScaleK*27000;k4 = ScaleK*27000;
c1 = 3577*ScaleC;c2 = 3577*ScaleC;c3 = 3577*ScaleC;c4 = 3577*ScaleC;
I1 = 0.9;I2 = 0.9;I3 = 0.9;I4 = 0.9;
m1 = 71;m2 = 71;m3 = 71;m4 = 71;
preload1 = m*g*b/(2*(a+b));preload2 = preload1;
preload3 = m*g*a/(2*(a+b));preload4 = preload3;

% 
% Hsz1（m）：质心到左前悬架作用中心的垂直距离
Hsz1 = 0.115;Hsz2 = 0.115;Hsz3 = 0.115;Hsz4 = 0.115;

%轮心在车身系下的横纵坐标(m)初值
xwc1 = a;
ywc1 = -cf/2;

xwc2 = a;
ywc2 = cf/2;

xwc3 = -b;
ywc3 = -cr/2;

xwc4 = -b;
ywc4 = cr/2;




%% ---时间和步长 basic simulation parameters 
TimeS = 15;
step = 0.001;
Time = 0:step:TimeS;
nstep = length(Time);

%% ---转向、驱动、制动、路谱 
% δ （rad）：前轮转角
% Td1（Nm）：左前轮驱动力矩，Td2（Nm）：右前轮驱动力矩% Td3（Nm）：左后轮驱动力矩% Td4（Nm）：右后轮驱动力矩
% Tb1（Nm）：左前轮制动力矩，Tb2（Nm）：右前轮制动力矩% Tb3（Nm）：左后轮制动力矩% Tb4（Nm）：右后轮制动力矩
% Zd1（m）：左前轮路面输入，Zd2（m）：右前轮路面输入% Zd3（m）：左后轮路面输入% Zd4（m）：右后轮路面输入
%TSWin(Nm):方向盘力矩输入

% delta = (180*pi/180 * sin(2*pi/10 * Time))';%%%%sine input
% delta =  0 * pi/180+zeros(nstep,1);%%%%step input
delta = 0*pi/80 + zeros(nstep,1);
% for j = 1:nstep
%     if j<(nstep/2)
%         delta(j) = 0;
%     else
%         if j<(nstep/2 + 0.2/step)
%         delta(j) = 0.9 * (j - nstep/2) *pi/180;
%         else 
%             delta(j) = 180*pi/180;
%         end
%     end
% end    %step input------delayed

TSWin = 10 + zeros(nstep,1);%转向回正
for j = 1:nstep
    if j>(nstep*3/4)
        TSWin(j) = 0;
    end
end

% TSWin = 6.5/TimeS * Time;%斜坡输入

Td1 = 500+zeros(nstep,1);
Tb1 = 0+zeros(nstep,1);
Zd1 = zeros(nstep,1);  %平地
% Zd1 = 0.01 * sin(2*pi/1 * Time); %正弦输入

Td2 = 500+zeros(nstep,1);
Tb2 = 0+zeros(nstep,1);
Zd2 = zeros(nstep,1); %平地
% Zd2 = 0.01 * sin(2*pi/1 * Time); %正弦输入

Td3 = 0+zeros(nstep,1);
Tb3 = 0+zeros(nstep,1);
Zd3 = zeros(nstep,1);
% Zd3 = 0.01 * sin(2*pi/1 * Time);

Td4 = 0+zeros(nstep,1);
Tb4 = 0+zeros(nstep,1);
Zd4 = zeros(nstep,1);
% Zd4 = 0.01 * sin(2*pi/1 * Time);

for k = 1:nstep
    if k>(nstep*3/4)
        Td1(k) = 0;
        Td2(k) = 0;
    end
end

%% 状态空间初始化 initializing for variables

% x（m）：全局坐标系下质心x坐标
% y（m）：全局坐标系下质心y坐标
% z（m）：全局坐标系下质心z坐标
% yaw（rad）：全局坐标系下车身横摆角
% rll（rad）：全局坐标系下车身侧倾角
% pth（rad）：全局坐标系下车身俯仰角
x = zeros(nstep,1);x(1) = 0;
y = zeros(nstep,1);y(1) = 0;
z = zeros(nstep,1);z(1) = -0.414;
yaw = zeros(nstep,1);yaw(1) = 0;
rll = zeros(nstep,1);rll(1) = 0;
pth = zeros(nstep,1);pth(1) = 0;

%% ---调车速 vehicle speed adjust
dx = zeros(nstep,1);dx(1) = 20; %0.5
dy = zeros(nstep,1);dy(1) = 0;
dz = zeros(nstep,1);dz(1) = 0;
dyaw = zeros(nstep,1);dyaw(1) = 0;
drll = zeros(nstep,1);drll(1) = 0;
dpth = zeros(nstep,1);dpth(1) = 0;

% 
% s1（m）：左前轮心位移，相对车身系，向下为正
% s2（m）：右前轮心位移，相对车身
% s3（m）：左后轮心位移，相对车身
% s4（m）：右后轮心位移，相对车身
s1 = zeros(nstep,1);s1(1) = 0;
s2 = zeros(nstep,1);s2(1) = 0;
s3 = zeros(nstep,1);s3(1) = 0;
s4 = zeros(nstep,1);s4(1) = 0;

% 
% s1’(m/s)
% s2’
% s3’
% s4’
ds1 = zeros(nstep,1);ds1(1) = 0;
ds2 = zeros(nstep,1);ds2(1) = 0;
ds3 = zeros(nstep,1);ds3(1) = 0;
ds4 = zeros(nstep,1);ds4(1) = 0;
% 
% s1’’(m/s^2)
% s2’’
% s3’’
% s4’’
dds1 = zeros(nstep,1);dds1(1) = 0;
dds2 = zeros(nstep,1);dds2(1) = 0;
dds3 = zeros(nstep,1);dds3(1) = 0;
dds4 = zeros(nstep,1);dds4(1) = 0;
% 
% n1（rad）：左前轮转动角
% n2（rad）：右前轮转动角
% n3（rad）：左后轮转动角
% n4（rad）：右后轮转动角
n1 = zeros(nstep,1);n1(1) = 0;
n2 = zeros(nstep,1);n2(1) = 0;
n3 = zeros(nstep,1);n3(1) = 0;
n4 = zeros(nstep,1);n4(1) = 0;
% 
% n1’(rad/s)
% n2’
% n3’
% n4’
dn1 = zeros(nstep,1);dn1(1) = dx(1)/(Rf1-(m*g*b/(2*(a+b)*Ktz1)));
dn2 = zeros(nstep,1);dn2(1) = dx(1)/(Rf2-(m*g*b/(2*(a+b)*Ktz2)));
dn3 = zeros(nstep,1);dn3(1) = dx(1)/(Rf3-(m*g*a/(2*(a+b)*Ktz3)));
dn4 = zeros(nstep,1);dn4(1) = dx(1)/(Rf4-(m*g*a/(2*(a+b)*Ktz4)));
% 
% n1’’(rad/s^2)
% n2’’
% n3’’
% n4’’
ddn1 = zeros(nstep,1);ddn1(1) = 0;
ddn2 = zeros(nstep,1);ddn2(1) = 0;
ddn3 = zeros(nstep,1);ddn3(1) = 0;
ddn4 = zeros(nstep,1);ddn4(1) = 0;



%(m/s)
u = zeros(nstep,1);u(1) = dx(1);
v = zeros(nstep,1);v(1) = 0;
w = zeros(nstep,1);w(1) = 0;
p = zeros(nstep,1);p(1) = 0;
q = zeros(nstep,1);q(1) = 0;
r = zeros(nstep,1);r(1) = 0;

%(m/s^2)
du = zeros(nstep,1);du(1) = 0;
dv = zeros(nstep,1);dv(1) = 0;
dw = zeros(nstep,1);dw(1) = 0;
dp = zeros(nstep,1);dp(1) = 0;
dq = zeros(nstep,1);dq(1) = 0;
dr = zeros(nstep,1);dr(1) = 0;

%转向齿条位移、速度、加速度【m】【m/s】【m/s2】
x_rack= zeros(nstep,1);
dx_rack = zeros(nstep,1);
ddx_rack = zeros(nstep,1);

%作为状态变量的方向盘转角
deltaV = zeros(nstep,1);
ddeltaV = zeros(nstep,1);
dddeltaV = zeros(nstep,1);

%% 中间变量、仿真产出值初始化 initializing for middle variables
%(N)
fs1 = zeros(nstep,1);fs1(1) = 0;
fs2 = zeros(nstep,1);fs2(1) = 0;
fs3 = zeros(nstep,1);fs3(1) = 0;
fs4 = zeros(nstep,1);fs4(1) = 0;

%(m)
Rd1 = Rf1 + zeros(nstep,1);
Rd2 = Rf2 + zeros(nstep,1);
Rd3 = Rf3 + zeros(nstep,1);
Rd4 = Rf4 + zeros(nstep,1);

%(N)
fwz1 = zeros(nstep,1);
fwz2 = zeros(nstep,1);
fwz3 = zeros(nstep,1);
fwz4 = zeros(nstep,1);

%(Nm)
Twy1 = zeros(nstep,1);
Twy2 = zeros(nstep,1);
Twy3 = zeros(nstep,1);
Twy4 = zeros(nstep,1);

%(_)
kappa1 = zeros(nstep,1);
kappa2 = zeros(nstep,1);
kappa3 = zeros(nstep,1);
kappa4 = zeros(nstep,1);

%(rad)
alpha1 = zeros(nstep,1);
alpha2 = zeros(nstep,1);
alpha3 = zeros(nstep,1);
alpha4 = zeros(nstep,1);

%(N)
fwx1 = zeros(nstep,1);
fwx2 = zeros(nstep,1);
fwx3 = zeros(nstep,1);
fwx4 = zeros(nstep,1);

%(N)
fwy1 = zeros(nstep,1);
fwy2 = zeros(nstep,1);
fwy3 = zeros(nstep,1);
fwy4 = zeros(nstep,1);

%(Nm)
Twz1 = zeros(nstep,1);
Twz2 = zeros(nstep,1);
Twz3 = zeros(nstep,1);
Twz4 = zeros(nstep,1);

%(N)
fx1 = zeros(nstep,1);
fx2 = zeros(nstep,1);
fx3 = zeros(nstep,1);
fx4 = zeros(nstep,1);

%(N)
fy1 = zeros(nstep,1);
fy2 = zeros(nstep,1);
fy3 = zeros(nstep,1);
fy4 = zeros(nstep,1);

%(N)
fz1 = zeros(nstep,1);
fz2 = zeros(nstep,1);
fz3 = zeros(nstep,1);
fz4 = zeros(nstep,1);

%K特性参数输出
KXmove1 = zeros(nstep,1);
KYmove1 = zeros(nstep,1);

KXmove2 = zeros(nstep,1);
KYmove2 = zeros(nstep,1);

KXmove3 = zeros(nstep,1);
KYmove3 = zeros(nstep,1);

KXmove4 = zeros(nstep,1);
KYmove4 = zeros(nstep,1);

KToe1 = zeros(nstep,1);
KToe2 = zeros(nstep,1);
KToe3 = zeros(nstep,1);
KToe4 = zeros(nstep,1);

KDive1 = zeros(nstep,1);
KDive2 = zeros(nstep,1);
KDive3 = zeros(nstep,1);
KDive4 = zeros(nstep,1);

KCamber1 = zeros(nstep,1);
KCamber2 = zeros(nstep,1);
KCamber3 = zeros(nstep,1);
KCamber4 = zeros(nstep,1);

%C特性参数初始化
CToe1_fx = zeros(nstep,1);
CToe2_fx = zeros(nstep,1);
CToe3_fx = zeros(nstep,1);
CToe4_fx = zeros(nstep,1);

CSteer1_fy = zeros(nstep,1);
CSteer2_fy = zeros(nstep,1);
CSteer3_fy = zeros(nstep,1);
CSteer4_fy = zeros(nstep,1);

CSteer1_Mz = zeros(nstep,1);
CSteer2_Mz = zeros(nstep,1);
CSteer3_Mz = zeros(nstep,1);
CSteer4_Mz = zeros(nstep,1);

CCamber1_Fx = zeros(nstep,1);
CCamber2_Fx = zeros(nstep,1);
CCamber3_Fx = zeros(nstep,1);
CCamber4_Fx = zeros(nstep,1);

CInc1_Fy = zeros(nstep,1);
CInc2_Fy = zeros(nstep,1);
CInc3_Fy = zeros(nstep,1);
CInc4_Fy = zeros(nstep,1);

CInc1_Mz = zeros(nstep,1);
CInc2_Mz = zeros(nstep,1);
CInc3_Mz = zeros(nstep,1);
CInc4_Mz = zeros(nstep,1);

CXmove1 = zeros(nstep,1);
CXmove2 = zeros(nstep,1);
CXmove3 = zeros(nstep,1);
CXmove4 = zeros(nstep,1);

CYmove1 = zeros(nstep,1);
CYmove2 = zeros(nstep,1);
CYmove3 = zeros(nstep,1);
CYmove4 = zeros(nstep,1);

CDive1 = zeros(nstep,1);
CDive2 = zeros(nstep,1);
CDive3 = zeros(nstep,1);
CDive4 = zeros(nstep,1);

delta1 = zeros(nstep,1);
delta2 = zeros(nstep,1);
T_assist = zeros(nstep,1);

AngleAirR= zeros(nstep,1);
VairR = zeros(nstep,1);
FxAir = zeros(nstep,1);
FyAir = zeros(nstep,1);

%% 构建参数集合
ParameterIn = zeros(100,1);
%--------------------
ParameterIn(1) = a;
ParameterIn(2) = b;
ParameterIn(3) = cf;
ParameterIn(4) = cr;
ParameterIn(5) = dsf;
ParameterIn(6) = dsr;
ParameterIn(7) = zf;
ParameterIn(8) = zr;
%---------------------
ParameterIn(9) = CA1;  ParameterIn(10) = CA2;  ParameterIn(11) = CA3;  ParameterIn(12) = CA4;
ParameterIn(13) = IA1;  ParameterIn(14) = IA2;  ParameterIn(15) = IA3;  ParameterIn(16) = IA4;
ParameterIn(17) = psio1; ParameterIn(18) = psio2; ParameterIn(19) = psio3; ParameterIn(20) = psio4;
ParameterIn(21) = thto1; ParameterIn(22) = thto2; ParameterIn(23) = thto3; ParameterIn(24) = thto4;
%----------------------------------------------------------------------------------------------------
ParameterIn(25) = Ktz1; ParameterIn(26) = Ktz2; ParameterIn(27) = Ktz3; ParameterIn(28) = Ktz4;
ParameterIn(29) = Ktx1; ParameterIn(30) = Ktx2; ParameterIn(31) = Ktx3; ParameterIn(32) = Ktx4;
ParameterIn(33) = Kty1; ParameterIn(34) = Kty2; ParameterIn(35) = Kty3; ParameterIn(36) = Kty4;
ParameterIn(37) = RllRes1; ParameterIn(38) = RllRes2; ParameterIn(39) = RllRes3; ParameterIn(40) = RllRes4;
ParameterIn(41) = width1 ; ParameterIn(42) = width2; ParameterIn(43) = width3; ParameterIn(44) = width4;
%----------------------------------------------------------------------------------------------------------
ParameterIn(45) = Ixx;
ParameterIn(46) = Iyy;
ParameterIn(47) = Izz;
ParameterIn(48) = Ixy;
ParameterIn(49) = Ixz;%-110
ParameterIn(50) = Iyz;
ParameterIn(51) = m;
%-------------------------
ParameterIn(52) = Rf1; ParameterIn(53) = Rf2; ParameterIn(54) = Rf3; ParameterIn(55) = Rf4;
ParameterIn(56) = k1; ParameterIn(57) = k2; ParameterIn(58) = k3; ParameterIn(59) = k4;
ParameterIn(60) = c1; ParameterIn(61) = c2; ParameterIn(62) = c3; ParameterIn(63) = c4;
ParameterIn(64) = I1; ParameterIn(65) = I2; ParameterIn(66) = I3; ParameterIn(67) = I4;
ParameterIn(68) = m1; ParameterIn(69) = m2; ParameterIn(70) = m3; ParameterIn(71) = m4;
ParameterIn(72) = preload1; ParameterIn(73) = preload2;
ParameterIn(74) = preload3; ParameterIn(75) = preload4;
%--------------------------------------------------------------------------------------------
ParameterIn(76) = Hsz1; ParameterIn(77) = Hsz2; ParameterIn(78) = Hsz3; ParameterIn(79) = Hsz4;

ParameterIn(80) = xwc1;
ParameterIn(81) = ywc1;

ParameterIn(82) = xwc2;
ParameterIn(83) = ywc2;

ParameterIn(84) = xwc3;
ParameterIn(85) = ywc3;

ParameterIn(86) = xwc4;
ParameterIn(87) = ywc4;

ParameterIn(88) = g;


%% 构建仿真参数集合
SimulationIn = [step, nstep];
%% 初始化状态量输入集合
VariableIn = zeros(32,1);
VariableIn_m1 = zeros(32,1);

%% ---开始迭代-改进欧拉法 integration Euler modified
for i=1:(nstep-1)
    %% ------构建控制输入集合
     
    ControlIn = [delta(i);
        Td1(i);Td2(i);Td3(i);Td4(i);
        Tb1(i);Tb2(i);Tb3(i);Tb4(i);
        Zd1(i);Zd2(i);Zd3(i);Zd4(i);TSWin(i)];
    
    ControlIn_p1 = [delta(i+1);
        Td1(i+1);Td2(i+1);Td3(i+1);Td4(i+1);
        Tb1(i+1);Tb2(i+1);Tb3(i+1);Tb4(i+1);
        Zd1(i+1);Zd2(i+1);Zd3(i+1);Zd4(i+1);TSWin(i+1)];
    
    %% ------构建状态量输入集合
    %--------------------------
    VariableIn(1) = x(i);
    VariableIn(2) = y(i);
    VariableIn(3) = z(i);
    VariableIn(4) = yaw(i);
    VariableIn(5) = rll(i);
    VariableIn(6) = pth(i);
    %--------------------------
    VariableIn(7) = s1(i);    VariableIn(8) = s2(i);    VariableIn(9) = s3(i);    VariableIn(10) = s4(i);
    VariableIn(11) = ds1(i);   VariableIn(12) = ds2(i);   VariableIn(13) = ds3(i);   VariableIn(14) = ds4(i);
    %-----------------------------
    VariableIn(15) = n1(i);    VariableIn(16) = n2(i);    VariableIn(17) = n3(i);    VariableIn(18) = n4(i);
    VariableIn(19) = dn1(i);   VariableIn(20) = dn2(i);   VariableIn(21) = dn3(i);   VariableIn(22) = dn4(i);
    %------------------------
    VariableIn(23) = u(i);
    VariableIn(24) = v(i);
    VariableIn(25) = w(i);
    VariableIn(26) = p(i);
    VariableIn(27) = q(i);
    VariableIn(28) = r(i);
    %-----------------------
    VariableIn(29) = x_rack(i);
    VariableIn(30) = dx_rack(i);
    VariableIn(31) = deltaV(i);
    VariableIn(32) = ddeltaV(i);
    
    %% ------构建其他需要输入的参数
    OthersIn = zeros(25,1);
    OthersIn(1) = i;
    if i>1
        OthersIn(2) = kappa1(i-1);
        OthersIn(3) = kappa2(i-1);
        OthersIn(4) = kappa3(i-1);
        OthersIn(5) = kappa4(i-1);
        
        OthersIn(6) = fx1(i-1);
        OthersIn(7) = fx2(i-1);
        OthersIn(8) = fx3(i-1);
        OthersIn(9) = fx4(i-1);
        
        OthersIn(10) = fy1(i-1);
        OthersIn(11) = fy2(i-1);
        OthersIn(12) = fy3(i-1);
        OthersIn(13) = fy4(i-1);
        
        OthersIn(14) = Twz1(i-1);
        OthersIn(15) = Twz2(i-1);
        OthersIn(16) = Twz3(i-1);
        OthersIn(17) = Twz4(i-1);
    end
    
    
    %% ----------改进欧拉法
    [Var_p, dVar_p, MiddleOut_p] = DynamicModel_15DOF(ControlIn, ParameterIn, SimulationIn, VariableIn, OthersIn);
    
    [Var_c1, dVar_c, MiddleOut_c] = DynamicModel_15DOF(ControlIn_p1, ParameterIn, SimulationIn, Var_p, OthersIn);
    Var_cp1 = VariableIn + step * dVar_c;
    VarIn_plus1 = (Var_p + Var_cp1)/2;
    dVarIn = (dVar_p + dVar_c)/2;
    MidOut = (MiddleOut_p + MiddleOut_c)/2;
    
    %% ----------欧拉法
%     [VarIn_plus1,dVarIn, MidOut] = DynamicModel_15DOF(ControlIn, ParameterIn, SimulationIn, VariableIn, OthersIn);
    

    %% ----------Adams-Bashforth 两步法
%     if i>1
%         ControlIn_m1 = [delta(i-1);
%         Td1(i-1);Td2(i-1);Td3(i-1);Td4(i-1);
%         Tb1(i-1);Tb2(i-1);Tb3(i-1);Tb4(i-1);
%         Zd1(i-1);Zd2(i-1);Zd3(i-1);Zd4(i-1);];
%         %---------------------
%         VariableIn_m1(1) = x(i-1);
%         VariableIn_m1(2) = y(i-1);
%         VariableIn_m1(3) = z(i-1);
%         VariableIn_m1(4) = yaw(i-1);
%         VariableIn_m1(5) = rll(i-1);
%         VariableIn_m1(6) = pth(i-1);
%         %--------------------------
%         VariableIn_m1(7) = s1(i-1);    VariableIn_m1(8) = s2(i-1);    VariableIn_m1(9) = s3(i-1);    VariableIn_m1(10) = s4(i-1);
%         VariableIn_m1(11) = ds1(i-1);   VariableIn_m1(12) = ds2(i-1);   VariableIn_m1(13) = ds3(i-1);   VariableIn_m1(14) = ds4(i-1);
%         %-----------------------------
%         VariableIn_m1(15) = n1(i-1);    VariableIn_m1(16) = n2(i-1);    VariableIn_m1(17) = n3(i-1);    VariableIn_m1(18) = n4(i-1);
%         VariableIn_m1(19) = dn1(i-1);   VariableIn_m1(20) = dn2(i-1);   VariableIn_m1(21) = dn3(i-1);   VariableIn_m1(22) = dn4(i-1);
%         %------------------------
%         VariableIn_m1(23) = u(i-1);
%         VariableIn_m1(24) = v(i-1);
%         VariableIn_m1(25) = w(i-1);
%         VariableIn_m1(26) = p(i-1);
%         VariableIn_m1(27) = q(i-1);
%         VariableIn_m1(28) = r(i-1);
%     %-----------------------
%         [Var_p, dVar_p, MiddleOut_p] = DynamicModel_15DOF(ControlIn, ParameterIn, SimulationIn, VariableIn, OthersIn);
%         [Var_m, dVar_m, MiddleOut_m] = DynamicModel_15DOF(ControlIn_m1, ParameterIn, SimulationIn, VariableIn_m1, OthersIn);
%         VarIn_plus1 = VariableIn + step/2 * (3*dVar_p - dVar_m);
%         
%         dVarIn = (3*dVar_p - dVar_m)/2;
%         MidOut = (3*MiddleOut_p - MiddleOut_m)/2;
%         
%     else
%         if (i==1)
%             [VarIn_plus1,dVarIn, MidOut] = DynamicModel_15DOF(ControlIn, ParameterIn, SimulationIn, VariableIn, OthersIn);
%         end
%         
%     end
%     
    
    %% ------状态变量输出
    %---------------------
    x(i+1) = VarIn_plus1(1);
    y(i+1) = VarIn_plus1(2);
    z(i+1) = VarIn_plus1(3);
    yaw(i+1) = VarIn_plus1(4);
    rll(i+1) = VarIn_plus1(5);
    pth(i+1) = VarIn_plus1(6);
    %--------------------------
    s1(i+1) = VarIn_plus1(7); s2(i+1) = VarIn_plus1(8);  s3(i+1) = VarIn_plus1(9);s4(i+1) = VarIn_plus1(10);
    ds1(i+1) = VarIn_plus1(11); ds2(i+1) = VarIn_plus1(12); ds3(i+1) = VarIn_plus1(13);  ds4(i+1) = VarIn_plus1(14);
    %-----------------------------
    n1(i+1) = VarIn_plus1(15);  n2(i+1) = VarIn_plus1(16); n3(i+1) = VarIn_plus1(17);  n4(i+1) = VarIn_plus1(18);
    dn1(i+1) = VarIn_plus1(19); dn2(i+1) = VarIn_plus1(20); dn3(i+1) = VarIn_plus1(21); dn4(i+1) = VarIn_plus1(22);
    %------------------------
    u(i+1) = VarIn_plus1(23);
    v(i+1) = VarIn_plus1(24);
    w(i+1) = VarIn_plus1(25);
    p(i+1) = VarIn_plus1(26);
    q(i+1) = VarIn_plus1(27);
    r(i+1) = VarIn_plus1(28);
    %------------------------
    x_rack(i+1) = VarIn_plus1(29);
    dx_rack(i+1) = VarIn_plus1(30);
    
    deltaV(i+1) = VarIn_plus1(31);
    ddeltaV(i+1) = VarIn_plus1(32);
    %% ------状态变量导数输出
    dx(i) = dVarIn(1);
    dy(i) = dVarIn(2);
    dz(i) = dVarIn(3);
    
    dyaw(i) = dVarIn(4);
    drll(i) = dVarIn(5);
    dpth(i) = dVarIn(6);
    

    ds1(i) = dVarIn(7); %【】【】【】【】【】
    ds2(i) = dVarIn(8); %【】【】【】【】【】
    ds3(i) = dVarIn(9); %【】【】【】【】【】
    ds4(i) = dVarIn(10); %【】【】【】【】【】

    dds1(i) = dVarIn(11);
    dds2(i) = dVarIn(12);
    dds3(i) = dVarIn(13);
    dds4(i) = dVarIn(14);

    dn1(i) = dVarIn(15); %【】【】【】【】【】
    dn2(i) = dVarIn(16); %【】【】【】【】【】
    dn3(i) = dVarIn(17); %【】【】【】【】【】
    dn4(i) = dVarIn(18); %【】【】【】【】【】

    ddn1(i) = dVarIn(19);
    ddn2(i) = dVarIn(20);
    ddn3(i) = dVarIn(21);
    ddn4(i) = dVarIn(22);

    du(i) = dVarIn(23);
    dv(i) = dVarIn(24);
    dw(i) = dVarIn(25);
    dp(i) = dVarIn(26);
    dq(i) = dVarIn(27);
    dr(i) = dVarIn(28);
    
    dx_rack(i) = dVarIn(29);
    ddx_rack(i) = dVarIn(30);
    
    ddeltaV(i) = dVarIn(31);
    dddeltaV(i) = dVarIn(32);
    
    
    %% ------中间变量、结果输出
    fs1(i) = MidOut(1) ;
    fs2(i) = MidOut(2) ;
    fs3(i) = MidOut(3) ;
    fs4(i) = MidOut(4) ;

    %(m)
    Rd1(i) = MidOut(5) ;
    Rd2(i) = MidOut(6) ;
    Rd3(i) = MidOut(7) ;
    Rd4(i) = MidOut(8) ;

    %(N)
    fwz1(i) = MidOut(9) ;
    fwz2(i) = MidOut(10) ;
    fwz3(i) = MidOut(11) ;
    fwz4(i) = MidOut(12) ;

    %(Nm)
    Twy1(i) = MidOut(13) ;
    Twy2(i) = MidOut(14) ;
    Twy3(i) = MidOut(15) ;
    Twy4(i) = MidOut(16) ;

    %(_)
    kappa1(i) = MidOut(17) ;
    kappa2(i) = MidOut(18) ;
    kappa3(i) = MidOut(19) ;
    kappa4(i) = MidOut(20) ;

    %(rad)
    alpha1(i) = MidOut(21) ;
    alpha2(i) = MidOut(22) ;
    alpha3(i) = MidOut(23) ;
    alpha4(i) = MidOut(24) ;

    %(N)
    fwx1(i) = MidOut(25) ;
    fwx2(i) = MidOut(26) ;
    fwx3(i) = MidOut(27) ;
    fwx4(i) = MidOut(28) ;

    %(N)
    fwy1(i) = MidOut(29) ;
    fwy2(i) = MidOut(30) ;
    fwy3(i) = MidOut(31) ;
    fwy4(i) = MidOut(32) ;

    %(Nm)
    Twz1(i) = MidOut(33) ;
    Twz2(i) = MidOut(34) ;
    Twz3(i) = MidOut(35) ;
    Twz4(i) = MidOut(36) ;

    %(N)
    fx1(i) = MidOut(37) ;
    fx2(i) = MidOut(38) ;
    fx3(i) = MidOut(39) ;
    fx4(i) = MidOut(40) ;

    %(N)
    fy1(i) = MidOut(41) ;
    fy2(i) = MidOut(42) ;
    fy3(i) = MidOut(43) ;
    fy4(i) = MidOut(44) ;

    %(N)
    fz1(i) = MidOut(45) ;
    fz2(i) = MidOut(46) ;
    fz3(i) = MidOut(47) ;
    fz4(i) = MidOut(48) ;
    
    KXmove1(i) = MidOut(49);
    KYmove1(i)=MidOut(50);

    KXmove2(i)=MidOut(51);
    KYmove2(i)=MidOut(52);

    KXmove3(i)=MidOut(53);
    KYmove3(i)=MidOut(54);

    KXmove4(i)=MidOut(55);
    KYmove4(i)=MidOut(56);

    KToe1(i)=MidOut(57);
    KToe2(i)=MidOut(58);
    KToe3(i)=MidOut(59);
    KToe4(i)=MidOut(60);

    KDive1(i)=MidOut(61);
    KDive2(i)=MidOut(62);
    KDive3(i)=MidOut(63);
    KDive4(i)=MidOut(64);

    KCamber1(i)=MidOut(65);
    KCamber2(i)=MidOut(66);
    KCamber3(i)=MidOut(67);
    KCamber4(i)=MidOut(68);
    
    CToe1_fx(i)=MidOut(69);
    CToe2_fx(i)=MidOut(70);
    CToe3_fx(i)=MidOut(71);
    CToe4_fx(i)=MidOut(72);
    
    CSteer1_fy(i) = MidOut(73);
    CSteer2_fy(i) = MidOut(74);
    CSteer3_fy(i) = MidOut(75);
    CSteer4_fy(i) = MidOut(76);
    
    
    CSteer1_Mz(i) = MidOut(77);
    CSteer2_Mz(i) = MidOut(78);
    CSteer3_Mz(i) = MidOut(79);
    CSteer4_Mz(i) = MidOut(80);
    
    CCamber1_Fx(i) = MidOut(81);
    CCamber2_Fx(i) = MidOut(82);
    CCamber3_Fx(i) = MidOut(83);
    CCamber4_Fx(i) = MidOut(84);
    
    CInc1_Fy(i) = MidOut(85);
    CInc2_Fy(i) = MidOut(86);
    CInc3_Fy(i) = MidOut(87);
    CInc4_Fy(i) = MidOut(88);
    
    CInc1_Mz(i) = MidOut(89);
    CInc2_Mz(i) = MidOut(90);
    CInc3_Mz(i) = MidOut(91);
    CInc4_Mz(i) = MidOut(92);  
    
    CXmove1(i) = MidOut(93);
    CXmove2(i) = MidOut(94);
    CXmove3(i) = MidOut(95);
    CXmove4(i) = MidOut(96);
    
    CYmove1(i) = MidOut(97);
    CYmove2(i) = MidOut(98);
    CYmove3(i) = MidOut(99);
    CYmove4(i) = MidOut(100);
    
    CDive1(i) = MidOut(101);
    CDive2(i) = MidOut(102);
    CDive3(i) = MidOut(103);
    CDive4(i) = MidOut(104);
    
    delta1(i) = MidOut(105);
    delta2(i) = MidOut(106);
    T_assist(i) = MidOut(107);
    
    AngleAirR(i) = MidOut(108);
    VairR(i) = MidOut(109);
    FxAir(i) = MidOut(110);
    FyAir(i) = MidOut(111);
    
    if(mod(i,round(nstep/10)) ==0)
        disp(i/nstep);
    end
end
%% 后处理 Post Porcessing
figure(1);
subplot(3,4,1);plot(Time,57.3*yaw);title('yaw(deg)');
subplot(3,4,2);plot(Time,57.3*pth);title('pth(deg)');
subplot(3,4,3);plot(Time,57.3*rll);title('rll(deg)');

subplot(3,4,4);plot(Time,x);title('x(m)');
subplot(3,4,5);plot(Time,y);title('y(m)');
subplot(3,4,6);plot(Time,1000*z);title('z(mm)');

subplot(3,4,7);plot(Time,57.3*dyaw);title('dyaw(deg/s)');
subplot(3,4,8);plot(Time,57.3*dpth);title('dpth(deg/s)');
subplot(3,4,9);plot(Time,57.3*drll);title('drll(deg/s)');

subplot(3,4,10);plot(Time,dx);title('dx(m/s)');
subplot(3,4,11);plot(Time,dy);title('dy(m/s)');
subplot(3,4,12);plot(Time,1000*dz);title('dz(mm/s)');

figure(2);
subplot(4,4,1);plot(Time,u);title('u(m/s)');
subplot(4,4,2);plot(Time,v);title('v(m/s)');
subplot(4,4,3);plot(Time,1000*w);title('w(mm/s)');

subplot(4,4,4);plot(Time,57.3*p);title('p(deg/s)');
subplot(4,4,5);plot(Time,57.3*q);title('q(deg/s)');
subplot(4,4,6);plot(Time,57.3*r);title('r(deg/s)');

subplot(4,4,7);plot(Time,1000*ds1);title('ds1(mm/s)');
subplot(4,4,8);plot(Time,1000*ds2);title('ds2(mm/s)');
subplot(4,4,9);plot(Time,1000*ds3);title('ds3(mm/s)');
subplot(4,4,10);plot(Time,1000*ds4);title('ds4(mm/s)');

subplot(4,4,11);plot(Time,57.3*dn1);title('dn1(deg/s)');
subplot(4,4,12);plot(Time,57.3*dn2);title('dn2(deg/s)');
subplot(4,4,13);plot(Time,57.3*dn3);title('dn3(deg/s)');
subplot(4,4,14);plot(Time,57.3*dn4);title('dn4(deg/s)');

subplot(4,4,15);plot(Time,1000*s1);title('s1(mm)');
subplot(4,4,16);plot(Time,1000*s2);title('s2(mm)');

figure(3);
subplot(4,4,1);plot(Time,du);title('du(m/s2)');
subplot(4,4,2);plot(Time,dv);title('dv(m/s2)');
subplot(4,4,3);plot(Time,1000*dw);title('dw(mm/s2)');

subplot(4,4,4);plot(Time,57.3*dp);title('dp(deg/s2)');
subplot(4,4,5);plot(Time,57.3*dq);title('dq(deg/s2)');
subplot(4,4,6);plot(Time,57.3*dr);title('dr(deg/s2)');

subplot(4,4,7);plot(Time,1000*dds1);title('dds1(mm/s2)');
subplot(4,4,8);plot(Time,1000*dds2);title('dds2(mm/s2)');
subplot(4,4,9);plot(Time,1000*dds3);title('dds3(mm/s2)');
subplot(4,4,10);plot(Time,1000*dds4);title('dds4(mm/s2)');

subplot(4,4,11);plot(Time,57.3*ddn1);title('ddn1(deg/s2)');
subplot(4,4,12);plot(Time,57.3*ddn2);title('ddn2(deg/s2)');
subplot(4,4,13);plot(Time,57.3*ddn3);title('ddn3(deg/s2)');
subplot(4,4,14);plot(Time,57.3*ddn4);title('ddn4(deg/s2)');

subplot(4,4,15);plot(Time,1000*s3);title('s3(mm)');
subplot(4,4,16);plot(Time,1000*s4);title('s4(mm)');

figure(4);
subplot(4,4,1);plot(Time,fx1);title('fx1(N)');
subplot(4,4,2);plot(Time,fx2);title('fx2(N)');
subplot(4,4,3);plot(Time,fx3);title('fx3(N)');
subplot(4,4,4);plot(Time,fx4);title('fx4(N)');

subplot(4,4,5);plot(Time,fy1);title('fy1(N)');
subplot(4,4,6);plot(Time,fy2);title('fy2(N)');
subplot(4,4,7);plot(Time,fy3);title('fy3(N)');
subplot(4,4,8);plot(Time,fy4);title('fy4(N)');

subplot(4,4,9);plot(Time,fz1);title('fz1(N)');
subplot(4,4,10);plot(Time,fz2);title('fz2(N)');
subplot(4,4,11);plot(Time,fz3);title('fz3(N)');
subplot(4,4,12);plot(Time,fz4);title('fz4(N)');

subplot(4,4,13);plot(Time,fs1);title('fs1(N)');
subplot(4,4,14);plot(Time,fs2);title('fs2(N)');
subplot(4,4,15);plot(Time,fs3);title('fs3(N)');
subplot(4,4,16);plot(Time,fs4);title('fs4(N)');

figure(5);
subplot(4,4,1);plot(Time,Twy1);title('Twy1滚动阻力(Nm)');
subplot(4,4,2);plot(Time,Twy2);title('Twy2(Nm)');
subplot(4,4,3);plot(Time,Twy3);title('Twy3(Nm)');
subplot(4,4,4);plot(Time,Twy4);title('Twy4(Nm)');

subplot(4,4,5);plot(Time,Twz1);title('Twz1回正力矩(Nm)');
subplot(4,4,6);plot(Time,Twz2);title('Twz2(Nm)');
subplot(4,4,7);plot(Time,Twz3);title('Twz3(Nm)');
subplot(4,4,8);plot(Time,Twz4);title('Twz4(Nm)');

subplot(4,4,9);plot(Time,57.3*alpha1);title('alpha1（deg）');
subplot(4,4,10);plot(Time,57.3*alpha2);title('alpha2（deg）');
subplot(4,4,11);plot(Time,57.3*alpha3);title('alpha3（deg）');
subplot(4,4,12);plot(Time,57.3*alpha4);title('alpha4（deg）');

subplot(4,4,13);plot(Time,kappa1);title('kappa1');
subplot(4,4,14);plot(Time,kappa2);title('kappa2');
subplot(4,4,15);plot(Time,kappa3);title('kappa3');
subplot(4,4,16);plot(Time,kappa4);title('kappa4');

figure(6);
plot(x,y);title('轨迹');axis equal;

figure(7);
subplot(2,2,1);plot(Time,(du - r.*v + q.*w));title('axm(m/s2)');
subplot(2,2,2);plot(Time,(dv - p.*w + r.*u));title('aym(m/s2)');
subplot(2,2,3);plot(Time,(dw - q.*u + p.*v));title('azm(m/s2)');

figure(8);
subplot(5,4,1);plot(Time,1000*KXmove1);title('KXmove1(mm)');
subplot(5,4,2);plot(Time,1000*KXmove2);title('KXmove2(mm)');
subplot(5,4,3);plot(Time,1000*KXmove3);title('KXmove3(mm)');
subplot(5,4,4);plot(Time,1000*KXmove4);title('KXmove4(mm)');

subplot(5,4,5);plot(Time,1000*KYmove1);title('KYmove1(mm)');
subplot(5,4,6);plot(Time,1000*KYmove2);title('KYmove2(mm)');
subplot(5,4,7);plot(Time,1000*KYmove3);title('KYmove3(mm)');
subplot(5,4,8);plot(Time,1000*KYmove4);title('KYmove4(mm)');

subplot(5,4,9);plot(Time,57.3*KToe1);title('KToe1(deg)');
subplot(5,4,10);plot(Time,57.3*KToe2);title('KToe2(deg)');
subplot(5,4,11);plot(Time,57.3*KToe3);title('KToe3(deg)');
subplot(5,4,12);plot(Time,57.3*KToe4);title('KToe4(deg)');

subplot(5,4,13);plot(Time,57.3*KDive1);title('KDive1(deg)');
subplot(5,4,14);plot(Time,57.3*KDive2);title('KDive2(deg)');
subplot(5,4,15);plot(Time,57.3*KDive3);title('KDive3(deg)');
subplot(5,4,16);plot(Time,57.3*KDive4);title('KDive4(deg)');

subplot(5,4,17);plot(Time,57.3*KCamber1);title('KCamber1(deg)');
subplot(5,4,18);plot(Time,57.3*KCamber2);title('KCamber2(deg)');
subplot(5,4,19);plot(Time,57.3*KCamber3);title('KCamber3(deg)');
subplot(5,4,20);plot(Time,57.3*KCamber4);title('KCamber4(deg)');

figure(9);
subplot(5,4,1);plot(Time,57.3*CToe1_fx);title('CToe1Fx(deg)');
subplot(5,4,2);plot(Time,57.3*CToe2_fx);title('CToe2Fx(deg)');
subplot(5,4,3);plot(Time,57.3*CToe3_fx);title('CToe3Fx(deg)');
subplot(5,4,4);plot(Time,57.3*CToe4_fx);title('CToe4Fx(deg)');

subplot(5,4,5);plot(Time,57.3*CSteer1_fy);title('CSteer1Fy(deg)');
subplot(5,4,6);plot(Time,57.3*CSteer2_fy);title('CSteer2Fy(deg)');
subplot(5,4,7);plot(Time,57.3*CSteer3_fy);title('CSteer3Fy(deg)');
subplot(5,4,8);plot(Time,57.3*CSteer4_fy);title('CSteer4Fy(deg)');

subplot(5,4,9);plot(Time,57.3*CSteer1_Mz);title('CSteer1Mz(deg)');
subplot(5,4,10);plot(Time,57.3*CSteer2_Mz);title('CSteer2Mz(deg)');
subplot(5,4,11);plot(Time,57.3*CSteer3_Mz);title('CSteer3Mz(deg)');
subplot(5,4,12);plot(Time,57.3*CSteer4_Mz);title('CSteer4Mz(deg)');

subplot(5,4,13);plot(Time,57.3*CCamber1_Fx);title('CCamber1Fx(deg)');
subplot(5,4,14);plot(Time,57.3*CCamber2_Fx);title('CCamber2Fx(deg)');
subplot(5,4,15);plot(Time,57.3*CCamber3_Fx);title('CCamber3Fx(deg)');
subplot(5,4,16);plot(Time,57.3*CCamber4_Fx);title('CCamber4Fx(deg)');

subplot(5,4,17);plot(Time,57.3*CInc1_Fy);title('CInc1Fy(deg)');
subplot(5,4,18);plot(Time,57.3*CInc2_Fy);title('CInc2Fy(deg)');
subplot(5,4,19);plot(Time,57.3*CInc3_Fy);title('CInc3Fy(deg)');
subplot(5,4,20);plot(Time,57.3*CInc4_Fy);title('CInc4Fy(deg)');

figure(10);
subplot(5,4,1);plot(Time,57.3*CInc1_Mz);title('CInc1Mz(deg)');
subplot(5,4,2);plot(Time,57.3*CInc2_Mz);title('CInc2Mz(deg)');
subplot(5,4,3);plot(Time,57.3*CInc3_Mz);title('CInc3Mz(deg)');
subplot(5,4,4);plot(Time,57.3*CInc4_Mz);title('CInc4Mz(deg)');

subplot(5,4,5);plot(Time,1000*CXmove1);title('CXmove1(mm)');
subplot(5,4,6);plot(Time,1000*CXmove2);title('CXmove2(mm)');
subplot(5,4,7);plot(Time,1000*CXmove3);title('CXmove3(mm)');
subplot(5,4,8);plot(Time,1000*CXmove4);title('CXmove4(mm)');

subplot(5,4,9);plot(Time,1000*CYmove1);title('CYmove1(mm)');
subplot(5,4,10);plot(Time,1000*CYmove2);title('CYmove2(mm)');
subplot(5,4,11);plot(Time,1000*CYmove3);title('CYmove3(mm)');
subplot(5,4,12);plot(Time,1000*CYmove4);title('CYmove4(mm)');

subplot(5,4,13);plot(Time,57.3*CDive1);title('CDive1(deg)');
subplot(5,4,14);plot(Time,57.3*CDive2);title('CDive2(deg)');
subplot(5,4,15);plot(Time,57.3*CDive3);title('CDive3(deg)');
subplot(5,4,16);plot(Time,57.3*CDive4);title('CDive4(deg)');

figure(11);
subplot(3,4,1);plot(Time,x_rack*1000);title('x_rack(mm)');
subplot(3,4,2);plot(Time,dx_rack*1000);title('dx_rack(mm/s)');
subplot(3,4,3);plot(Time,ddx_rack*1000);title('ddx_rack(mm/s2)');
subplot(3,4,4);plot(Time,57.3*delta1);title('delta1(deg)左前转角');
subplot(3,4,5);plot(Time,57.3*delta2);title('delta2(deg)右前转角');
subplot(3,4,6);plot(Time,T_assist);title('T_assist(Nm)');

ratio_S = 44.12/1000/(2*pi);
subplot(3,4,7);plot(Time,57.3*(deltaV - x_rack/ratio_S));title('方向盘与转向机相对转角(deg)');
subplot(3,4,8);plot(Time,57.3*(deltaV));title('方向盘转角(deg)');
subplot(3,4,9);plot(Time,57.3*(ddeltaV));title('方向盘转角速度(deg/s)');
subplot(3,4,10);plot(Time,57.3*(dddeltaV));title('方向盘转角加速度(deg/s2)');

figure(12);
subplot(2,4,1);plot(Time,57.3*AngleAirR);title('AngleAirR(deg)');
subplot(2,4,2);plot(Time,VairR);title('VairR(m/s)');
subplot(2,4,3);plot(Time,FxAir);title('FxAir(N)');
subplot(2,4,4);plot(Time,FyAir);title('FyAir(N)');

figure(13);
plot(x,y);hold on;axis equal;
for i=1:nstep
    if mod(i,round(nstep/40))==0
        quiver(x(i),y(i),10*cos(yaw(i)),10*sin(yaw(i)),0.2,'r','LineWidth',1);
    end
end

dataOut = saveToCSV(nstep,20,Time,dx,dy,dz,dpth,dyaw,drll);
% dataOut2 = saveToCSV2(nstep,20,Time,x,y,z,pth,yaw,rll,deltaV);
% system('D:\data\Unreal Projects\CarPostProcessor0804\PackUp\PackUp0817\WindowsNoEditor\CarPostProcessor0804.exe')