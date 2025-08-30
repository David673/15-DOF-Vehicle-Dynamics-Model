function [VarP1,dVarOut, MidOut]=DynamicModel_15DOF(CtrlIn,ParIn, SimIn, VarIn,OthersIn)
%% 初始化 initializing for parameters

%% 其他量解包
i = 1;%
kappa1_former = OthersIn(2);
kappa2_former = OthersIn(3);
kappa3_former = OthersIn(4);
kappa4_former = OthersIn(5);

fx1_former = OthersIn(6);
fx2_former = OthersIn(7);
fx3_former = OthersIn(8);
fx4_former = OthersIn(9);

fy1_former = OthersIn(10);
fy2_former = OthersIn(11);
fy3_former = OthersIn(12);
fy4_former = OthersIn(13);

Twz1_former = OthersIn(14);
Twz2_former = OthersIn(15);
Twz3_former = OthersIn(16);
Twz4_former = OthersIn(17);

%% 状态变量输入
x(i) = VarIn(1);
y(i) = VarIn(2);
z(i) = VarIn(3);
yaw(i) = VarIn(4);
rll(i) = VarIn(5);
pth(i) = VarIn(6);

%--------------------------
s1(i) = VarIn(7);   s2(i) = VarIn(8);    s3(i) = VarIn(9);    s4(i) = VarIn(10);
ds1(i) = VarIn(11);   ds2(i) = VarIn(12);   ds3(i) = VarIn(13);   ds4(i) = VarIn(14);
%-----------------------------
n1(i) = VarIn(15);  n2(i) = VarIn(16);  n3(i) = VarIn(17); n4(i) = VarIn(18);
dn1(i) = VarIn(19);   dn2(i) = VarIn(20);  dn3(i) = VarIn(21);   dn4(i) = VarIn(22);
%------------------------
u(i) = VarIn(23);
v(i) = VarIn(24);
w(i) = VarIn(25);
p(i) = VarIn(26);
q(i) = VarIn(27);
r(i) = VarIn(28);
%-----------------------
x_rack(i) = VarIn(29);
dx_rack(i) = VarIn(30);
deltaV(i) = VarIn(31);
ddeltaV(i) = VarIn(32);

%% 参数集合解包
%--------------------
a = ParIn(1);
b = ParIn(2);
cf = ParIn(3);
cr = ParIn(4);
dsf = ParIn(5);
dsr = ParIn(6);
zf = ParIn(7);
zr = ParIn(8);
%---------------------
CA1 = ParIn(9);  CA2 = ParIn(10); CA3 = ParIn(11);  CA4 = ParIn(12);
IA1 = ParIn(13); IA2 = ParIn(14); IA3 = ParIn(15); IA4 = ParIn(16);
psio1 = ParIn(17); psio2 = ParIn(18); psio3 = ParIn(19); psio4 = ParIn(20);
thto1 = ParIn(21); thto2 = ParIn(22); thto3 = ParIn(23); thto4 = ParIn(24);
%----------------------------------------------------------------------------------------------------
Ktz1 = ParIn(25); Ktz2 = ParIn(26);  Ktz3 = ParIn(27); Ktz4 = ParIn(28);
Ktx1 = ParIn(29); Ktx2 = ParIn(30); Ktx3 = ParIn(31); Ktx4 = ParIn(32);
Kty1 = ParIn(33); Kty2 = ParIn(34); Kty3 = ParIn(35); Kty4 = ParIn(36);
RllRes1 = ParIn(37); RllRes2 = ParIn(38); RllRes3 = ParIn(39); RllRes4 = ParIn(40);
width1 = ParIn(41) ; width2 = ParIn(42); width3 = ParIn(43); width4 = ParIn(44);
%----------------------------------------------------------------------------------------------------------
Ixx = ParIn(45);
Iyy = ParIn(46);
Izz = ParIn(47);
Ixy = ParIn(48);
Ixz = ParIn(49);%-110
Iyz = ParIn(50);
m = ParIn(51);
%-------------------------
Rf1 = ParIn(52); Rf2 = ParIn(53); Rf3 = ParIn(54); Rf4 = ParIn(55);
k1 = ParIn(56); k2 = ParIn(57); k3 = ParIn(58); k4 = ParIn(59);
c1 = ParIn(60); c2 = ParIn(61);  c3 = ParIn(62);  c4 = ParIn(63);
I1 = ParIn(64); I2 = ParIn(65); I3 = ParIn(66); I4 = ParIn(67);
m1 = ParIn(68); m2 = ParIn(69); m3 = ParIn(70); m4 = ParIn(71);
preload1 = ParIn(72); preload2 = ParIn(73);
preload3 = ParIn(74); preload4 = ParIn(75);
%--------------------------------------------------------------------------------------------
Hsz1 = ParIn(76); Hsz2 = ParIn(77);  Hsz3 = ParIn(78);  Hsz4 = ParIn(79);

%轮心在车身系下的横纵坐标(m)不考虑K特性
% xwc1 = ParIn(80);
% ywc1 = ParIn(81);
% 
% xwc2 = ParIn(82);
% ywc2 = ParIn(83);
% 
% xwc3 = ParIn(84);
% ywc3 = ParIn(85);
% 
% xwc4 = ParIn(86);
% ywc4 = ParIn(87);

%% 轮心在车身系下的横纵坐标(m)考虑KC特性
KXmove1 = 0;
KXmove2 = 0;
KXmove3 = 0;
KXmove4 = 0;

KYmove1 = 0;
KYmove2 = 0;
KYmove3 = 0;
KYmove4 = 0;

%加入C特性

CXmove1 = 0;
CXmove2 = 0;
CXmove3 = 0;
CXmove4 = 0;

CYmove1 = 0;
CYmove2 = 0;
CYmove3 = 0;
CYmove4 = 0;

% 考虑KC特性之后的轮心位移

xwc1 = ParIn(80) + KXmove1 + CXmove1;
ywc1 = ParIn(81) + KYmove1 + CYmove1;

xwc2 = ParIn(82) + KXmove2 + CXmove2;
ywc2 = ParIn(83) + KYmove2 + CYmove2;

xwc3 = ParIn(84) + KXmove3 + CXmove3;
ywc3 = ParIn(85) + KYmove3 + CYmove3;

xwc4 = ParIn(86) + KXmove4 + CXmove4;
ywc4 = ParIn(87) + KYmove4 + CYmove4;
%-------------------
g = ParIn(88);

%% 仿真过程参数解包
step = SimIn(1);

%% 控制、外部输入变量解包
delta = CtrlIn(1);

Td1= CtrlIn(2); Td2= CtrlIn(3); Td3= CtrlIn(4); Td4= CtrlIn(5);
Tb1= CtrlIn(6); Tb2= CtrlIn(7); Tb3= CtrlIn(8); Tb4= CtrlIn(9);
Zd1= CtrlIn(10);Zd2= CtrlIn(11);Zd3= CtrlIn(12);Zd4= CtrlIn(13);

TSWin = CtrlIn(14);



%% -----------求解前的准备-----------------------------------------
[dyaw(i),dpth(i),drll(i)]=AngleRateA(p(i),q(i),r(i),rll(i),pth(i));
    %-------------算法需改进-----------------------------------------
    rll(i+1)=rll(i)+step*drll(i);
    pth(i+1)=pth(i)+step*dpth(i);
    yaw(i+1)= yaw(i)+step*dyaw(i);
    %1大地-车身坐标变换矩阵
    Cga = CgaCal(rll(i),pth(i),yaw(i));
    temp1 = Cga*[u(i);v(i);w(i)];
    dx(i) = temp1(1);
    dy(i) = temp1(2);
    dz(i) = temp1(3);
    
    x(i+1) = x(i) + step * dx(i);
    y(i+1) = y(i) + step * dy(i);
    z(i+1) = z(i) + step * dz(i);
    %2相对风速与空气阻力
    [AngleAirR,VairR] = RelativeAirVelocity(temp1,[0;0;0],Cga);
    [FxAir,FyAir,FzAir,MxAir,MyAir,MzAir,XAir,~,~] = Aerodynamics(VairR,AngleAirR);
    %1车身-轮胎坐标变换矩阵考虑k特性
    
    KToe1 = 0;
    KToe2 = 0;
    KToe3 = 0;
    KToe4 = 0;
    
    KDive1 = 0;
    KDive2 = 0;
    KDive3 = 0;
    KDive4 = 0;
    
    KCamber1 = 0;%-
    KCamber2 = 0;%+
    KCamber3 = 0;%-
    KCamber4 = 0;%+
    
    CToe1_Fx = 0;
    CToe2_Fx = 0;
    CToe3_Fx = 0;
    CToe4_Fx = 0;
    
    CSteer1_Fy = 0;
    CSteer2_Fy = 0;
    CSteer3_Fy = 0;
    CSteer4_Fy = 0;
    
    CSteer1_Mz = 0;
    CSteer2_Mz = 0;
    CSteer3_Mz = 0;
    CSteer4_Mz = 0;
    
    CCamber1_Fx = 0;
    CCamber2_Fx = 0;
    CCamber3_Fx = 0;
    CCamber4_Fx = 0;
    
    CInc1_Fy = 0;
    CInc2_Fy = 0;
    CInc3_Fy = 0;
    CInc4_Fy = 0;
    
    CInc1_Mz = 0;
    CInc2_Mz = 0;
    CInc3_Mz = 0;
    CInc4_Mz = 0;
    
    CDive1 = 0;
    CDive2 = 0;
    CDive3 = 0;
    CDive4 = 0;
    
    % 考虑转向阿克曼
    [delta1(i),delta2(i)] = KAckermann(x_rack(i));
    
    Caw1 = CawCal((psio1 + delta1(i) + 1*KToe1 + 1*CToe1_Fx + 1*CSteer1_Fy + 1*CSteer1_Mz),(CA1 + 1*KDive1 + CDive1),(thto1 + 1*KCamber1 + CCamber1_Fx + CInc1_Fy + CInc1_Mz));%含k特性
    Caw2 = CawCal((psio2 + delta2(i) + 1*KToe2 + 1*CToe2_Fx + 1*CSteer2_Fy + 1*CSteer2_Mz),(CA2 + 1*KDive2 + CDive2),(thto2 + 1*KCamber2 + CCamber2_Fx + CInc2_Fy + CInc2_Mz));%含k特性
    Caw3 = CawCal((psio3 + 1*KToe3 + 1*CToe3_Fx + 1*CSteer3_Fy + 1*CSteer3_Mz),(CA3 + 1*KDive3 + CDive3),(thto3 + 1*KCamber3 + CCamber3_Fx + CInc3_Fy + CInc3_Mz));
    Caw4 = CawCal((psio4 + 1*KToe4 + 1*CToe4_Fx + 1*CSteer4_Fy + 1*CSteer4_Mz),(CA4 + 1*KDive4 + CDive4),(thto4 + 1*KCamber4 + CCamber4_Fx + CInc4_Fy + CInc4_Mz));
    
    % 1车身-轮胎坐标变换矩阵不考虑k特性
%     Caw1 = CawCal((psio1+delta(i)),CA1,thto1);
%     Caw2 = CawCal((psio2+delta(i)),CA2,thto2);
%     Caw3 = CawCal(psio3,CA3,thto3);
%     Caw4 = CawCal(psio4,CA4,thto4);
        
    %% 继续求解
        
    zwc1 = zf + s1(i);
    zwc2 = zf + s2(i);
    zwc3 = zr + s3(i);
    zwc4 = zr + s4(i);
    %1悬架弹簧阻尼力
    fs1(i) =  k1*s1(i) - c1*ds1(i) - preload1;
    fs2(i) =  k2*s2(i) - c2*ds2(i) - preload2;
    fs3(i) =  k3*s3(i) - c3*ds3(i) - preload3;
    fs4(i) =  k4*s4(i) - c4*ds4(i) - preload4;
    %2车轮动态半径
    Zwc1 = Cga(3,1)*xwc1 + Cga(3,2)*ywc1 + Cga(3,3)*zwc1 + z(i);
    Zwc2 = Cga(3,1)*xwc2 + Cga(3,2)*ywc2 + Cga(3,3)*zwc2 + z(i);
    Zwc3 = Cga(3,1)*xwc3 + Cga(3,2)*ywc3 + Cga(3,3)*zwc3 + z(i);
    Zwc4 = Cga(3,1)*xwc4 + Cga(3,2)*ywc4 + Cga(3,3)*zwc4 + z(i);
    
    Cgw1 = Cga*Caw1;
    Cgw2 = Cga*Caw2;
    Cgw3 = Cga*Caw3;
    Cgw4 = Cga*Caw4;
    
    Rd1(i) = (Zd1(i) - Zwc1)/Cgw1(3,3);
    Rd2(i) = (Zd2(i) - Zwc2)/Cgw2(3,3);
    Rd3(i) = (Zd3(i) - Zwc3)/Cgw3(3,3);
    Rd4(i) = (Zd4(i) - Zwc4)/Cgw4(3,3);
    
    %2车身系下重力加速度投影
    temp2 = Cga\[0;0;g];
    gx = temp2(1);
    gy = temp2(2);
    gz = temp2(3);
    
    %3轮胎垂向力
    faz1 = Caw1\[0, 0, (Ktz1 * (Rf1 - Rd1(i)))]';
    faz2 = Caw2\[0, 0, (Ktz2 * (Rf2 - Rd2(i)))]';
    faz3 = Caw3\[0, 0, (Ktz3 * (Rf3 - Rd3(i)))]';
    faz4 = Caw4\[0, 0, (Ktz4 * (Rf4 - Rd4(i)))]';
    
    fwz1(i) = faz1(3);
    fwz2(i) = faz2(3);
    fwz3(i) = faz3(3);
    fwz4(i) = faz4(3);
    
    if (fwz1(i)<0)
        fwz1(i) = 0;
    end
    if (fwz2(i)<0)
        fwz2(i) = 0;
    end
    if (fwz3(i)<0)
        fwz3(i) = 0;
    end
    if (fwz4(i)<0)
        fwz4(i) = 0;
    end
    
    
    %轮胎接地点在车身系中的位移
    xwp1 = xwc1 + Rd1(i) * Caw1(1,3);
    ywp1 = ywc1 + Rd1(i) * Caw1(2,3);
    zwp1 = zwc1 + Rd1(i) * Caw1(3,3);
    
    xwp2 = xwc2 + Rd2(i) * Caw2(1,3);
    ywp2 = ywc2 + Rd2(i) * Caw2(2,3);
    zwp2 = zwc2 + Rd2(i) * Caw2(3,3);
    
    xwp3 = xwc3 + Rd3(i) * Caw3(1,3);
    ywp3 = ywc3 + Rd3(i) * Caw3(2,3);
    zwp3 = zwc3 + Rd3(i) * Caw3(3,3);
    
    xwp4 = xwc4 + Rd1(i) * Caw4(1,3);
    ywp4 = ywc4 + Rd1(i) * Caw4(2,3);
    zwp4 = zwc4 + Rd1(i) * Caw4(3,3);
    
    %4滚动阻力矩
    Twy1(i) = sign(dn1(i)) * RllRes1 * fwz1(i);
    Twy2(i) = sign(dn2(i)) * RllRes2 * fwz2(i);
    Twy3(i) = sign(dn3(i)) * RllRes3 * fwz3(i);
    Twy4(i) = sign(dn4(i)) * RllRes4 * fwz4(i);
    
    %4轮胎接地点速度（车身系）
    uw1 = u(i) + q(i)*zwp1 - r(i)*ywp1;
    vw1 = v(i) - p(i)*zwp1 + r(i)*xwp1;
    ww1 = w(i) + p(i)*ywp1 - q(i)*xwp1 + ds1(i);
    %ww1 = 0;

    uw2 = u(i) + q(i)*zwp2 - r(i)*ywp2;
    vw2 = v(i) - p(i)*zwp2 + r(i)*xwp2;
    ww2 = w(i) + p(i)*ywp2 - q(i)*xwp2 + ds2(i);
   % ww2 = 0;

    uw3 = u(i) + q(i)*zwp3 - r(i)*ywp3;
    vw3 = v(i) - p(i)*zwp3 + r(i)*xwp3;
    ww3 = w(i) + p(i)*ywp3 - q(i)*xwp3 + ds3(i);
   % ww3 = 0;

    uw4 = u(i) + q(i)*zwp4 - r(i)*ywp4;
    vw4 = v(i) - p(i)*zwp4 + r(i)*xwp4;
    ww4 = w(i) + p(i)*ywp4 - q(i)*xwp4 + ds4(i);
   % ww4 = 0;
    
    %5轮胎接地点速度（轮胎系）---------------需要分析【分母有0】-----------------
    VVt1 = Caw1\([uw1,vw1,ww1]');
    VVt2 = Caw2\([uw2,vw2,ww2]');
    VVt3 = Caw3\([uw3,vw3,ww3]');
    VVt4 = Caw4\([uw4,vw4,ww4]');
    uwt1 = VVt1(1);
    vwt1 = VVt1(2);
    uwt2 = VVt2(1);
    vwt2 = VVt2(2);
    uwt3 = VVt3(1);
    vwt3 = VVt3(2);
    uwt4 = VVt4(1);
    vwt4 = VVt4(2);



    %6纵向滑移速度
    Vsx1 = uwt1 - dn1(i) * Rd1(i);
    Vsx2 = uwt2 - dn2(i) * Rd2(i);
    Vsx3 = uwt3 - dn3(i) * Rd3(i);
    Vsx4 = uwt4 - dn4(i) * Rd4(i);
    
    %6侧向滑移速度
    Vsy1 = vwt1;
    Vsy2 = vwt2;                                     
    Vsy3 = vwt3;                                     
    Vsy4 = vwt4;
    
    %% -----------侧偏角与纵滑率-------------
    
    %7纵向滑移率------------【需要分析】【有速度时暂时ok】--------------------
    kappa1(i) = -Vsx1/(uwt1);
    kappa2(i) = -Vsx2/(uwt2);                             
    kappa3(i) = -Vsx3/(uwt3);                             
    kappa4(i) = -Vsx4/(uwt4);     
    %纵滑率上下限为正负1
    if (kappa1(i)>=1)||(kappa1(i)<=-1)
        kappa1(i)=1*sign(kappa1(i));
    end
    if (kappa2(i)>=1)||(kappa2(i)<=-1)
        kappa2(i)=1*sign(kappa2(i));
    end
    if (kappa3(i)>=1)||(kappa3(i)<=-1)
        kappa3(i)=1*sign(kappa3(i));
    end
    if (kappa4(i)>=1)||(kappa4(i)<=-1)
        kappa4(i)=1*sign(kappa4(i));
    end
    
    
    %考虑纵滑率不稳定的情况
    if (uwt1==0&&i>1)
        kappa1(i)=kappa1_former;
    else
        if (uwt1==0&&i==1)
            kappa1(i)=0;
        end
    end
    
    if (uwt2==0&&i>1)
        kappa2(i)=kappa2_former;
    else
        if (uwt2==0&&i==1)
            kappa2(i)=0;
        end
    end
    
    if (uwt3==0&&i>1)
        kappa3(i)=kappa3_former;
    else
        if (uwt3==0&&i==1)
            kappa3(i)=0;
        end
    end
    
    if (uwt4==0&&i>1)
        kappa4(i)=kappa4_former;
    else
        if (uwt4==0&&i==1)
            kappa4(i)=0;
        end
    end
    
    %7侧偏角(rad)--------------------【需要分析】【有速度时暂时ok】----------------------------
    alpha1(i) = atan( Vsy1 / (dn1(i)*Rd1(i)));
    alpha2(i) = atan( Vsy2 / (dn2(i)*Rd2(i)));
    alpha3(i) = atan( Vsy3 / (dn3(i)*Rd3(i)));
    alpha4(i) = atan( Vsy4 / (dn4(i)*Rd4(i)));
    if(abs(u(i))<2.5 && u(i)~=0)   %低速下限制侧偏角的值以提高稳定性
        alpha1(i) = alpha1(i) * ((1-0.01)*abs(u(i))/2.5 + 0.01);
        alpha2(i) = alpha2(i) * ((1-0.01)*abs(u(i))/2.5 + 0.01);
        alpha3(i) = alpha3(i) * ((1-0.01)*abs(u(i))/2.5 + 0.01);
        alpha4(i) = alpha4(i) * ((1-0.01)*abs(u(i))/2.5 + 0.01);
    end
    if (abs(dn1(i))<5&&dn1(i)~=0)%车轮抱死情况下的处理方法
        alpha1(i) = alpha1(i) * ((1-0.01)*abs(dn1(i))/5 + 0.01);
    end
    if (abs(dn2(i))<5&&dn2(i)~=0)
        alpha2(i) = alpha2(i) * ((1-0.01)*abs(dn2(i))/5 + 0.01);
    end
    if (abs(dn3(i))<5&&dn3(i)~=0)
        alpha3(i) = alpha3(i) * ((1-0.01)*abs(dn3(i))/5 + 0.01);
    end
    if (abs(dn4(i))<5&&dn4(i)~=0)
        alpha4(i) = alpha4(i) * ((1-0.01)*abs(dn4(i))/5 + 0.01);
    end
    
    %% 线性截止轮胎模型
    %8纵向力
%     fwx1(i) = Ktx1 * kappa1(i) * fwz1(i);
%     fwx2(i) = Ktx2 * kappa2(i) * fwz2(i);
%     fwx3(i) = Ktx3 * kappa3(i) * fwz3(i);
%     fwx4(i) = Ktx4 * kappa4(i) * fwz4(i);
%     
%     if (abs(fwx1(i))>fwz1(i))
%         fwx1(i)=sign(kappa1(i))*fwz1(i);
%     end
%     if (abs(fwx2(i))>fwz2(i))
%         fwx2(i)=sign(kappa2(i))*fwz2(i);
%     end
%     if (abs(fwx3(i))>fwz3(i))
%         fwx3(i)=sign(kappa3(i))*fwz3(i);
%     end
%     if (abs(fwx4(i))>fwz4(i))
%         fwx4(i)=sign(kappa4(i))*fwz4(i);
%     end
%     %8侧向力,这里隐含了侧偏刚度是负值的概念
%     fwy1(i) = - Kty1 * alpha1(i) * fwz1(i);
%     fwy2(i) = - Kty2 * alpha2(i) * fwz2(i);
%     fwy3(i) = - Kty3 * alpha3(i) * fwz3(i);
%     fwy4(i) = - Kty4 * alpha4(i) * fwz4(i);
%     if (abs(fwy1(i))>fwz1(i))
%         fwy1(i)=-sign(alpha1(i))*fwz1(i);
%     end
%     if (abs(fwy2(i))>fwz2(i))
%         fwy2(i)=-sign(alpha2(i))*fwz2(i);
%     end
%     if (abs(fwy3(i))>fwz3(i))
%         fwy3(i)=-sign(alpha3(i))*fwz3(i);
%     end
%     if (abs(fwy4(i))>fwz4(i))
%         fwy4(i)=-sign(alpha4(i))*fwz4(i);
%     end
    
    %% MF轮胎模型
    [fwx1(i),fwy1(i),~,~,MzMF1] = PAC2002_forces( fwz1(i) , kappa1(i) , alpha1(i) , (thto1 + 1*KCamber1 + CCamber1_Fx + CInc1_Fy + CInc1_Mz) );
    [fwx2(i),fwy2(i),~,~,MzMF2] = PAC2002_forces( fwz2(i) , kappa2(i) , alpha2(i) , (thto2 + 1*KCamber2 + CCamber2_Fx + CInc2_Fy + CInc2_Mz) );
    [fwx3(i),fwy3(i),~,~,MzMF3] = PAC2002_forces_R( fwz3(i) , kappa3(i) , alpha3(i) , (thto3 + 1*KCamber3 + CCamber3_Fx + CInc3_Fy + CInc3_Mz) );
    [fwx4(i),fwy4(i),~,~,MzMF4] = PAC2002_forces_R( fwz4(i) , kappa4(i) , alpha4(i) , (thto4 + 1*KCamber4 + CCamber4_Fx + CInc4_Fy + CInc4_Mz) );
    
    %% 回正力矩
    %9回正力矩,隐含了侧向力与轮胎回正力矩反向的信息
%     Twz1(i) = sign(alpha1(i)) * width1 * abs(fwy1(i));
%     Twz2(i) = sign(alpha2(i)) * width2 * abs(fwy2(i));
%     Twz3(i) = sign(alpha3(i)) * width3 * abs(fwy3(i));
%     Twz4(i) = sign(alpha4(i)) * width4 * abs(fwy4(i));
% 考虑主销几何
%     Twz1(i) = Twz_Reimpell(fwx1(i),fwy1(i),fwz1(i),(IA1+1*KCamber1 + CCamber1_Fx + CInc1_Fy + CInc1_Mz),(CA1 + 1*KDive1 + CDive1),75.5/1000,0,delta1(i),width1, Rd1(i), -5/1000);
%     Twz2(i) = Twz_Reimpell(fwx2(i),fwy2(i),fwz2(i),(IA2+1*KCamber2 + CCamber2_Fx + CInc2_Fy + CInc2_Mz),(CA2 + 1*KDive2 + CDive2),75.5/1000,0,delta2(i),width2, Rd2(i), 5/1000);
%     Twz3(i) = Twz_Reimpell(fwx3(i),fwy3(i),fwz3(i),(IA3+1*KCamber3 + CCamber3_Fx + CInc3_Fy + CInc3_Mz),(CA3 + 1*KDive3 + CDive3),0,0,0,width3, Rd3(i), -5/1000);
%     Twz4(i) = Twz_Reimpell(fwx4(i),fwy4(i),fwz4(i),(IA4+1*KCamber4 + CCamber4_Fx + CInc4_Fy + CInc4_Mz),(CA4 + 1*KDive4 + CDive4),0,0,0,width4, Rd4(i), 5/1000);
% 在MF轮胎模型下考虑主销几何    
    Twz1(i) = Twz_Reimpell_MF(fwx1(i),fwy1(i),fwz1(i),(IA1+1*KCamber1 + CCamber1_Fx + CInc1_Fy + CInc1_Mz),(CA1 + 1*KDive1 + CDive1),75.5/1000,0,delta1(i),MzMF1, Rd1(i));
    Twz2(i) = Twz_Reimpell_MF(fwx2(i),fwy2(i),fwz2(i),(IA2+1*KCamber2 + CCamber2_Fx + CInc2_Fy + CInc2_Mz),(CA2 + 1*KDive2 + CDive2),75.5/1000,0,delta2(i),MzMF2, Rd2(i));
    Twz3(i) = Twz_Reimpell_MF(fwx3(i),fwy3(i),fwz3(i),(IA3+1*KCamber3 + CCamber3_Fx + CInc3_Fy + CInc3_Mz),(CA3 + 1*KDive3 + CDive3),0,0,0,MzMF3, Rd3(i));
    Twz4(i) = Twz_Reimpell_MF(fwx4(i),fwy4(i),fwz4(i),(IA4+1*KCamber4 + CCamber4_Fx + CInc4_Fy + CInc4_Mz),(CA4 + 1*KDive4 + CDive4),0,0,0,MzMF4, Rd4(i));
    %9轮胎力转到车身系
    temp3 = Caw1*[fwx1(i) + faz1(1), fwy1(i) + faz1(2), fwz1(i)]';
    fx1(i) = temp3(1);
    fy1(i) = temp3(2);
    fz1(i) = temp3(3);
    
    temp4 = Caw2*[fwx2(i) + faz2(1), fwy2(i) + faz2(2), fwz2(i)]';   
    fx2(i) = temp4(1);
    fy2(i) = temp4(2);
    fz2(i) = temp4(3);
    
    temp5 = Caw3*[fwx3(i) + faz3(1), fwy3(i) + faz3(2), fwz3(i)]';
    fx3(i) = temp5(1);
    fy3(i) = temp5(2);
    fz3(i) = temp5(3);
    
    temp6 = Caw4*[fwx4(i) + faz4(1), fwy4(i) + faz4(2), fwz4(i)]';   
    fx4(i) = temp6(1);
    fy4(i) = temp6(2);
    fz4(i) = temp6(3);
        
    %% -----------解方程------------------------------------
    % 1转向系统动力学
    %转角输入
%     [ddx_rack(i),T_assist] = SteerByWire(Twz1(i),Twz2(i),dx_rack(i),fz1(i),fz2(i),delta(i),x_rack(i));
%     dddeltaV(i) = 0;
%     ddeltaV(i+1) = 0;
%     deltaV(i+1) = delta(i);

    
    %力矩输入
    [ddx_rack(i),T_assist,dddeltaV(i)] = SteerByWire_TorqueIn(Twz1(i),Twz2(i),dx_rack(i),fz1(i),fz2(i),deltaV(i),x_rack(i),TSWin(i),ddeltaV(i));
  
    
    ddeltaV(i+1) = ddeltaV(i) + step * dddeltaV(i);
    deltaV(i+1) = deltaV(i) + step * ddeltaV(i+1);
    
    
    %公共部分
    dx_rack(i+1) = dx_rack(i) + step * ddx_rack(i);
    x_rack(i+1) = x_rack(i) + step * dx_rack(i+1);
    
    %方向盘转角限位
    if abs(deltaV(i))>1.875*2*pi
        deltaV(i) = 1.875*2*pi * sign(deltaV(i));
        deltaV(i+1) = 1.875*2*pi * sign(deltaV(i));
        ddeltaV(i) = 0;
        ddeltaV(i+1)= 0;
        dddeltaV(i) = 0;
    end
    
    %1轮胎自旋(与其他方程解耦)
%     ddn1(i) = (Td1(i) - Tb1(i) - (1)*Rd1(i) * fwx1(i) * sign(dn1(i)) - Twy1(i)) / I1;
%     ddn2(i) = (Td2(i) - Tb2(i) - (1)*Rd2(i) * fwx2(i) * sign(dn2(i)) - Twy2(i)) / I2;
%     ddn3(i) = (Td3(i) - Tb3(i) - (1)*Rd3(i) * fwx3(i) * sign(dn3(i)) - Twy3(i)) / I3;
%     ddn4(i) = (Td4(i) - Tb4(i) - (1)*Rd4(i) * fwx4(i) * sign(dn4(i)) - Twy4(i)) / I4;
%     
%     ddn1(i) = (Td1(i) - Tb1(i) - (1)*Rd1(i) * fwx1(i) * sign(u(i)) - Twy1(i)) / I1;
%     ddn2(i) = (Td2(i) - Tb2(i) - (1)*Rd2(i) * fwx2(i) * sign(u(i)) - Twy2(i)) / I2;
%     ddn3(i) = (Td3(i) - Tb3(i) - (1)*Rd3(i) * fwx3(i) * sign(u(i)) - Twy3(i)) / I3;
%     ddn4(i) = (Td4(i) - Tb4(i) - (1)*Rd4(i) * fwx4(i) * sign(u(i)) - Twy4(i)) / I4;


    ddn1(i) = (Td1(i) - Tb1(i)*sign(dn1(i))*sign(u(i)) - (1)*Rd1(i) * fwx1(i) * sign(u(i)) - Twy1(i)) / I1;
    ddn2(i) = (Td2(i) - Tb2(i)*sign(dn2(i))*sign(u(i)) - (1)*Rd2(i) * fwx2(i) * sign(u(i)) - Twy2(i)) / I2;
    ddn3(i) = (Td3(i) - Tb3(i)*sign(dn3(i))*sign(u(i)) - (1)*Rd3(i) * fwx3(i) * sign(u(i)) - Twy3(i)) / I3;
    ddn4(i) = (Td4(i) - Tb4(i)*sign(dn4(i))*sign(u(i)) - (1)*Rd4(i) * fwx4(i) * sign(u(i)) - Twy4(i)) / I4;
    
    
    %状态变量更新------------算法可改进---------------------------
    dn1(i+1) = dn1(i) + ddn1(i) * step;
    n1(i+1) = n1(i) + dn1(i) * step;
    
    dn2(i+1) = dn2(i) + ddn2(i) * step;
    n2(i+1) = n2(i) + dn2(i) * step;
    
    dn3(i+1) = dn3(i) + ddn3(i) * step;
    n3(i+1) = n3(i) + dn3(i) * step;
    
    dn4(i+1) = dn4(i) + ddn4(i) * step;
    n4(i+1) = n4(i) + dn4(i) * step;
    
    %车身系下轮心速度
    u1 = u(i)+q(i)*zwc1-r(i)*ywc1;
    v1 = v(i)-p(i)*zwc1+r(i)*ywc1;
    w1 = w(i)+ds1(i)+p(i)*ywc1-q(i)*xwc1;
    
    u2 = u(i)+q(i)*zwc2-r(i)*ywc2;
    v2 = v(i)-p(i)*zwc2+r(i)*ywc2;
    w2 = w(i)+ds2(i)+p(i)*ywc2-q(i)*xwc2;

    u3 = u(i)+q(i)*zwc3-r(i)*ywc3;
    v3 = v(i)-p(i)*zwc3+r(i)*ywc3;
    w3 = w(i)+ds3(i)+p(i)*ywc3-q(i)*xwc3;

    u4 = u(i)+q(i)*zwc4-r(i)*ywc4;
    v4 = v(i)-p(i)*zwc4+r(i)*ywc4;
    w4 = w(i)+ds4(i)+p(i)*ywc4-q(i)*xwc4;

    
    
    %剩余十个方程
    FFx = fx1(i) + fx2(i) + fx3(i) + fx4(i) + FxAir ...
        + 1*(m + m1 + m2 + m3 + m4) * gx ...
        - m * (1)*(- r(i)*v(i) +q(i)*w(i))...
        - m1 * ( q(i) * w1 - r(i) * v1 + q(i) * ds1(i))...
        - m2 * ( q(i) * w2 - r(i) * v2 + q(i) * ds2(i))...
        - m3 * ( q(i) * w3 - r(i) * v3 + q(i) * ds3(i))...
        - m4 * ( q(i) * w4 - r(i) * v4 + q(i) * ds4(i));

    FFy = (fy1(i) + fy2(i) + fy3(i) + fy4(i))*sign(u(i)) + FyAir ...%%%%%【】【】【】【】【】【】【】【】【】【】【】【】【】【】【】
        + (m + m1 + m2 + m3 + m4) * gy ...
        - m1 * ( -p(i) * w1 + r(i) * u1 - p(i) * ds1(i))...
        - m2 * ( -p(i) * w2 + r(i) * u2 - p(i) * ds2(i))...
        - m3 * ( -p(i) * w3 + r(i) * u3 - p(i) * ds3(i))...
        - m4 * ( -p(i) * w4 + r(i) * u4 - p(i) * ds4(i))...
        - m*(- p(i)*w(i) + r(i)*u(i));

    FFz = fs1(i) + fs2(i) + fs3(i) + fs4(i) + m * gz + FzAir...
        - m*(- q(i)*u(i) + p(i)*v(i));

    FFz1 = -fs1(i) - fz1(i) + m1*gz ...
        - m1 * ( p(i) * v1 - q(i) * u1);

    FFz2 = -fs2(i) - fz2(i) + m2*gz ...
        - m2 * ( p(i) * v2 - q(i) * u2);

    FFz3 = -fs3(i) - fz3(i) + m3*gz ...
        - m3 * ( p(i) * v3 - q(i) * u3);

    FFz4 = -fs4(i) - fz4(i) + m4*gz ...
        - m4 * ( p(i) * v4 - q(i) * u4);

    MMx = -fy1(i)*zwp1 - fy2(i)*zwp2 - fy3(i)*zwp3 - fy4(i)*zwp4 + MxAir ...%【】【】【】【】【】【】【】【】【】【】【】
        + (1)*(fs2(i) - fs1(i))*dsf + (1)*(fs4(i) - fs3(i))*dsr ...
        - (- q(i)*Ixz*p(i) - r(i)*Iyy*q(i) + q(i)*Izz*r(i));

    MMy =  (Td1(i)+Td2(i)+Td3(i)+Td4(i)) - (Tb1(i)+Tb2(i)+Tb3(i)+Tb4(i))*sign(u(i))  + MyAir + FzAir * (XAir - a)...
        - (fs1(i)+fs2(i))*a + (fs3(i)+fs4(i))*b ...
        + fx1(i)*Hsz1 + fx2(i)*Hsz2 + fx3(i)*Hsz3 + fx4(i)*Hsz4 ...
        - ((Ixx*r(i) + Ixz*p(i))*p(i) + (-Ixz*r(i) - Izz*p(i))*r(i));

%     MMz =  (1)*(Twz1(i) + Twz2(i) + Twz3(i) + Twz4(i))... %【】【】【】【】【】【】【】【】【】【】【】【】【】【】【】
%         + (sign(u(i)))*((1)*fy1(i)*xwp1 + (1)*fy2(i)*xwp2 + (1)*fy3(i)*xwp3 + (1)*fy4(i)*xwp4)... %【】【】【】【】【】【】【
%         - (fx1(i)*ywp1 + fx2(i)*ywp2 + fx3(i)*ywp3 + fx4(i)*ywp4) ...
%         + (m1*gy*a + m2*gy*a - m3*gy*b - m4*gy*b)...
%         - (- q(i)*Ixx*p(i) + q(i)*Ixz*r(i) + p(i)*Iyy*q(i))...%good
%         - m1 *cf/2* ( q(i) * w1 - r(i) * v1 + q(i) * ds1(i))...
%         - (- m2 *cf/2)* ( q(i) * w2 - r(i) * v2 + q(i) * ds2(i)) ...
%         - m3 *cr/2* ( q(i) * w3 - r(i) * v3 + q(i) * ds3(i)) ...
%         - (- m4 *cr/2)* ( q(i) * w4 - r(i) * v4 + q(i) * ds4(i))...
%         - m1 *a* ( -p(i) * w1 + r(i) * u1 - p(i) * ds1(i)) ...
%         - m2 *a* ( -p(i) * w2 + r(i) * u2 - p(i) * ds2(i)) ...
%         - (- m3 *b)* ( -p(i) * w3 + r(i) * u3 - p(i) * ds3(i)) ...
%         - (- m4 *b)* ( -p(i) * w4 + r(i) * u4 - p(i) * ds4(i));

    MMz =  (1)*(-T_assist(i) + Twz3(i) + Twz4(i)) + MzAir + (-1)*FyAir * (XAir - a)... %【】【】【】【】【】【】【】【】【】【】【】【】【】【】【】
        + (sign(u(i)))*((1)*fy1(i)*xwp1 + (1)*fy2(i)*xwp2 + (1)*fy3(i)*xwp3 + (1)*fy4(i)*xwp4)... %【】【】【】【】【】【】【
        - (fx1(i)*ywp1 + fx2(i)*ywp2 + fx3(i)*ywp3 + fx4(i)*ywp4) ...
        + (m1*gy*a + m2*gy*a - m3*gy*b - m4*gy*b)...
        - (- q(i)*Ixx*p(i) + q(i)*Ixz*r(i) + p(i)*Iyy*q(i))...%good
        - m1 *cf/2* ( q(i) * w1 - r(i) * v1 + q(i) * ds1(i))...
        - (- m2 *cf/2)* ( q(i) * w2 - r(i) * v2 + q(i) * ds2(i)) ...
        - m3 *cr/2* ( q(i) * w3 - r(i) * v3 + q(i) * ds3(i)) ...
        - (- m4 *cr/2)* ( q(i) * w4 - r(i) * v4 + q(i) * ds4(i))...
        - m1 *a* ( -p(i) * w1 + r(i) * u1 - p(i) * ds1(i)) ...
        - m2 *a* ( -p(i) * w2 + r(i) * u2 - p(i) * ds2(i)) ...
        - (- m3 *b)* ( -p(i) * w3 + r(i) * u3 - p(i) * ds3(i)) ...
        - (- m4 *b)* ( -p(i) * w4 + r(i) * u4 - p(i) * ds4(i));
    
    Maa = [m+m1+m2+m3+m4,0,0,0,0,0;
        0,m+m1+m2+m3+m4,0,0,0,0;
        0,0,m1,0,0,0;
        0,0,0,m2,0,0;
        0,0,0,0,m3,0;
        0,0,0,0,0,m4];
    
    Mbb = [m,0,0,0;
        0,Ixx,0,-Ixz;
        0,0,Iyy,0;
        0,( -Ixz - m1 *a* zwc1- m2 *a* zwc2+ m3 *b * zwc3+m4 *b * zwc4),(m1 *cf/2 * zwc1- m2 *cf/2 * zwc2+ m3 *cr/2 * zwc3- m4 *cr/2 * zwc4),(Izz- m1 *cf/2* ywc1+ m2 *cf/2* ywc2 - m3 *cr/2* ywc3+ m4 *cr/2 * ywc4 + m1 *a* ywc1 + m2 *a * ywc2 - m3 *b* ywc3 - m4 *b * ywc4)];
    
    Mab = [0,0, (m1 * zwc1 + m2 * zwc2 + m3 * zwc3+ m4 * zwc4),(- m1 * ywc1 - m2 * ywc2 - m3 * ywc3- m4 * ywc4 );
        0,(- m1 * zwc1 - m2 * zwc2 - m3 * zwc3 - m4 * zwc4),0,( m1 * ywc1+ m2 * ywc2 + m3 * ywc3 + m4 * ywc4 );
        m1,(m1 * ywc1),(- m1 * xwc1),0;
        m2,(m2 * ywc2),(- m2 * xwc2),0;
        m3,(m3 * ywc3),(- m3 * xwc3),0;
        m4,(m4 * ywc4),(- m4 * xwc4),0];
    
    Mba = [0,0,0,0,0,0;
        0,0,0,0,0,0;
        0,0,0,0,0,0;
        (m1*cf/2 - m2*cf/2 + m3*cr/2 - m4*cr/2),(m1*a + m2*a - m3*b - m4*b),0,0,0,0];
    
    Faa = [FFx,FFy,FFz1,FFz2,FFz3,FFz4]';
    Fbb = [FFz,MMx,MMy,MMz]';
    
    dQb =  ( Mbb - Mba*(Maa\Mab) )\(Fbb - Mba*(Maa\Faa) );
    dQa = Maa\(Faa - Mab * dQb);
    
    %(m/s^2)
    du(i) = dQa(1);
    dv(i) = dQa(2);
    dds1(i) = dQa(3);
    dds2(i) = dQa(4);
    dds3(i) = dQa(5);
    dds4(i) = dQa(6);
    
    dw(i) = dQb(1);%(m/s2)
    dp(i) = dQb(2);%(rad/s^2)
    dq(i) = dQb(3);
    dr(i) = dQb(4);
    
    u(i+1) = u(i) + du(i)*step;
    v(i+1) = v(i) + dv(i)*step;
    w(i+1) = w(i) + dw(i)*step;
    p(i+1) = p(i) + dp(i)*step;
    q(i+1) = q(i) + dq(i)*step;
    r(i+1) = r(i) + dr(i)*step;
    
    ds1(i+1) = ds1(i) + dds1(i)*step; 
    ds2(i+1) = ds2(i) + dds2(i)*step; 
    ds3(i+1) = ds3(i) + dds3(i)*step; 
    ds4(i+1) = ds4(i) + dds4(i)*step; 

    s1(i+1) = s1(i) + ds1(i)*step;
    s2(i+1) = s2(i) + ds2(i)*step;
    s3(i+1) = s3(i) + ds3(i)*step;
    s4(i+1) = s4(i) + ds4(i)*step;
%% 状态变量输出结果汇总
VarP1 = zeros(32,1);

VarP1(1) = x(i+1);
VarP1(2) = y(i+1);
VarP1(3) = z(i+1);

VarP1(4) = yaw(i+1);
VarP1(5) = rll(i+1);
VarP1(6) = pth(i+1);

VarP1(7) = s1(i+1); 
VarP1(8) = s2(i+1); 
VarP1(9) = s3(i+1); 
VarP1(10) = s4(i+1); 

VarP1(11) = ds1(i+1);
VarP1(12) = ds2(i+1);
VarP1(13) = ds3(i+1);
VarP1(14) = ds4(i+1);

VarP1(15) = n1(i+1);
VarP1(16) = n2(i+1);
VarP1(17) = n3(i+1);
VarP1(18) = n4(i+1);

VarP1(19) = dn1(i+1);
VarP1(20) = dn2(i+1);
VarP1(21) = dn3(i+1);
VarP1(22) = dn4(i+1);

VarP1(23) = u(i+1);
VarP1(24) = v(i+1);
VarP1(25) = w(i+1);
VarP1(26) = p(i+1);
VarP1(27) = q(i+1);
VarP1(28) = r(i+1);

VarP1(29) = x_rack(i+1);
VarP1(30) = dx_rack(i+1);

VarP1(31) = deltaV(i+1);
VarP1(32) = ddeltaV(i+1);

%% 输出状态变量导数
dVarOut = zeros(32,1);

dVarOut(1) = dx(i);
dVarOut(2) = dy(i);
dVarOut(3) = dz(i);

dVarOut(4) = dyaw(i);
dVarOut(5) = drll(i);
dVarOut(6) = dpth(i);

dVarOut(7) = ds1(i); 
dVarOut(8) = ds2(i); 
dVarOut(9) = ds3(i); 
dVarOut(10) = ds4(i); 

dVarOut(11) = dds1(i);
dVarOut(12) = dds2(i);
dVarOut(13) = dds3(i);
dVarOut(14) = dds4(i);

dVarOut(15) = dn1(i);
dVarOut(16) = dn2(i);
dVarOut(17) = dn3(i);
dVarOut(18) = dn4(i);

dVarOut(19) = ddn1(i);
dVarOut(20) = ddn2(i);
dVarOut(21) = ddn3(i);
dVarOut(22) = ddn4(i);

dVarOut(23) = du(i);
dVarOut(24) = dv(i);
dVarOut(25) = dw(i);
dVarOut(26) = dp(i);
dVarOut(27) = dq(i);
dVarOut(28) = dr(i);

dVarOut(29) = dx_rack(i);
dVarOut(30) = ddx_rack(i);

dVarOut(31) = ddeltaV(i);
dVarOut(32) = dddeltaV(i);

%% 有价值的中间变量汇总输出
MidOut = zeros(120,1);

MidOut(1) = fs1;
MidOut(2) = fs2;
MidOut(3) = fs3;
MidOut(4) = fs4;

%(m)
MidOut(5) = Rd1;
MidOut(6) = Rd2;
MidOut(7) = Rd3;
MidOut(8) = Rd4;

%(N)
MidOut(9) = fwz1;
MidOut(10) = fwz2;
MidOut(11) = fwz3;
MidOut(12) = fwz4;

%(Nm)
MidOut(13) = Twy1;
MidOut(14) = Twy2;
MidOut(15) = Twy3;
MidOut(16) = Twy4;

%(_)
MidOut(17) = kappa1;
MidOut(18) = kappa2;
MidOut(19) = kappa3;
MidOut(20) = kappa4;

%(rad)
MidOut(21) = alpha1;
MidOut(22) = alpha2;
MidOut(23) = alpha3;
MidOut(24) = alpha4;

%(N)
MidOut(25) = fwx1;
MidOut(26) = fwx2;
MidOut(27) = fwx3;
MidOut(28) = fwx4;

%(N)
MidOut(29) = fwy1;
MidOut(30) = fwy2;
MidOut(31) = fwy3;
MidOut(32) = fwy4;

%(Nm)
MidOut(33) = Twz1;
MidOut(34) = Twz2;
MidOut(35) = Twz3;
MidOut(36) = Twz4;

%(N)
MidOut(37) = fx1;
MidOut(38) = fx2;
MidOut(39) = fx3;
MidOut(40) = fx4;

%(N)
MidOut(41) = fy1;
MidOut(42) = fy2;
MidOut(43) = fy3;
MidOut(44) = fy4;

%(N)
MidOut(45) = fz1;
MidOut(46) = fz2;
MidOut(47) = fz3;
MidOut(48) = fz4;

MidOut(49) = KXmove1;
MidOut(50) = KYmove1;

MidOut(51) = KXmove2;
MidOut(52) = KYmove2;

MidOut(53) = KXmove3;
MidOut(54) = KYmove3;

MidOut(55) = KXmove4;
MidOut(56) = KYmove4;

MidOut(57) = KToe1;
MidOut(58) = KToe2;
MidOut(59) = KToe3;
MidOut(60) = KToe4;

MidOut(61) = KDive1;
MidOut(62) = KDive2;
MidOut(63) = KDive3;
MidOut(64) = KDive4;

MidOut(65) = KCamber1;
MidOut(66) = KCamber2;
MidOut(67) = KCamber3;
MidOut(68) = KCamber4;

MidOut(69) = CToe1_Fx;
MidOut(70) = CToe2_Fx;
MidOut(71) = CToe3_Fx;
MidOut(72) = CToe4_Fx;

MidOut(73) = CSteer1_Fy;
MidOut(74) = CSteer2_Fy;
MidOut(75) = CSteer3_Fy;
MidOut(76) = CSteer4_Fy;

MidOut(77) = CSteer1_Mz;
MidOut(78) = CSteer2_Mz;
MidOut(79) = CSteer3_Mz;
MidOut(80) = CSteer4_Mz;

MidOut(81) = CCamber1_Fx;
MidOut(82) = CCamber2_Fx;
MidOut(83) = CCamber3_Fx;
MidOut(84) = CCamber4_Fx;

MidOut(85) = CInc1_Fy;
MidOut(86) = CInc2_Fy;
MidOut(87) = CInc3_Fy;
MidOut(88) = CInc4_Fy;

MidOut(89) = CInc1_Mz;
MidOut(90) = CInc2_Mz;
MidOut(91) = CInc3_Mz;
MidOut(92) = CInc4_Mz;

MidOut(93) = CXmove1;
MidOut(94) = CXmove2;
MidOut(95) = CXmove3;
MidOut(96) = CXmove4;

MidOut(97) = CYmove1;
MidOut(98) = CYmove2;
MidOut(99) = CYmove3;
MidOut(100) = CYmove4;

MidOut(101) = CDive1;
MidOut(102) = CDive2;
MidOut(103) = CDive3;
MidOut(104) = CDive4;

MidOut(105) = delta1(i);
MidOut(106) = delta2(i);
MidOut(107) = T_assist;

MidOut(108) = AngleAirR;
MidOut(109) = VairR;
MidOut(110) = FxAir;
MidOut(111) = FyAir;