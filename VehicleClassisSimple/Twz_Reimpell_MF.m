function Twz = Twz_Reimpell_MF(~,Fwy,Fwz,Inc,Caster,Lat_offset_C,X_cord_C,delta,MzMF, Rd)
%输出为【Nm】
%Fwx,轮胎系下纵向力【N】
%Fwy,轮胎系下侧向力【N】
%Fwz,轮胎系下垂向力【N】
%Inc,主销内倾角，含KC特性引起的【rad】
%Caster,主销后倾角，含KC特性引起的【rad】
%Lat_offset_C,轮心与主销中心点的偏移距离y向,应该是个相对位置，不受KC影响【m】
%X_cord_C,轮心与主销中心点的偏移距离x向，应该是个相对位置，不受KC影响【m】
%delta,轮胎转角【rad】
%width，轮胎拖距【m】
%Rd,车轮动态半径【m】
%Tir_lat_offset轮胎侧向偏距【m】
%【注意】轮胎纵向拖距【width】与侧向偏距【Tir_lat_offset】均随轮胎侧偏角的变化而变化，第一版暂时认为其为常数，在后期更新中进行修正
%% 
%垂向力引起的回正力矩，fz始终大于等于0，方向与车身系z向相同，当车轮右转【正向】，产生的回正力矩方向在车身系下为【负值】。
rn = Lat_offset_C * cos(Inc);
M_fz = - Fwz * sin(Inc) * sin(delta) * rn;

%侧向力引起的回正力矩【方向存疑】fy为正（向右）时，回正力矩与绕z轴正方向相反，为负
M_fy = - Fwy * cos(Inc) * ( Rd * tan(Caster) + X_cord_C);

%总回正力矩
Twz = M_fz + M_fy + MzMF*0;

%公式详见【德】Reimpell J. 所著《汽车底盘基础》