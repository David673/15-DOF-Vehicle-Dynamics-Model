function [Fx,Fy,Mx,My,Mz]=PAC2002_forces_R( Fz , kappa , alpha , gamma )
% kappa:×ÝÏò»¬ÒÆÂÊ¡¾ÎÞÁ¿¸Ù¡¿
% alpha£º²àÆ«½Ç¡¾rad¡¿
% gamma£ºcamber ½Ç¡¾rad¡¿
%% info
% [MDI_HEADER]
% FILE_TYPE                ='tir'
% FILE_VERSION             =3.0
% FILE_FORMAT              ='ASCII'
% ! : TIRE_VERSION :      PAC2002
% ! : COMMENT :           Tire                    315/80 R22.5
% ! : COMMENT :           Manufacturer            
% ! : COMMENT :           Nom. section with  (m)  0.318       
% ! : COMMENT :           Nom. aspect ratio  (-)  0.80
% ! : COMMENT :           Infl. pressure    (Pa)  800000
% ! : COMMENT :           Rim diameter    (inch)  22.5        
% ! : COMMENT :           Measurement ID          
% ! : COMMENT :           Test speed       (m/s)  16.7        
% ! : COMMENT :           Road surface            
% ! : COMMENT :           Road condition          Dry
% ! : FILE_FORMAT :       ASCII
% ! : Copyright (C) 2004-2011 MSC Software Corporation
% !
% ! USE_MODE specifies the type of calculation performed:
% !       0: Fz only, no Magic Formula evaluation
% !       1: Fx,My only
% !       2: Fy,Mx,Mz only
% !       3: Fx,Fy,Mx,My,Mz uncombined force/moment calculation
% !       4: Fx,Fy,Mx,My,Mz combined force/moment calculation
% !     +10: including relaxation behaviour
% !     *-1: mirroring of tyre characteristics
% !
% !    example: USE_MODE = -12 implies:
% !       -calculation of Fy,Mx,Mz only
% !       -including relaxation effects
% !       -mirrored tyre characteristics
% !
% %----------------------------------------------------------------units
% [UNITS]
% LENGTH                   ='meter'
% FORCE                    ='newton'
% ANGLE                    ='radian'
% MASS                     ='kg'
% TIME                     ='second'
%% model
% %----------------------------------------------------------------model
% [MODEL]
%PROPERTY_FILE_FORMAT     ='PAC2002'
%USE_MODE                 = 4                   %%Tyre use switch (IUSED)
%VXLOW                    = 1                    
LONGVL                   = 16.5 ;                %%Measurement speed  
%TYRESIDE                 = 'LEFT'               %%Mounted side of tyre at vehicle/test bench
% ! 3D contact can be switched on by deleting the comment ! character
% ! When no further coefficients are specified, default values will be taken
% !CONTACT_MODEL            = '3D_ENVELOPING'
%% dimension
% %-----------------------------------------------------------dimensions
% [DIMENSION]
UNLOADED_RADIUS          = 0.298           ;       %%Free tyre radius         
WIDTH                    = 0.215         ;       %%Nominal section width of the tyre         
ASPECT_RATIO             = 0.8           ;       %%Nominal aspect ratio
RIM_RADIUS               = 0.28          ;       %%Nominal rim radius         
RIM_WIDTH                = 0.3           ;    %%Rim width         
%% shape
% %----------------------------------------------------------------shape
% [SHAPE]
% {radial width}
%  1.0    0.0
%  1.0    0.4
%  1.0    0.9
%  0.9    1.0
%% vertical
% %------------------------------------------------------------parameter
% [VERTICAL]
VERTICAL_STIFFNESS       = 3.6e+005     ;         % %Tyre vertical stiffness         
VERTICAL_DAMPING         = 500          ;        %%Tyre vertical damping         
BREFF                    = 3.5          ;        %%Low load stiffness e.r.r.         
DREFF                    = 0.5          ;        %%Peak value of e.r.r.         
FREFF                    = -0.005       ;        %%High load stiffness e.r.r.         
FNOMIN                   = 4100       ;        %%Nominal wheel load
%% slip range
%------------------------------------------------------long_slip_range
%[LONG_SLIP_RANGE]
KPUMIN                   = -1.5         ;        %Minimum valid wheel slip         
KPUMAX                   = 1.5          ;        %Maximum valid wheel slip         
%-----------------------------------------------------slip_angle_range
%[SLIP_ANGLE_RANGE]
ALPMIN                   = -1.5708      ;        %Minimum valid slip angle         
ALPMAX                   = 1.5708       ;        %Maximum valid slip angle         
%-----------------------------------------------inclination_slip_range
%[INCLINATION_ANGLE_RANGE]
CAMMIN                   = -0.26181     ;        %Minimum valid camber angle         
CAMMAX                   = 0.26181      ;        %Maximum valid camber angle         
%-------------------------------------------------vertical_force_range
%[VERTICAL_FORCE_RANGE]
FZMIN                    = 250          ;       %Minimum allowed wheel load         
FZMAX                    = 35000        ;        %Maximum allowed wheel load         
%% scaling coeff
%--------------------------------------------------------------scaling
%[SCALING_COEFFICIENTS]
LFZO                     = 1            ;        %Scale factor of nominal (rated) load         
LCX                      = 1            ;        %Scale factor of Fx shape factor         
LMUX                     = 1            ;        %Scale factor of Fx peak friction coefficient         
LEX                      = 1            ;        %Scale factor of Fx curvature factor         
LKX                      = 1            ;        %Scale factor of Fx slip stiffness         
LHX                      = 0            ;        %Scale factor of Fx horizontal shift         
LVX                      = 0            ;        %Scale factor of Fx vertical shift         
LGAX                     = 1            ;        %Scale factor of camber for Fx         
LCY                      = 1            ;        %Scale factor of Fy shape factor         
LMUY                     = 1            ;        %Scale factor of Fy peak friction coefficient         
% LEY                      = 1            ;        %Scale factor of Fy curvature factor         
LKY                      = 1         ;        %Scale factor of Fy cornering stiffness         
LHY                      = 0            ;        %Scale factor of Fy horizontal shift         
LVY                      = 0            ;        %Scale factor of Fy vertical shift         
LGAY                     = 1            ;        %Scale factor of camber for Fy         
LTR                      = 1            ;        %Scale factor of Peak of pneumatic trail         
% LRES                     = 0            ;        %Scale factor for offset of residual torque         
LGAZ                     = 1            ;        %Scale factor of camber for Mz         
LXAL                     = 1            ;        %Scale factor of alpha influence on Fx         
LYKA                     = 1            ;        %Scale factor of alpha influence on Fx         
LVYKA                    = 1            ;        %Scale factor of kappa induced Fy         
LS                       = 1            ;        %Scale factor of Moment arm of Fx         
% LSGKP                    = 1            ;        %Scale factor of Relaxation length of Fx         
% LSGAL                    = 1            ;        %Scale factor of Relaxation length of Fy         
% LGYR                     = 1            ;       %Scale factor of gyroscopic torque         
LMX                      = 1            ;        %Scale factor of overturning couple         
LVMX                     = 0            ;        %Scale factor of Mx vertical shift         
% LMY                      = 1            ;        %Scale factor of rolling resistance torque         
%% longitudinal coeff
%---------------------------------------------------------longitudinal
%[LONGITUDINAL_COEFFICIENTS]
PCX1                     = 1.62        ;       %Shape factor Cfx for longitudinal force         
PDX1                     = 1.035       ;       %Longitudinal friction Mux at Fznom         
PDX2                     = -0.0487      ;       %Variation of friction Mux with load         
PDX3                     = -2.27   ;       %Variation of friction Mux with camber         
PEX1                     = 0.5       ;       %Longitudinal curvature Efx at Fznom         
PEX2                     = -0.122         ;       %Variation of curvature Efx with load         
PEX3                     = -0.063      ;       %Variation of curvature Efx with load squared         
PEX4                     = 0   ;       %Factor in curvature Efx while driving         
PKX1                     = 19.4        ;       %Longitudinal slip stiffness Kfx/Fz at Fznom         
PKX2                     = -0.13       ;       %Variation of slip stiffness Kfx/Fz with load         
PKX3                     = 0.171       ;       %Exponent in slip stiffness Kfx/Fz with load         
PHX1                     = -0.0005   ;       %Horizontal shift Shx at Fznom         
PHX2                     = 8.42e-5   ;       %Variation of shift Shx with load         
PVX1                     = 0  ;       %Vertical shift Svx/Fz at Fznom         
PVX2                     = 0   ;       %Variation of shift Svx/Fz with load         
RBX1                     = 9.0         ;       %Slope factor for combined slip Fx reduction         
RBX2                     = -8.75       ;       %Variation of slope Fx reduction with kappa         
RCX1                     = 1.125       ;       %Shape factor for combined slip Fx reduction         
REX1                     = 0.078      ;       %Curvature factor of combined Fx         
REX2                     = -0.16     ;       %Curvature factor of combined Fx with load         
RHX1                     = 0.0045181     ;       %Shift factor for combined slip Fx reduction         
% PTX1                     = 1.5           ;       %Relaxation length SigKap0/Fz at Fznom         
% PTX2                     = 1.4           ;       %Variation of SigKap0/Fz with load         
% PTX3                     = 1             ;       %Variation of SigKap0/Fz with exponent of load         
% PTX4                     = 0.1           ;       %Low speed damping
%% overturning coeff
%----------------------------------------------------------overturning
%[OVERTURNING_COEFFICIENTS]
QSX1                     = 0             ;       %Lateral force induced overturning moment         
% QSX2                     = 0             ;       %Camber induced overturning couple         
QSX3                     = 0             ;       %Fy induced overturning couple         
%% lateral coeff
%--------------------------------------------------------------lateral
%[LATERAL_COEFFICIENTS]
PCY1                     = 1.29        ;       %Shape factor Cfy for lateral forces         
PDY1                     = -0.9       ;       %Lateral friction Muy         
PDY2                     = 0.18     ;       %Variation of friction Muy with load         
PDY3                     = -4.5       ;       %Variation of friction Muy with squared camber         
PEY1                     = -1.07       ;       %Lateral curvature Efy at Fznom         
PEY2                     = 0.68     ;       %Variation of curvature Efy with load         
PEY3                     = -0.63       ;       %Zero order camber dependency of curvature Efy         
PEY4                     = -12.35        ;       %Variation of curvature Efy with camber         
PKY1                     = -12.95       ;       %Maximum value of stiffness Kfy/Fznom         
PKY2                     = 1.72        ;       %Load at which Kfy reaches maximum value         
PKY3                     = 0.22      ;       %Variation of Kfy/Fznom with camber         
PHY1                     = 0.0035     ;       %Horizontal shift Shy at Fznom         
PHY2                     = -0.003    ;       %Variation of shift Shy with load         
PHY3                     = 0.045     ;       %Variation of shift Shy with camber         
PVY1                     = 0.045      ;       %Vertical shift in Svy/Fz at Fznom         
PVY2                     = -0.03     ;       %Variation of shift Svy/Fz with load         
PVY3                     = -0.174      ;       %Variation of shift Svy/Fz with camber         
PVY4                     = -0.45     ;       %Variation of shift Svy/Fz with camber and load         
RBY1                     = 6.38        ;       %Slope factor for combined Fy reduction         
RBY2                     = 7.95        ;       %Variation of slope Fy reduction with alpha         
RBY3                     = -0.06   ;       %Shift term for alpha in slope Fy reduction         
RCY1                     = 1.10          ;       %Shape factor for combined Fy reduction         
REY1                     = 0.23      ;       %Curvature factor of combined Fy         
REY2                     = 0.41   ;       %Curvature factor of combined Fy with load         
RHY1                     = 0.0007      ;       %Shift factor for combined Fy reduction         
RHY2                     = 0.024  ;       %Shift factor for combined Fy reduction with load         
RVY1                     = 0     ;       %Kappa induced side force Svyk/Muy*Fz at Fznom         
RVY2                     = 0     ;       %Variation of Svyk/Muy*Fz with load         
RVY3                     = 0      ;       %Variation of Svyk/Muy*Fz with camber         
RVY4                     = 0     ;       %Variation of Svyk/Muy*Fz with alpha         
RVY5                     = 1.95           ;       %Variation of Svyk/Muy*Fz with kappa         
RVY6                     = -50       ;       %Variation of Svyk/Muy*Fz with atan(kappa)         
% PTY1                     = 1.2           ;       %Peak value of relaxation length SigAlp0/R0         
% PTY2                     = 2.5           ;       %Value of Fz/Fznom where SigAlp0 is extreme         
%% rolling resistance coeff
%---------------------------------------------------rolling resistance
%[ROLLING_COEFFICIENTS]
% QSY1                     = 0.008         ;       %Rolling resistance torque coefficient         
% QSY2                     = 0             ;       %Rolling resistance torque depending on Fx         
% QSY3                     = 0             ;       %Rolling resistance torque depending on speed         
% QSY4                     = 0             ;       %Rolling resistance torque depending on speed ^4         
%% aligning coeff
%-------------------------------------------------------------aligning
%[ALIGNING_COEFFICIENTS]
QBZ1                     = 8.37        ;       %Trail slope factor for trail Bpt at Fznom         
QBZ2                     = -2.92       ;       %Variation of slope Bpt with load         
QBZ3                     = 0.48       ;       %Variation of slope Bpt with load squared         
QBZ4                     = -0.45       ;       %Variation of slope Bpt with camber         
QBZ5                     = -0.44      ;       %Variation of slope Bpt with absolute camber         
QBZ9                     = 3.44        ;       %Slope factor Br of residual torque Mzr         
QBZ10                    = 0             ;       %Slope factor Br of residual torque Mzr         
QCZ1                     = 1.19        ;       %Shape factor Cpt for pneumatic trail         
QDZ1                     = 0.11      ;       %Peak trail Dpt" = Dpt*(Fz/Fznom*R0)         
QDZ2                     = -0.003     ;       %Variation of peak Dpt" with load         
QDZ3                     = -0.56       ;       %Variation of peak Dpt" with camber         
QDZ4                     = 8.40       ;       %Variation of peak Dpt" with camber squared         
QDZ6                     = -0.003    ;       %Peak residual torque Dmr" = Dmr/(Fz*R0)         
QDZ7                     = 0.005     ;       %Variation of peak factor Dmr" with load         
QDZ8                     = -0.12     ;       %Variation of peak factor Dmr" with camber         
QDZ9                     = 0.12      ;       %Variation of peak factor Dmr" with camber and load         
QEZ1                     = -2.9    ;       %Trail curvature Ept at Fznom         
QEZ2                     = -0.55     ;       %Variation of curvature Ept with load         
QEZ3                     = 0             ;       %Variation of curvature Ept with load squared         
QEZ4                     = -0.13        ;       %Variation of curvature Ept with sign of Alpha-t         
QEZ5                     = -3.68       ;       %Variation of Ept with camber and sign Alpha-t         
QHZ1                     = 0.003     ;       %Trail horizontal shift Sht at Fznom         
QHZ2                     = 0.00082    ;       %Variation of shift Sht with load         
QHZ3                     = 0.154       ;       %Variation of shift Sht with camber         
QHZ4                     = 0.12      ;       %Variation of shift Sht with camber and load         
SSZ1                     = 0.025     ;       %Nominal value of s/R0: effect of Fx on Mz         
SSZ2                     = 0      ;       %Variation of distance s/R0 with Fy/Fznom         
SSZ3                     = 0.5       ;       %Variation of distance s/R0 with camber         
SSZ4                     = -0.27      ;       %Variation of distance s/R0 with load and camber         
% QTZ1                     = 0             ;       %Gyration torque constant         
% MBELT                    = 0             ;       %Belt mass of the wheel   
%% pressure coeff
pi = 1;
pi0 = 1;
%----------
LIP = 1;
PPX1 = 1;
PPX2 = 1;
PPX3 = 1;
PPX4 = 1;
PPY3 = 1;
PPY4 = 1;
PPY1 = 1;
PPY2 = 1;
QPZ1 = 1;
QPZ2 = 1;
%
%% parking
e0 = 1;
e1 = 1;
e2 = 1;
e3 = 1;
e4 = 1;
e5 = 1;
e6 = 1;
% e7 = 1;
e8 = 1;
%
%% longitudinal force at pure slip Fx0

dfz = ( Fz - FNOMIN * LFZO ) / ( FNOMIN * LFZO );

dpi = ( pi - pi0 * LIP ) / ( pi0 * LIP );

Shx = ( PHX1 + PHX2 * dfz ) * LHX;
kappax = kappa + Shx; %--------------------------------key
Svx = Fz * ( PVX1 + PVX2 * dfz ) * LVX * LMUX * e1;
Kx = Fz * ( PKX1 + PKX2 * dfz ) * exp( PKX3 * dfz ) * ( 1 + PPX1 * dpi + PPX2 * dpi^2 ) * LKX;
Cx = PCX1 * LCX;
gammax = gamma * LGAX;
mux = ( PDX1 + PDX2 * dfz ) * ( 1 + PPX3 * dpi + PPX4 * dpi^2 ) * ( 1 - PDX3 * gammax^2 ) * LMUX;
Dx = mux * Fz * e1;
Ex = ( PEX1 + PEX2 * dfz + PEX3 * dfz^2 ) * ( 1 + PEX4 * sign( kappax ) ) * LEX;
Bx = Kx / ( Cx * Dx );

%--------------------------------------------------------------------------------------------
Fx0 = Dx * sin( Cx * atan( Bx * kappax - Ex * ( Bx * kappax - atan( Bx * kappax ) ) ) ) + Svx;
%Fx = Fx0;

%% lateral force at pure slip Fy0

gammay = gamma * LGAY;
Shy = ( PHY1 + PHY2 * dfz ) * LHY + PHY3 * gammay * e0 + e4 - 1;
alphay = alpha + Shy;
Cy = PCY1 * LCY;
muy = ( PDY1 + PDY2 * dfz ) * ( 1 + PPY3 * dpi + PPY4 * dpi^2 ) * ( 1 + PDY3 * gammay^2 ) * LMUY;
Dy = muy * Fz * e2;
Ey = ( PEY1 + PEY2 * dfz ) * ( 1 - ( PEY3 + PEY4 * gammay ) * sign ( alphay ) );
Ky0 = PKY1 * FNOMIN * ( 1 + PPY1 * dpi ) * sin( 2 * atan( Fz / ( PKY2 * FNOMIN * LFZO * ( 1 + PPY2 * dpi ) ) ) ) * LFZO * LKY;
Ky = Ky0 * ( 1 - PKY3 * abs( gammay ) ) * e3;
By = Ky / ( Cy * Dy );
Svy = Fz * ( ( PVY1 + PVY2 * dfz ) * LVY + ( PVY3 + PVY4 * dfz ) * gammay ) * LMUY * e2;           %LKYG is missing
Fy0 = Dy * sin( Cy * atan ( By * alphay - Ey * ( By * alphay - atan( By * alphay ) ) ) ) + Svy;
%Fy = Fy0;
%% aligning moment at pure slip Mz0

gammaz = gamma * LGAZ;
Br = ( QBZ9 * ( LKY / LMUY ) + QBZ10 * By * Cy ) * e6;
% Cr = e7;
Dr = Fz * ( ( QDZ6 + QDZ7 * dfz ) * 1 + ( QDZ8 + QDZ9 * dfz ) * ( 1 + QPZ2 * dpi ) * gammaz ) * UNLOADED_RADIUS * LMUY + e8 - 1;
Sht = QHZ1 + QHZ2 * dfz + ( QHZ3 + QHZ4 * dfz ) * gammaz;
Dt = Fz * ( QDZ1 + QDZ2 * dfz ) * ( 1 - QPZ1 * dpi ) * ( 1 + QDZ3 * gammaz + QDZ4 * gammaz^2 ) * ( UNLOADED_RADIUS / ( FNOMIN * LFZO ) ) * LTR * e5;
Ct = QCZ1;
Bt = ( QBZ1 + QBZ2 * dfz + QBZ3 * dfz^2 ) * ( 1 + QBZ4 * gammaz + QBZ5 * abs( gammaz ) ) * LKY / LMUY;
Shf = Shy + Svy / Ky;
alphar = alpha + Shf;
% Mzr = Dr * cos( Cr * atan( Br * alphar ) ) * cos( alpha );
alphat = alpha + Sht;
Et = ( QEZ1 + QEZ2 * dfz + QEZ3 * dfz^2 ) * ( 1 + ( QEZ4 + QEZ5 * gammaz ) * ( ( 2 / pi ) * atan( Bt * Ct * alphat ) ) );
% t = Dt * cos( Ct * atan( Bt * alphat - Et * ( Bt * alphat - atan( Bt * alphat ) ) ) ) * cos( alpha );

% Mz0 = - t * Fy0 + Mzr;
%Mz = Mz0;
%% formulas for the longitudinal force at combined slip Fxc

Shxa = RHX1;
Exa = REX1 + REX2 * dfz;
Cxa = RCX1;
Bxa = RBX1 * cos( atan( RBX2 * kappa ) ) * LXAL;
alphas = alpha + Shxa;
Gka = ( cos( Cxa * atan( Bxa * alphas - Exa * ( Bxa * alphas - atan( Bxa * alphas ) ) ) ) ) / ( cos( Cxa * atan( Bxa * Shxa - Exa * ( Bxa * Shxa - atan( Bxa * Shxa ) ) ) ) );

Fxc = Fx0 * Gka;
Fx = Fxc;
%% formulas for the lateral force at combined slip Fyc

Dvyk = muy * Fz * ( RVY1 + RVY2 * dfz + RVY3 * gamma ) * cos( atan( RVY4 * alpha ) );
Svyk = Dvyk * sin( RVY5 * atan( RVY6 * kappa ) ) * LVYKA;
Shyk = RHY1 + RHY2 * dfz;
Eyk = REY1 + REY2 * dfz;
Cyk = RCY1;
Byk = RBY1 * cos( atan( RBY2 * ( alpha - RBY3 ) ) ) * LYKA;
kappas = kappa + Shyk;
Gyk = ( cos( Cyk * atan( Byk * kappas - Eyk * ( Byk * kappas - atan( Byk * kappas ) ) ) ) ) / ( cos( Cyk * atan( Byk * Shyk - Eyk * ( Byk * Shyk - atan( Byk * Shyk ) ) ) ) );

Fyc = Fy0 * Gyk + Svyk;
Fy = Fyc;
%% formulas for the aligning moment at combined slip Mzc

s = ( SSZ1 + SSZ2 * Fyc / ( FNOMIN * LFZO ) + ( SSZ3 + SSZ4 * dfz ) * gamma ) * UNLOADED_RADIUS * LS;
alphar_eq = atan( sqrt( tan( alphar ) * tan( alphar ) + ( Kx / Ky )^2 * kappa^2 ) ) * sign( alphar );
Mzr = Dr * cos( atan( Br * alphar_eq ) ) * cos( alpha );
alphat_eq = atan( sqrt( tan( alphat ) * tan( alphat ) + ( Kx / Ky )^2 * kappa^2 ) ) * sign( alphat );
t = Dt * cos( Ct * atan( Bt * alphat_eq - Et * ( Bt * alphat_eq - atan( Bt * alphat_eq ) ) ) ) * cos( alpha ); 

Mzc = - t * Fy0 * Gyk + Mzr - s * Fxc;
Mz = Mzc;
%% formulas for the overturning moment at pure and combined slip Mx0

Mx0 = UNLOADED_RADIUS * Fz * ( QSX3 * Fyc / ( FNOMIN * LFZO ) + QSX1 * LVMX ) * LMX;
Mx = Mx0;
%% formulas for the rolling resistance moment at pure and combined slip My0

My0 = UNLOADED_RADIUS * ( Svx + Kx * Shx );
My = My0;
