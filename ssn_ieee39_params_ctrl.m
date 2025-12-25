f=60;
fNom=f;
Rk_def=0.1;
Lk_def=0.1e-3;
Ts_PMU=1/1000;

addRs=1;  % set to 1 to add a little stator resistance
Rsadd=1e-3;
J = 3.3e5;


%%%%%%  machine 1  %%%%%%%
MVA=10000e6;
Vb=500e3;
Ifd_nom=3300;   %nominal field current
pp=1;  % number pair of poles

Ra= 0.0000000000E+00  ;
Xl = 0.7500000000E+01;  
X0 = 0.7500000000E+01 ; 

Xd = 0.5000000000E+02 ; 
Rf = 0.2485506629E+00  ;
Xf = 0.6559097468E+03 ; 

Xq = 0.4750000000E+02  ;
RQ1 = 0.2802156577E+01 ; 
XQ1 = 0.7394716995E+03 ; 

% transformation into SSN machine parameters
MVA1=MVA;
Vb1=Vb;
Ifn1=Ifd_nom;
pp1=pp;
    

Rs1=Ra;
Ll1=Xl/2/pi/f;
Lmd1=(Xd-Xl)/2/pi/f;
Lmq1=(Xq-Xl)/2/pi/f;
Rf1=Rf;
%Ns_on_Nf=Lmd*I_field_nominal*wbase/Vbase;   stator/field turn ratio
Ns_on_Nf=Lmd1*Ifn1*2*pi*f/Vb1;  % stator/field turn ratio
Rf_prime1=Rf*Ns_on_Nf*Ns_on_Nf;
Lf_prime1=Xf/2/pi/f*Ns_on_Nf*Ns_on_Nf;
Rkq1_prime1=RQ1*Ns_on_Nf*Ns_on_Nf;
Lkq1_prime1=XQ1/2/pi/f*Ns_on_Nf*Ns_on_Nf;
X0_1=X0;
angle1=-41.4;


%%%%%%  machine 2  %%%%%%%
MVA=1000e6;
Vb=20e3;
Ifd_nom=2200;   %nominal field current
pp=1;  % number pair of poles



Ra = 0.0000000000E+00 ; 
Xl = 0.1400000000E+00 ; 
X0 = 0.1400000000E+00 ; 

Xd =0.1180000000E+01 ; 
Rf = 0.3708160202E-01  ;
Xf = 0.9170509110E+02  ;

Xq =0.1128000000E+01  ;
RQ1 = 0.2944160274E+00 ; 
XQ1 =0.1664883412E+03  ;

% transformation into SSN machine parameters
MVA2=MVA;
Vb2=Vb;
Ifn2=Ifd_nom;
pp2=pp;

Rs2=Ra;
Ll2=Xl/2/pi/f;
Lmd2=(Xd-Xl)/2/pi/f;
Lmq2=(Xq-Xl)/2/pi/f;
Rf2=Rf;
%Ns_on_Nf=Lmd*I_field_nominal*wbase/Vbase;   stator/field turn ratio
Ns_on_Nf=Lmd2*Ifn2*2*pi*f/Vb2;  % stator/field turn ratio
Rf_prime2=Rf*Ns_on_Nf*Ns_on_Nf;
Lf_prime2=Xf/2/pi/f*Ns_on_Nf*Ns_on_Nf;
Rkq1_prime2=RQ1*Ns_on_Nf*Ns_on_Nf;
Lkq1_prime2=XQ1/2/pi/f*Ns_on_Nf*Ns_on_Nf;
X0_2=X0;
angle2=-30;




%%%%%%  machine 3  %%%%%%%
MVA=1000e6;
Vb=20e3;
Ifd_nom=2200;   %nominal field current
pp=1;  % number pair of poles

% rotor and feild values NOT REFERED to stator here
Ra=0.0000000000E+00;  
Xl=0.1216000000E+00 ; 
X0=0.1216000000E+00 ; 

Xd=0.9980000000E+00  ;
Rf =0.4895614140E-01  ;
Xf=0.1051993739E+03 ;

Xq=0.9480000000E+00  ;
RQ1=0.2174490639E+00  ;
XQ1=0.1229645487E+03 ;

% transformation into SSN machine parameters
MVA3=MVA;
Vb3=Vb;
Ifn3=Ifd_nom;
pp3=pp;

Rs3=Ra;
Ll3=Xl/2/pi/f;
Lmd3=(Xd-Xl)/2/pi/f;
Lmq3=(Xq-Xl)/2/pi/f;
Rf3=Rf;
%Ns_on_Nf=Lmd*I_field_nominal*wbase/Vbase;   stator/field turn ratio
Ns_on_Nf=Lmd3*Ifn3*2*pi*f/Vb3;  % stator/field turn ratio
Rf_prime3=Rf*Ns_on_Nf*Ns_on_Nf;
Lf_prime3=Xf/2/pi/f*Ns_on_Nf*Ns_on_Nf;
Rkq1_prime3=RQ1*Ns_on_Nf*Ns_on_Nf;
Lkq1_prime3=XQ1/2/pi/f*Ns_on_Nf*Ns_on_Nf;
X0_3=X0;
angle3=-28.3;

%%%%%%  machine 4  %%%%%%%
MVA=1000e6;
Vb=20e3;
Ifd_nom=2200;   %nominal field current
pp=1;  % number pair of poles

Ra =0.0000000000E+00 ; 
Xl = 0.1180000000E+00  ;
X0 = 0.1180000000E+00 ; 

Xd =0.1048000000E+01 ; 
Rf =0.4410203394E-01  ;
Xf = 0.9460236733E+02  ;

Xq = 0.1032000000E+01  ;
RQ1= 0.3835932245E+00  ;
XQ1 = 0.2169168581E+03  ;

% transformation into SSN machine parameters
MVA4=MVA;
Vb4=Vb;
Ifn4=Ifd_nom;
pp4=pp;

Rs4=Ra;
Ll4=Xl/2/pi/f;
Lmd4=(Xd-Xl)/2/pi/f;
Lmq4=(Xq-Xl)/2/pi/f;
Rf4=Rf;
%Ns_on_Nf=Lmd*I_field_nominal*wbase/Vbase;   stator/field turn ratio
Ns_on_Nf=Lmd4*Ifn4*2*pi*f/Vb4;  % stator/field turn ratio
Rf_prime4=Rf*Ns_on_Nf*Ns_on_Nf;
Lf_prime4=Xf/2/pi/f*Ns_on_Nf*Ns_on_Nf;
Rkq1_prime4=RQ1*Ns_on_Nf*Ns_on_Nf;
Lkq1_prime4=XQ1/2/pi/f*Ns_on_Nf*Ns_on_Nf;
X0_4=X0;
angle4=-27.1;



%%%%%%  machine 5  %%%%%%%
MVA=600e6;
Vb=20e3;
Ifd_nom=3300;   %nominal field current
pp=1;  % number pair of poles

Ra=0.0000000000E+00 ; 
Xl=0.2160000000E+00  ;
X0 =0.2160000000E+00 ; 

Xd = 0.2666666667E+01 ; 
Rf =0.8436535386E-02 ; 
Xf =0.1717469412E+02 ; 

Xq =0.2466666667E+01  ;
RQ1 = 0.1037599328E+00  ;
XQ1= 0.1721129217E+02 ;
% transformation into SSN machine parameters
MVA5=MVA;
Vb5=Vb;
Ifn5=Ifd_nom;
pp5=pp;

Rs5=Ra;
Ll5=Xl/2/pi/f;
Lmd5=(Xd-Xl)/2/pi/f;
Lmq5=(Xq-Xl)/2/pi/f;
Rf5=Rf;
%Ns_on_Nf=Lmd*I_field_nominal*wbase/Vbase;   stator/field turn ratio
Ns_on_Nf=Lmd5*Ifn5*2*pi*f/Vb5;  % stator/field turn ratio
Rf_prime5=Rf*Ns_on_Nf*Ns_on_Nf;
Lf_prime5=Xf/2/pi/f*Ns_on_Nf*Ns_on_Nf;
Rkq1_prime5=RQ1*Ns_on_Nf*Ns_on_Nf;
Lkq1_prime5=XQ1/2/pi/f*Ns_on_Nf*Ns_on_Nf;
X0_5=X0;
angle5=-28;

%%%%%%  machine 6  %%%%%%%
MVA=1000e6;
Vb=20e3;
Ifd_nom=2200;   %nominal field current
pp=1;  % number pair of poles
Ra= 0.0000000000E+00  ;
Xl=0.8960000000E-01  ;
X0 =0.8960000000E-01 ; 

Xd = 0.1016000000E+01  ;
Rf = 0.3680192128E-01  ;
Xf = 0.1012801815E+03 ; 

Xq =0.9640000000E+00  ;
RQ1 = 0.7648105083E+00 ; 
XQ1 = 0.1153307076E+03  ;

% transformation into SSN machine parameters
MVA6=MVA;
Vb6=Vb;
Ifn6=Ifd_nom;
pp6=pp;

Rs6=Ra;
Ll6=Xl/2/pi/f;
Lmd6=(Xd-Xl)/2/pi/f;
Lmq6=(Xq-Xl)/2/pi/f;
Rf6=Rf;
%Ns_on_Nf=Lmd*I_field_nominal*wbase/Vbase;   stator/field turn ratio
Ns_on_Nf=Lmd6*Ifn6*2*pi*f/Vb6;  % stator/field turn ratio
Rf_prime6=Rf*Ns_on_Nf*Ns_on_Nf;
Lf_prime6=Xf/2/pi/f*Ns_on_Nf*Ns_on_Nf;
Rkq1_prime6=RQ1*Ns_on_Nf*Ns_on_Nf;
Lkq1_prime6=XQ1/2/pi/f*Ns_on_Nf*Ns_on_Nf;
X0_6=X0;
angle6=-25.7;


%%%%%%  machine 7  %%%%%%%
MVA=1000e6;
Vb=20e3;
Ifd_nom=2200;   %nominal field current
pp=1;  % number pair of poles

Ra =0.0000000000E+00  ;
Xl =0.1288000000E+00  ;
X0 =0.1288000000E+00  ;
Xd = 0.1180000000E+01  ;
Rf= 0.3936153091E-01  ;
Xf =0.8398844319E+02  ;
Xq=0.1168000000E+01  ;
RQ1 = 0.3368635080E+00  ;
XQ1 = 0.1904918260E+03  ;
% transformation into SSN machine parameters
MVA7=MVA;
Vb7=Vb;
Ifn7=Ifd_nom;
pp7=pp;

Rs7=Ra;
Ll7=Xl/2/pi/f;
Lmd7=(Xd-Xl)/2/pi/f;
Lmq7=(Xq-Xl)/2/pi/f;
Rf7=Rf;
%Ns_on_Nf=Lmd*I_field_nominal*wbase/Vbase;   stator/field turn ratio
Ns_on_Nf=Lmd7*Ifn7*2*pi*f/Vb7;  % stator/field turn ratio
Rf_prime7=Rf*Ns_on_Nf*Ns_on_Nf;
Lf_prime7=Xf/2/pi/f*Ns_on_Nf*Ns_on_Nf;
Rkq1_prime7=RQ1*Ns_on_Nf*Ns_on_Nf;
Lkq1_prime7=XQ1/2/pi/f*Ns_on_Nf*Ns_on_Nf;
X0_7=X0;
angle7=-23;





%%%%%%  machine 8  %%%%%%%
MVA8=1000e6;
Vb8=20e3;
Ibase8=MVA8/Vb8;
Ifd_nom8=2200;   %nominal field current
pp=1;  % number pair of poles



Ra =0.0000000000E+00  ;
Xl = 0.1120000000E+00  ;
X0 = 0.1120000000E+00  ;

Xd =0.1160000000E+01  ;
Rf =0.3510692536E-01  ;
Xf = 0.8867449367E+02;

Xq = 0.1120000000E+01  ;
RQ1 = 0.6546455256E+00  ;
XQ1 = 0.1011861750E+03 ; 
% transformation into SSN machine parameters
MVA8=MVA;
Vb8=Vb;
Ifn8=Ifd_nom;
pp8=pp;

Rs8=Ra;
Ll8=Xl/2/pi/f;
Lmd8=(Xd-Xl)/2/pi/f;
Lmq8=(Xq-Xl)/2/pi/f;
Rf8=Rf;
%Ns_on_Nf=Lmd*I_field_nominal*wbase/Vbase;   stator/field turn ratio
Ns_on_Nf=Lmd8*Ifn8*2*pi*f/Vb8;  % stator/field turn ratio
Rf_prime8=Rf*Ns_on_Nf*Ns_on_Nf;
Lf_prime8=Xf/2/pi/f*Ns_on_Nf*Ns_on_Nf;
Rkq1_prime8=RQ1*Ns_on_Nf*Ns_on_Nf;
Lkq1_prime8=XQ1/2/pi/f*Ns_on_Nf*Ns_on_Nf;
X0_8=X0;
angle8=-29;


%%%%%%  machine 9  %%%%%%%
MVA=1000e6;
Vb=20e3;
Ifd_nom=2200;   %nominal field current
pp=1;  % number pair of poles


Ra =0.0000000000E+00  ;
Xl = 0.1192000000E+00  ;
X0 = 0.1192000000E+00  ;

Xd = 0.8424000000E+00  ;
Rf =0.7448979696E-01  ;
Xf = 0.1345127410E+03  ;

Xq =0.8200000000E+00 ; 
RQ1 = 0.1794710789E+00 ; 
XQ1 = 0.1326116454E+03 ; 

% transformation into SSN machine parameters
MVA9=MVA;
Vb9=Vb;
Ifn9=Ifd_nom;
pp9=pp;

Rs9=Ra;
Ll9=Xl/2/pi/f;
Lmd9=(Xd-Xl)/2/pi/f;
Lmq9=(Xq-Xl)/2/pi/f;
Rf9=Rf;
%Ns_on_Nf=Lmd*I_field_nominal*wbase/Vbase;   stator/field turn ratio
Ns_on_Nf=Lmd9*Ifn9*2*pi*f/Vb9;  % stator/field turn ratio
Rf_prime9=Rf*Ns_on_Nf*Ns_on_Nf;
Lf_prime9=Xf/2/pi/f*Ns_on_Nf*Ns_on_Nf;
Rkq1_prime9=RQ1*Ns_on_Nf*Ns_on_Nf;
Lkq1_prime9=XQ1/2/pi/f*Ns_on_Nf*Ns_on_Nf;
X0_9=X0;
angle9=-23.6;



%%%%%%  machine 10 %%%%%%%
MVA=1000e6;
Vb=20e3;
Ifd_nom=660;   %nominal field current
pp=1;  % number pair of poles

Ra = 0.0000000000E+00 ; 
Xl = 0.5000000000E-01 ; 
X0 = 0.5000000000E-01 ; 

Xd =0.4000000000E+00  ;
Rf =0.8652303554E+00  ;
Xf = 0.3327078426E+04  ;

Xq =0.2760000000E+00  ;
RQ1 = 0.7359405020E+06 ; 
XQ1 = 0.2774430329E+05 ;

% transformation into SSN machine parameters
MVA10=MVA;
Vb10=Vb;
Ifn10=Ifd_nom;
pp10=pp;

Rs10=Ra;
Ll10=Xl/2/pi/f;
Lmd10=(Xd-Xl)/2/pi/f;
Lmq10=(Xq-Xl)/2/pi/f;
Rf10=Rf;
%Ns_on_Nf=Lmd*I_field_nominal*wbase/Vbase;   stator/field turn ratio
Ns_on_Nf=Lmd10*Ifn10*2*pi*f/Vb10;  % stator/field turn ratio
Rf_prime10=Rf*Ns_on_Nf*Ns_on_Nf;
Lf_prime10=Xf/2/pi/f*Ns_on_Nf*Ns_on_Nf;
Rkq1_prime10=RQ1*Ns_on_Nf*Ns_on_Nf;
Lkq1_prime10=XQ1/2/pi/f*Ns_on_Nf*Ns_on_Nf;
X0_10=X0;
angle10=-34.7;

if addRs==1
   Rs1= Rsadd;
   Rs2= Rsadd;
   Rs3= Rsadd;
   Rs4= Rsadd;
   Rs5= Rsadd;
   Rs6= Rsadd;
   Rs7= Rsadd;
   Rs8= Rsadd;
   Rs9= Rsadd;
   Rs10= Rsadd;
   
end

% %VF_EMTP=[826,205,182,180,110,110,100,110,255, 760];  % Vfd from EMTP-Rv (final values of 20 sec of IEEE 39 bus dmeo)
% VF_EMTP=[991.4298251 197.162164	236.9789139	199.2338553	98.29405836	154.1188044	131.1300505	127.0398928	264.4473764	714.6684749]; % Vfd from EMTP-Rv (final values of 30 sec of IEEE 39 bus demo)
% 
% Vf_nom=[Rf1*Ifn1,Rf2*Ifn2,Rf3*Ifn3,Rf4*Ifn4,Rf5*Ifn5,Rf6*Ifn6,Rf7*Ifn7,Rf8*Ifn8,Rf9*Ifn9,Rf10*Ifn10];
% VF_FACTOR=VF_EMTP./Vf_nom;
% 
% % these angle are taken frmo EMTP-RV
% %angle=[ angle1 angle2 angle3 angle4 angle5 angle6 angle7 angle8 angle9 angle10];
% 
% %angle= [-5.0900e+00   0  -2.6200e+00   4.4600e+00   1.2540e+01   1.0170e+01  3.4850e+01   1.1260e+01   1.2230e+01 -1.6830e+01]
% angle=[-53963.68113	-53958.59064	-53961.21902	-53954.13767	-53946.05201	-53948.42669	-53923.74716	-53947.33873	-53946.36021	-53975.4248];
% angle=angle+53975.4248; %put machine 10 at 0 deg
% 
% angle=[100.8171873	124.2561131	122.1205425	128.0133774	142.8496361	131.1516957	129.6452201	127.7771774	135.8022751	102.6895952];  % taken at T=0!
% 
% angle=[-1617.098424	-1645.226808	-1647.832017	-1649.357691	-1637.939548	-1645.35901	-1644.83535	-1644.36456	-1640.047007	-1658.842347];
% angle=angle-min(angle);





s=10; % for transformer PU value in 1000MVA base s=10;
PSSMODEL=0; % not used yet

 
% PSSE load flow. not working so good...
%INITIAL CONDITION LOAD FLOW USED  1 ITERATIONS
%----------------------------- MACHINE INITIAL CONDITIONS -----------------------------
%   BUS#           BASKV ID  ETERM   EFD    POWER    VARS   P.F.  ANGLE   ID     IQ
PSSEINIT=[
     39           1.0000 1  1.0300 1.0382 1042.52  410.96 0.9303  -9.84 0.4186 1.0042;     %%%the big machine is G1 bus 39  this row is last in PSSe
     31           1.0000 1  1.0400 3.9283  572.93  714.66 0.6255  22.72 0.8404 0.2634;
     32           1.0000 1  0.9831 3.4398  650.00  532.93 0.7733  31.96 0.7971 0.3093;
     33           1.0000 1  0.9972 2.8713  632.00  318.58 0.8930   5.37 0.6197 0.3461;
     34           1.0000 1  1.0123 2.1251  508.00  111.20 0.9769  33.22 0.3680 0.3584;
     35           1.0000 1  1.0493 2.9995  650.00  327.20 0.8932  35.83 0.5949 0.3564;
     36           1.0000 1  1.0635 2.7172  560.00  251.05 0.9125  36.40 0.4682 0.3373;
     37           1.0000 1  1.0278 2.1824  540.00   71.37 0.9914  37.69 0.3731 0.3764;
     38           1.0000 1  1.0265 2.6367  830.00  167.86 0.9802  48.49 0.6654 0.4876;
     30           1.0000 1  1.0475 1.9386  250.00  247.21 0.7111   1.15 0.2612 0.2108;];   % G10 bus 30 :this row is first in PSSe


  
% take PSSE load flow
% VF_FACTORx=PSSEINIT(:,5)';
% anglex=PSSEINIT(:,9)';

% transformer taps

taps=[1 1 1.07 1.07 1.009 1.025 1 1.025 1.025 1.025]*1;

disp('Load flow from SPS model NE39bus2.slx on matlablab central (with a load on bus6 like in IEEE spec, not bus7...)');
VF_FACTOR=[1.11192 2.17666 2.13908 2.00969  3.88471   2.2539   2.24212  1.86611 2.04131  1.23074];
angle=[-61.2872 -41.8184 -38.2055 -30.4068 -27.8882 -41.2014  -38.0249 -34.2544 -26.563 -85.8164];
angle(1)=angle(1)-30; % added D-Y xfo from SPS load flow

Bus = [ ...
     1   345
     2   345
     3   345
     4   345
     5   345
     6   345
     7   345
     8   345
     9   345
    10   345
    11   345
    12   230
    13   345
    14   345
    15   345
    16   345
    17   345
    18   345
    19   345
    20   345
    21   345
    22   345
    23   345
    24   345
    25   345
    26   345
    27   345
    28   345
    29   345
    30    20
    31    20
    32    20
    33    20
    34    20
    35    20
    36    20
    37    20
    38    20
    39   345];


%
%IEEE 39 bus standard for EMT simulation
% Estimation of line length
%
% estimation of line length for IEEE 39 bus model.
% the original specification does not give the line length.
% therefore, we try to estimate them
% the propagation speed of overhead line is just below the speed of light
% 300000 km/s typically for positive sequence.
% How to do it?
% 1- find Cpos in F and Lpos in H like in the original model. (positive sequence value)
% 2- find a length len (km) that makes the 1/(sqrt(Lpos.Cpos)*len close to
% 290000-295000 km/s

% carefull, the paper value are in Pu
% test pour ligne 3-4
% Rpu=0.0013; Xpu=0.0213; Bpu=0.2214; %in pu from paper.
% w=2*pi*60;
% S=100e6;
% Vb=345e3; %base voltage,from the model
% Zb=Vb*Vb/S;
% 
% L=Xpu*Zb/w;
% C=Bpu/Zb/w;
% C =
%   4.9341e-007
% L =
%     0.0672
% OK!!!
%
% Donc L et C sont les inductances et capacitance totale de la ligne
% Il faut alors trouver une longueur qui donne une vitesse de propagation inférieur à la vitesse de la lumiere, 300000 km/s
% v=1/sqrt(l*c) ou l et c sont les inductances et capacitance par unité de longueur /km
% v=1/sqrt(L*C)*len  ou len est la longeur de la ligne
% une ligne aérienne typique a une vitesse de propagation juste en dessous de 300000 km/s
% (attention, certains cables ont des vitesses bien plus lente down to 100000 km/s)




%% inter area lines

TL_1_39_len=105;
% check : 1/sqrt(0.079*1.67e-6)*105=  2.8908e+005  ok, near but below 300000.
TL_3_4_len=53;
% check: 1/sqrt(0.067*0.493e-6)*53 = 2.9162e+005
TL_16_17_len=26;
% check: 1/sqrt(0.028*0.299e-6)*26= 2.8416e+005
TL_13_14_len=32;
% check: 1/sqrt(0.032*0.384e-6)*32= 2.8868e+005
TL_4_14_len=33;
% 1/sqrt(0.041*0.308e-6)*33= 2.9366e+005

%% East System
TL_16_24_len=15;
%1/sqrt(0.019*0.152e-6)*16=2.9773e+005
TL_24_23_len=88;
%1/sqrt(0.111*0.805e-6)*88=2.9439e+005
TL_16_21_len=46;
%1/sqrt(0.043*0.568e-6)*46 =2.9434e+005
TL_21_22_len=47;
%1/sqrt(0.044*0.572e-6)*47 = 2.9626e+005
TL_15_16_len=31.5;
%1/sqrt(0.03*0.381e-6)*31.5 =2.9464e+005
TL_16_19_len=61;
%1/sqrt(0.062*0.677e-6)*61 =2.9774e+005
TL_22_23_len=33;
%1/sqrt(0.03*0.411e-6)*33=2.9719e+005
TL_14_15_len=70;
%1/sqrt(0.069*0.816e-6)*70=2.9500e+005

%% North System
TL_26_29_len=200;
%1/sqrt(0.197*2.293e-6)*200=2.9757e+005
TL_26_28_len=151;
%1/sqrt(0.15*1.739e-6)*151 =2.9565e+005
TL_28_29_len=48;
%1/sqrt(0.048*0.555e-6)*48 =2.9409e+005
TL_25_26_len=101;
%1/sqrt(0.102*1.143e-6)*101= 2.9580e+005
TL_26_27_len=46.5;
%1/sqrt(0.046*0.534e-6)*46.5 =2.9669e+005
TL_2_25_len=28;
%1/sqrt(0.027*0.325e-6)*28 =2.9891e+005
TL_17_27_len=59;
%1/sqrt(0.055*0.717e-6)*59 =2.9711e+005
TL_17_18_len=26;
%1/sqrt(0.026*0.294e-6)*26 =2.9738e+005
TL_3_18_len=42;
%1/sqrt(0.042*0.476e-6)*42 =2.9704e+005
TL_2_2_len=49;
%1/sqrt(0.048*0.573e-6)*49=2.9546e+005
TL_1_2_len=134;
%1/sqrt(0.13*1.56e-6)*134 =2.9756e+005

%% West system
TL_4_5_len=32.5;
%1/sqrt(0.04*0.299e-6)*32.5 =2.9718e+005
TL_5_6_len=14;    % PI-line @ 50us
% 1/sqrt(0.014*0.162e-6)*14= 2.9397e+005
TL_6_11_len=26.5;
%1/sqrt(0.026*0.31e-6)*26.5= 2.9517e+005
TL_5_8_len=32;
%1/sqrt(0.035*0.329e-6)*32 =2.9821e+005
TL_6_7_len=25.5;
%1/sqrt(0.029*0.252e-6)*25.5 =2.9829e+005
TL_10_13_len=14;   % PI-line @ 50us
% 1/sqrt(0.014*0.162e-6)*14= 2.9397e+005
TL_10_11_len=14;   % PI-line @ 50us
% 1/sqrt(0.014*0.162e-6)*14= 2.9397e+005

TL_9_39_len=136;
%1/sqrt(0.079*2.674e-6)*136 = 2.9590e+005
TL_7_8_len=15;   % PI-line @ 50us
%1/sqrt(0.015*0.174e-6)*15 = 2.9361e+005
TL_8_9_len=93;
%1/sqrt(0.115*0.848e-6)*93 = 2.9781e+005




% needs to be done for all lines
% carefull with line shorter than 10-15 km becaue you may have to replace
% with pi-line if propagation dleay smaller than Ts
% criteria if (sqrt(L.C)<Ts  then 'switch to pi-line'

PSSMODEL=1;
LPSS=0;
WAPSS=0;
WAPSS2=0;


% Required data to run New England 39 Bus benchmark
% clear
% clc
% Definition for the columns
% system base =100 MVA

%% Line data Format (line)
% All values are given on the same system base MVA
% 1: From bus  
% 2: To bus   
% 3: Resistance (pu)
% 4: Reactance  (pu)
% 5: Charge     (pu)
% 6: Transformer Tap Amplitute
% 7: base MVA
% 8: Nomonal Voltage (KV) 

%   1   2    3        4       5     6    7   8    
line=[...
    1	2	0.0035	0.0411	0.6987	0	100	345
    1	39	0.001	0.025	0.75	0	100	345
    2	3	0.0013	0.0151	0.2572	0	100	345
    2	25	0.007	0.0086	0.146	0	100	345
    2	30	0	    0.0181	0	  1.025	100	22
    3	4	0.0013	0.0213	0.2214	0	100	345
    3	18	0.0011	0.0133	0.2138	0	100	345
    4	5	0.0008	0.0128	0.1342	0	100	345
    4	14	0.0008	0.0129	0.1382	0	100	345
    5	8	0.0008	0.0112	0.1476	0	100	345
    6	5	0.0002	0.0026	0.0434	0	100	345
    6	7	0.0006	0.0092	0.113	0	100	345
    6	11	0.0007	0.0082	0.1389	0	100	345
    7	8	0.0004	0.0046	0.078	0	100	345
    8	9	0.0023	0.0363	0.3804	0	100	345
    9	39	0.001	0.025	1.2	    0	100	345
    10	11	0.0004	0.0043	0.0729	0	100	345
    10	13	0.0004	0.0043	0.0729	0	100	345
    10	32	0	    0.02	0	  1.07	100	22
    12	11	0.0016	0.0435	0	  1.006	100	345
    12	13	0.0016	0.0435	0	  1.006	100	345
    13	14	0.0009	0.0101	0.1723	0	100	345
    14	15	0.0018	0.0217	0.366	0	100	345
    15	16	0.0009	0.0094	0.171	0	100	345
    16	17	0.0007	0.0089	0.1342	0	100	345
    16	19	0.0016	0.0195	0.304	0	100	345
    16	21	0.0008	0.0135	0.2548	0	100	345
    16	24	0.0003	0.0059	0.068	0	100	345
    17	18	0.0007	0.0082	0.1319	0	100	345
    17	27	0.0013	0.0173	0.3216	0	100	345
    19	33	0.0007	0.0142	0	  1.07	100	22
    19	20	0.0007	0.0138	0	  1.06	100	345
    20	34	0.0009	0.018	0	  1.009	100	22
    21	22	0.0008	0.014	0.2565	0	100	345
    22	23	0.0006	0.0096	0.1846	0	100	345
    22	35	0	    0.0143	0	  1.025	100	22
    23	24	0.0022	0.035	0.361	0	100	345
    23	36	0.0005	0.0272	0	    1	100	22
    25	26	0.0032	0.0323	0.513	0	100	345
    25	37	0.0006	0.0232	0	  1.025	100	22
    26	27	0.0014	0.0147	0.2396	0	100	345
    26	28	0.0043	0.0474	0.7802	0	100	345
    26	29	0.0057	0.0625	1.029	0	100	345
    28	29	0.0014	0.0151	0.249	0	100	345
    29	38	0.0008	0.0156	0	  1.025	100	22
    31	6	0	    0.025	0	    1	100	22];

%% Machine Data Format (mac_con)
% 1.  Machine Number
% 2.  Bus Number
% 3.  Base MVA
% 4.  Leakage Reactance x_l(pu)
% 5.  Resistance r_a(pu)
% 6.  d-axis sychronous reactance x_d(pu)
% 7.  d-axis transient reactance x'_d(pu)
% 8.  d-axis subtransient reactance x"_d(pu)
% 9.  d-axis open-circuit time constant T'_do(sec),
% 10. d-axis open-circuit subtransient time constant T"_do(sec)
% 11. q-axis sychronous reactance x_q(pu)
% 12. q-axis transient reactance x'_q(pu)
% 13. q-axis subtransient reactance x"_q(pu)
% 14. q-axis open-circuit time constant T'_qo(sec)
% 15. q-axis open circuit subtransient time constant % T"_qo(sec)
% 16. inertia constant H(sec)
% 17. damping coefficient d_o(pu)
% 18. dampling coefficient d_1(pu)
% 19. bus number
% Note: all the following machines use sub-transient model
% 1  2     3      4      5      6     7    8      9    10    11    12    13    14    15    16    17    18  19
mac_con=[
  1  39  1000.0 0.30 0.0010  0.200 0.060 0.01  7.000 0.003 0.190 0.080 0.03  0.700 0.005 50.00 0.000 0.00 39 ;
  2  31  1000.0 0.350 0.0270  2.950 0.697 0.01  6.560 0.003 2.820 1.7   0.03  1.500 0.005 3.030 0.000 0.00 31 ;
  3  32  1000.0 0.304 .00386  2.495 0.531 0.01  5.700 0.003 2.370 0.876 0.03  1.500 0.005 3.580 0.000 0.00 32 ;
  4  33  1000.0 0.295 .00222  2.620 0.436 0.01  5.690 0.003 2.580 1.66  0.03  1.500 0.005 2.860 0.000 0.00 33 ;
  5  34  1000.0 0.540 0.0014  6.700 1.320 0.01  5.400 0.003 6.200 1.66  0.03  0.440 0.005 2.600 0.000 0.00 34 ;
  6  35  1000.0 0.224 0.0615  2.540 0.500 0.01  7.300 0.003 2.410 0.814 0.03  0.400 0.005 3.480 0.000 0.00 35 ;
  7  36  1000.0 0.322 .00268  2.950 0.490 0.01  5.660 0.003 2.920 1.86  0.03  1.500 0.005 2.640 0.000 0.00 36 ;
  8  37  1000.0 0.280 .00686  2.900 0.570 0.01  6.700 0.003 2.800 0.911 0.03  0.410 0.005 2.430 0.000 0.00 37 ;
  9  38  1000.0 0.298 0.0030  2.106 0.570 0.01  4.790 0.003 2.050 0.587 0.03  1.960 0.005 3.450 0.000 0.00 38 ;
  10 30  1000.0 0.125 0.0014  1.000 0.310 0.01  10.20 0.003 0.690 0.08  0.03  1.500 0.005 4.200 0.000 0.00 30 ];

mac_con(:,5)=0;

p0=[1000 520.81 650 632 508 650 560 540 830 250]'./1000;%Active Power Generation of PV units
Pn(1:10,1)=mac_con(:,3)/1000; % Nominal Power

%% Power System Stabilizer Format (MB)
% Applied power system stabilizer is MBPSS with simplified settings
% Note: All machines use MBPSS with same configuration 
% 1: Global gain (G)
% 2: Frequency of low frequency band (FL) Hz
% 3: Gain of low frequency band (KL)
% 4: Frequency of intermediate frequency band (FI) Hz
% 5: Gain of intermediate frequency band (KI)
% 6: Frequency of high frequency band (FH) Hz
% 7: Gain of high frequency band (KH)
%   1    2  3     4   5      6  7
MB=[1   0.2 30   1.25 40    12 160];

%% Excitation System format (AVR_Data)
% All machines use IEEE type 1 synchronous machine voltage regulator combined to an exciter
% 1. Low pass filter time constant (Tr) sec
% 2. Regulator gain (Ka)
% 3. regulator time constant (Ta) sec
% 4. Lead-lag compensator time constant (Tb) sec
% 5. Lead-lag compensator time constant (Tc) sec
% 6. Terminal voltage (pu)
% 7. Lower limit for regulator output (Emin)   
% 8. Upper limit for regulator output (Emax)
%     1    2      3      4     5     6       7    8
  AVR_Data=[...
    0.01  200    0.015   10    1    1.03    -5    5
    0.01  200    0.015   10    1    1.03    -5    5
    0.01  200    0.015   10    1    1.03    -5    5
    0.01  200    0.015   10    1    1.03    -5    5
    0.01  200    0.015   10    1    1.03    -5    5
    0.01  200    0.015   10    1    1.03    -5    5
    0.01  200    0.015   10    1    1.03    -5    5
    0.01  200    0.015   10    1    1.03    -5    5
    0.01  200    0.015   10    1    1.03    -5    5
    0.01  200    0.015   10    1    1.03    -5    5];
 
C0=829.7e-9;
L0=3.220e-3;
R0=1;
Ns=120*60/(2); %Nominal speed of synchronous machines
s=10;
% PSSModel=1;%1:No pSS //1:MB 
%% Bus data (Bus)
% 1. Bus number
% 2. Nominal phase-phase voltage 
%    1    2
Bus = [ ...
     1   345
     2   345
     3   345
     4   345
     5   345
     6   345
     7   345
     8   345
     9   345
    10   345
    11   345
    12   230
    13   345
    14   345
    15   345
    16   345
    17   345
    18   345
    19   345
    20   345
    21   345
    22   345
    23   345
    24   345
    25   345
    26   345
    27   345
    28   345
    29   345
    30    22
    31    22
    32    22
    33    22
    34    22
    35    22
    36    22
    37    22
    38    22
    39   345];

   zbase=(line(:,8).^2)./line(:,7);
   line(:,3)=line(:,3).*zbase;
   line(:,4)=line(:,4).*zbase/(120*pi);
   line(:,5)=line(:,5)./zbase/(120*pi);

Ts=5e-5;
%mode_of_operation=1;
%BESS_control_mode=1;
%Gen_Q_Control =1;

%Exciter_init;

%Exciter_init
Xq=0.474;
%%====================================================
% ST6C Exciter Parameters

KPA=30;
KIA=59.91;
KFF=1;
KM=1;
KLR=22.47;
KCL=1.04;
VAMAX=7.05;
VAMIN=-6.11;
VRMAX=7.05;
VRMIN=-6.11;
ILR=5.02;
KG=1;
TG=0.024;
TA=0.01;
VBMAX=999;
VMMAX=7.05;
VMMIN=-6.11;
KI=0;
Kp=1;
Kc=0;
Vt0=0;
VAAMAX=VAMAX;
%%========================================================
% UEL2C parameters

TUQ=.005;
TUP=0.050;
TUV=0.2;
TQREF=0;
Tadj=0;
K1=0;
K2=2;
VBIAS=1;
KFB=0;
TUL=0;
VUImax=1;
VUImin=-1;
VUELmax1=1;
VUELmax2=1;
VUELmin1=-1;
VUELmin2=-1;
KUF=0;
KUL=0.24;
KUI=0.001;
TU1=0;TU2=0;TU3=0;TU4=0;
KFIX=1;
VF=0;
VFB=0;
%%===========================================================
%OEL3C PARAMETERS
ITFpu=1.10;
TF=0.02;
KSCALE=0.319;
K1=1;
TOEL=5.00;
VOELmax1=0.2353;
VOELmax2=0.0;
VOELmin1=-0.24;
VOELmin2=-1.0;
KPOEL=1;
KOEL=0.24;
%%=============================================================
%SCL1C
TQSCL=0.012;
TIT=0.1;
TINV=0;
KIUOEX=1;
KIUUEX=1;
KIOEX=0.0303;
KIUEX=0.0303;
KPOEX=0;
KPUEX=0;
VSCLmax=0.20;
VSCLmin=-0.10;
ISCLIM=1;
IQMIN=0.1;
K=1;

VAAMAX=VAMAX;
Xq=0.474;
Ts=5e-5;
f=60;
Ts_Control = 5e-5;
Tbess_Control= 5e-5;

db1=.015;
delay=2;
KP1=1;
KI1=10;
KP2=60;
KI2=5;
KP3=1;
KI3=10;
KP4=1;
KI4=50;
KP5=1;
KI5=25;
Pt_BE_noCC=50e6;
Pt_BE_CC=315e6;
Qt_BE=35e6;
G_ferr=20e6;
Qgain_Vref_BESS=20e6;
KP6=1;
KI6=25;
P_t=334e6;
Ts_grid=Ts;
vNomHV = 345E3;
fNom=60;





     





