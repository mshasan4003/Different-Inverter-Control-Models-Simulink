%clear; clear all; clc;
inv1 = struct; % 500 kW inverter
inv2 = struct; % 180 kW inverter
Ts = 50e-6;
Ts_Control = 50e-6;
f = 60;
w = 2*pi*f;
Vg = 16.5e3;   %4.16e3 for 123 bus, 12.47e3 for 650 bus, 2401.78 for 13 bus 
Vpp = 480;     %480 for 123 bus, 400 for 650 bus
Vm = sqrt(2)*Vpp/sqrt(3); % peak phase voltage
Vdc = 1000;     %900 for 123 bus, 700 for 650 bus
Sw = 10000;    %PWM switching frequency

%% Grid forming inverter parameters
inv1.P = 100e6;
inv1.on = 0.00;
[inv1.L,inv1.Lg,inv1.Cf,inv1.Rf,inv1.fRes,inv1.C] = LCLDesign(Sw,f,Vdc,Vpp,inv1.P);
inv1.R = 0.0266;
% L = 0.00031753;
% Cf = 149.21e-6;
% Rf = 0.0857;
% Lg = 0.00001;
inv1.Rg = 0.001;
inv1.Tf = inv1.L/inv1.R; %Filter time constant
inv1.Idc = inv1.P/Vdc;
% C =  14616e-6;
inv1.Zb = Vpp^2/inv1.P;
inv1.Lb = inv1.Zb/w;
inv1.Lpu = inv1.L/inv1.Lb;
inv1.Im = sqrt(2)*inv1.P/(sqrt(3)*Vpp);
inv1.Lmax = Vdc/(3*w*inv1.Im);
inv1.Cb = 1/(w*inv1.Zb);

%PLL integrator
inv1.kp_pll = 60;
inv1.ki_pll = 1400;

%current controller integrator
inv1.kp_cd = 0.3*1.1;
inv1.kp_cq = 0.3*1.1;
inv1.ki_cd = 20*1.1;
inv1.ki_cq = 20*1.1;
% For these equations to work, tune R thereby tf of the model
% inv1.kp_cd = inv1.L/(3*Ts_Control);
% inv1.kp_cq = inv1.L/(3*Ts_Control);
% inv1.ki_cd = inv1.kp_cd/inv1.Tf;
% inv1.ki_cq = inv1.kp_cq/inv1.Tf;
inv1.wn = sqrt(2*inv1.kp_cd/(3*Ts*inv1.L));
inv1.zeta = 1/(3*Ts*inv1.wn);

% DC link controller
inv1.kp_VDC = 3;
inv1.ki_VDC = 200;

% %outer loop controller
% kp_vcd = 0.3*1.1;
% kp_vcq = 0.3*1.1;
% ki_vcd = 20*1.1;
% ki_vcq = 20*1.1;

%outer loop controller
inv1.kp_vcd = inv1.L/(3*Ts_Control);
inv1.kp_vcq = inv1.L/(3*Ts_Control);
inv1.ki_vcd = inv1.kp_vcd/inv1.Tf;
inv1.ki_vcq = inv1.kp_vcq/inv1.Tf;


% Anti-wind up gain
inv1.k_aw = 1;
inv1.clMax = 1.5;
inv1.clMin = -1.5;
inv1.vlMax = 1.2;
inv1.vlMin = -1.2;
inv1.type = 1;
%%

%% Grid following inverter parameters
inv2.P = 100e6;
inv2.on = 0.07;
[inv2.L,inv2.Lg,inv2.Cf,inv2.Rf,inv2.fRes,inv2.C] = LCLDesign(Sw,f,Vdc,Vpp,inv2.P);
inv2.R = 0.0266;
% L = 0.00031753;
% Cf = 149.21e-6;
% Rf = 0.0857;
% Lg = 0.00001;
inv2.Rg = 0.001;
inv2.Tf = inv2.L/inv2.R; %Filter time constant
inv2.Idc = inv2.P/Vdc;
% C =  14616e-6;
inv2.Zb = Vpp^2/inv2.P;
inv2.Lb = inv2.Zb/w;
inv2.Lpu = inv2.L/inv2.Lb;
inv2.Im = sqrt(2)*inv2.P/(sqrt(3)*Vpp);
inv2.Lmax = Vdc/(3*w*inv2.Im);
inv2.Cb = 1/(w*inv2.Zb);

%PLL integrator
inv2.kp_pll = 60;
inv2.ki_pll = 1400;

%current controller integrator
inv2.kp_cd = 0.3*1.1;
inv2.kp_cq = 0.3*1.1;
inv2.ki_cd = 20*1.1;
inv2.ki_cq = 20*1.1;
% For these equations to work, tune R thereby tf of the model
% kp_cd = L/(3*Ts_Control);
% kp_cq = L/(3*Ts_Control);
% ki_cd = kp_vcd/Tf;
% ki_cq = kp_vcq/Tf;
inv2.wn = sqrt(2*inv2.kp_cd/(3*Ts*inv2.L));
inv2.zeta = 1/(3*Ts*inv2.wn);

% DC link controller
inv2.kp_VDC = 3;
inv2.ki_VDC = 200;

% %outer loop controller
% kp_vcd = 0.3*1.1;
% kp_vcq = 0.3*1.1;
% ki_vcd = 20*1.1;
% ki_vcq = 20*1.1;

%outer loop controller
inv2.kp_vcd = inv2.L/(3*Ts_Control);
inv2.kp_vcq = inv2.L/(3*Ts_Control);
inv2.ki_vcd = inv2.kp_vcd/inv2.Tf;
inv2.ki_vcq = inv2.kp_vcq/inv2.Tf;


% Anti-wind up gain
inv2.k_aw = 1;
inv2.clMax = 1.1;
inv2.clMin = -1.1;
inv2.vlMax = 1.2;
inv2.vlMin = -1.2;
inv2.type = 0;
%%


%Droop co-efficients for PVDERPUModelsPNMultiInvDroop3.slx
% As of now they are smae for both inverters
KQ_p = 0.4;
KP_p = 0.2;
KQ_i = inv1.ki_cq/10;
KP_i = inv1.ki_cd/10;
KV_e = 1;

% Sync check PI gains - PI output given to all inverter reference gen blocks
kp_sync = KQ_p/10; %KQ_p is 0.1
ki_sync = inv1.ki_cd*40/50; % inv1.ki_cd is 22
