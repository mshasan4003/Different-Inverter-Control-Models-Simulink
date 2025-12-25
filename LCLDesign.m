function [L1,L2,Cf,Rf,fr,Cdc] = LCLDesign(fsw,f,Vdc,Vpp,P)
%LCLDESIGN Summary of this function goes here
%   Detailed explanation goes here
w = 2*pi*f;
Vph = Vpp/sqrt(3);
%DC link capacitor
Idc = P/Vdc;
dV = 0.05*Vdc;
Cdc = 1.5*P/(2*w*Vdc*dV);
%LCL filter design
Zb = Vpp^2/P;
Cb = 1/(2*pi*f*Zb);
Cf = 0.05*Cb;
dIm = 0.1*P*sqrt(2)/(3*Vph);
L1 = Vdc/(6*fsw*dIm);
ka = 0.2;
L2 = (sqrt(1/ka^2) + 1)/(Cf*(2*pi*fsw)^2); 
wr = sqrt((L1+L2)/(L1*L2*Cf));
fr = wr/(2*pi);
Rf = 1/(3*wr*Cf);
Iinv = P/(Vpp*sqrt(3));
Iinvpk = Iinv*sqrt(2);
Isc = Iinvpk*1.2;
Lb = Zb/w;
Lpu = (L1+L2)/Lb;
end

