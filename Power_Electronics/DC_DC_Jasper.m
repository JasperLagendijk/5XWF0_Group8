%% Boost converter - Inductance
clear all, close all, clc

V_in = [10, 30];
V_out = 60;
eta = 0.8;
f = 10e3;
I_max = 2;


delta_L_est = 0.4*I_max*(V_out/mean(V_in));


L = mean(V_in)*(V_out-mean(V_in))/(delta_L_est*f*V_out)

%% Boost converter - Input Capacitance
V_p = 5;
dc = 0.8;

C_min = (I_max*dc*(1-dc)*1000)/(f*V_p)


%% Inductance of coil



%% 
clc, clear all, close all
Vi = [8 50]; %V
Ii = [5 2.5]; %A
Vo = 60; %V
fs = 20e3;
eta = 1;

R = (Vo^2*eta)./(Vi.*Ii);
%R = 10;
V_ripple = 0.4; %
I_ripple = 0.2; %

d = (Vo-Vi)/Vo;

I_L = Vi./((1-d).^2.*R);
delta_iL = I_ripple * I_L

L_temp = d.*Vi./(fs.*delta_iL);
 
%% Inductance of a coil
L = max(L_temp)*1.5;
mu


