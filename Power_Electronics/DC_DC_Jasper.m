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