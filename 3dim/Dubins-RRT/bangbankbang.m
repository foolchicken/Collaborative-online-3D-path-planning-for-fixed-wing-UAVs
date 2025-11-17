close all
clear
clc

% syms umax umin tf xf vf
%
% a2 = vf - umax*tf;
% a1 = xf - 0.5*umax*tf^2-a2*tf

umin = -1;
umax = 1;
tf = 1;
x0 = 0;
xf = 0;
v0 = 0.4;
vf = 0;


syms ta tb

umin * ta - umax * tb
vf - v0 - umax * tf

0.5*umin * ta ^ 2 - 0.5*umax * tb ^ 2 + (vf-v0)*tb
umax*tf

