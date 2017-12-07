function [y] = f_car(state, ctrl, dt)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

m = 1400;
W = 13720;
Nw = 2;
f = 0.01;
Iz = 2667;
a = 1.35;
b = 1.45;
By = 0.27;
Cy = 1.2;
Dy = 2921;
Ey = -1.6;
Shy = 0;
Svy = 0;

x = state(1);
u = state(2);
y = state(3);
v = state(4);
phi = state(5);
r = state(6);

delta_f = ctrl(1);
Fx = ctrl(2);

af = (delta_f - atan((v+a*r)/u))*180/pi;
ar = -(atan((v-b*r)/u))*180/pi;
phi_yf = (1-Ey)*(af+Shy)+Ey/By*atan(By*(af+Shy));
phi_yr = (1-Ey)*(ar+Shy)+Ey/By*atan(By*(ar+Shy));
Fyf = Dy*sin(Cy*atan(By*phi_yf))+Svy;
Fyr = Dy*sin(Cy*atan(By*phi_yr))+Svy;

dx = u*cos(phi)-v*sin(phi);
du = 1/m*(-f*W + Nw*Fx - Fyf*sin(delta_f)) + v*r;
dy = u*sin(phi)+v*cos(phi);
dv = 1/m*(Fyf*cos(delta_f)+Fyr) - u*r;
dphi = r;
dr = 1/Iz*(a*Fyf*cos(delta_f)-b*Fyr);

y = state + [dx; du; dy; dv; dphi; dr]*dt;
end

