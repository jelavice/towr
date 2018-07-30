

clc;
clear all;









totalTime = 2;
t = linspace(0, totalTime, 200);


p0 = 1.0;
v0 = 0.0;
p1 = 2.0;
v1 = 1.0;
T = totalTime;

a = p0;

b = v0;

c = -(3*(p0 - p1) + T*(2*v0+v1))/ T^2;

d = (2*(p0-p1)+T*(v0+v1)) / T^3;
% a third-order polynomial and it's derivatives
pos  = d*t.^3 + c*t.^2 + b*t + a;


plot(t, pos, 'linewidth', 2);
grid on




