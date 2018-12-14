%% Derivatives of a Euler->Angular velocity mappings w.r.t euler angles.
%
% Used in class EulerConverter to specifcy analytical derivatives.
%
% Author: Alexander Winkler

clc;
clear all;

syms z y x vx vy vz real %euler angles yaw, pitch, roll 



%% derivative of rotation matrix defined by euler angles
% from kindr cheet sheet, using convention xyz
% see https://docs.leggedrobotics.com/kindr/cheatsheet_latest.pdf

R = [cos(y)*cos(z), cos(z)*sin(x)*sin(y) - cos(x)*sin(z), sin(x)*sin(z) + cos(x)*cos(z)*sin(y);
     cos(y)*sin(z), cos(x)*cos(z) + sin(x)*sin(y)*sin(z), cos(x)*sin(y)*sin(z) - cos(z)*sin(x);
           -sin(y),                        cos(y)*sin(x), cos(x)*cos(y)]

v = [vx, vy, vz]'         

% R =[cos(y)*cos(z),                         -cos(y) * sin(z),                           sin(y);
%     cos(x)*sin(z) + cos(z) * sin(x) *sin(y), cos(x)*cos(z) - sin(x)*sin(y)*sin(z),     -cos(y)*sin(x);
%     sin(x)*sin(z) - cos(x) * cos(z) *sin(y), cos(z) * sin(x) + cos(x) *sin(y) * sin(z), cos(x)*cos(y)]
         

       
product = R * v;

simplify(jacobian(product, [x , y ,z]))
