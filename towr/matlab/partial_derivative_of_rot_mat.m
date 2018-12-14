%% Derivatives of a Euler->Angular velocity mappings w.r.t euler angles.
%
% Used in class EulerConverter to specifcy analytical derivatives.
%
% Author: Alexander Winkler

clc;
clear all;

syms z y x %euler angles yaw, pitch, roll 



%% derivative of rotation matrix defined by euler angles
% from kindr cheet sheet, using convention zyx
% see https://docs.leggedrobotics.com/kindr/cheatsheet_latest.pdf

R = [cos(y)*cos(z), cos(z)*sin(x)*sin(y) - cos(x)*sin(z), sin(x)*sin(z) + cos(x)*cos(z)*sin(y);
     cos(y)*sin(z), cos(x)*cos(z) + sin(x)*sin(y)*sin(z), cos(x)*sin(y)*sin(z) - cos(z)*sin(x);
           -sin(y),                        cos(y)*sin(x), cos(x)*cos(y)]
         

% R =[cos(y)*cos(z),                         -cos(y) * sin(z),                           sin(y);
%     cos(x)*sin(z) + cos(z) * sin(x) *sin(y), cos(x)*cos(z) - sin(x)*sin(y)*sin(z),     -cos(y)*sin(x);
%     sin(x)*sin(z) - cos(x) * cos(z) *sin(y), cos(z) * sin(x) + cos(x) *sin(y) * sin(z), cos(x)*cos(y)]
         

       
axis = R(:,2);

simplify(jacobian(axis, [x , y ,z]))
