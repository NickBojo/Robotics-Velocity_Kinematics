function [ J ] = ScaraJacobian( q )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

k0 = [0 0 1]';
k1 = k0;
k2 = -k0;
k3 = [0 1 0]';
k4 = [1 0 0]';
k5 = k2;
k6 = k5;

l1 = 1;
l2 = 1;
l3 = 0.5;
l6 = 0.2;

C0 = eye(3);
o0 = [0; 0; 0];
coord0 = [ C0 o0;
           zeros(1,3), 1];

coord1 = coord0*dh_transform(pi/2 + q(1), l3 + l6, l1, 0);
coord2 = coord1*dh_transform(q(2), 0, l2, pi);
coord3 = coord2*dh_transform(-pi/2,l3+q(3), 0, -pi/2);
coord4 = coord3*dh_transform(pi/2 + q(4), 0, 0, -pi/2);
coord5 = coord4*dh_transform(-pi/2 + q(5), 0, 0, pi/2);
coord6 = coord5*dh_transform(pi/2 + q(6), l6, 0, 0);

o1 = coord1(1:3,4);
o4 = coord4(1:3,4)
o6 = coord6(1:3,4)

J = [cross(k0, o6 -o0) cross(k1, o6 - o1) k2 cross(k3, o6-o4) cross(k4, o6-o4) cross(k5, o6-o4);
     k0                k1                 zeros(3,1)  k3               k4                k5];
end

