function dz = dynamics2(z,u,p)
%% System parameters 

m = p.m;
I = p.I;
r = p.r;

%% State definition

x = z(1,:);  % not used in the dynamics
y = z(2,:);  % not used in the dynamics
q = z(3,:);
dx = z(4,:);
dy = z(5,:);
dq = z(6,:);

%% Input definition

Fx = u(1,:);
Fy = u(2,:);
thau = u(3,:);

%% System dynamics

ddx = Fx/m - thau/I .* r .* sin(q) - dq.^2.*r.*cos(q);
ddy = Fy/m + thau/I .* r .* cos(q) - dq.^2.*r.*sin(q);
ddq = thau/I;

dz = [dx;dy;dq;ddx;ddy;ddq];


end