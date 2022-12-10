function dz = vertDynamics(z,u,p)

% X is the state
% u is the input 
% p is the problem settings
% dx is the state derivatives

m = p.m;

dx = z(1,:);
v_x = z(2,:);
y = z(3,:);
v_y = z(4,:);

F = u(1,:);
theta = u(2,:);
% 
% ddx = F*cos(theta)/p.m;
% ddy = F*sin(theta)/p.m;

dx = v_x;
dv_x = (F/m).*cos(theta);
dy = v_y;
dv_y = (F/m).*sin(theta);

dz = [dx; dv_x; dy; dv_y];

end