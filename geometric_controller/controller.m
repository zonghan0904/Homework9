function out = controller(u,P)

% input(25*1):desired trajectory and full state feedback, x v R Omega time
% output(4*1): force and moment control input

% process inputs
xd    = u(1:3);
b1d   = u(4:6);

% current state
x     = u(7:9);
v     = u(10:12);
R     = reshape(u(13:21),3,3);
Omega = u(22:24);
t     = u(end);

Rd = [1 0 0; 0 1 0; 0 0 1;];
Omega_d = [0; 0.1; 1];

Rd_dot = Rd * hat(Omega_d);
eR = vee((Rd.' * R - R.' * Rd)) / 2;
eOmega = Omega - R.' * Rd * Omega_d;

ex = x(3) - xd(3);
ev = [0; 0; 0];
e3 = [0; 0; 1];
xd_ddot = [0; 0; 0];

J = [P.Jxx 0 0; 0 P.Jyy 0; 0 0 P.Jzz]; 

f = -(-P.kx * ex - P.kv * ev - P.mass * P.gravity * e3 + P.mass * xd_ddot).'* R * e3;
M = -P.kR * eR - P.kOmega * eOmega + cross(Omega, J * Omega);

% f = 0;
% M = [0; 0; 0];

out = [f;M];
end