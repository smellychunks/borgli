function flip_math

% constants, mks units
g = 9.81; % gravity acceleration

% Geometric Properties, Body (Hollow Cube)
l_b = 100e-3;
t_b = 5e-3;
%rho_b = 1e3;
rho_b = 1e3;
l = l_b;

% Geometric Properties, Wheel (Ring)
OD_w = 80e-3;
ID_w = 70e-3;
t_w = 30e-3;
rho_w = 6e3;

% Mass Properties
[m_b, I_b] = inertia_hollow_cube(l_b,t_b,rho_b);
[m_w, I_w] = inertia_wheel(OD_w,ID_w,t_w,rho_w);

% Required angular velocity to achieve flip, assuming perfect stop
omega_w = sqrt( (2-sqrt(2)) .* ( I_w + I_b +m_w.*l.^2 ) ./ I_w.^2 .*
  ( m_b.*l_b + m_w.*l ).*g );
motor_rpm = omega_w ./ (2*pi) .* 60;
fprintf('Motor Speed:\n%.2f rad/s\n%.2f RPM\n',omega_w,motor_rpm)

% Motor Properties
W = 200; % Motor Power
KV = 1250; % Motor KV rating
V = 11.1; % Voltage applied
T = W./(KV .* V); % Motor Torque
alpha_w = T./I_w; % Angular acceleration of flywheel
t = omega_w./alpha_w; % Spin-up time
fprintf('Spin-up Time:\n%.2f s\n',t)
end

function [m, I] = inertia_hollow_cube(L, t, rho)
% Result verified in CAD
% L: outer edge length
% t: wall thickness
% rho: density
l = L - 2.*t;
m = rho.*(L.^3 - l.^3);
I = (1/6) .* rho.*(L.^5 - l.^5) + (1/2).*m.*L.^2 ;
end

function [m, I] = inertia_wheel(OD, ID, t, rho)
% Result verified in CAD
% OD: Outer diameter
% ID: Inner diameter
% t: thickness
% rho: densitys
R = OD./2;
r = ID./2;
m = pi.*rho.*t.*(R.^2-r.^2);
I = 1/2.*m.*(R.^2 + r.^2);
end

%Diff eqs from paper
%C_b = 1.02e-3;
%C_w = .05e-3;
%% initial conditions
%theta_b = -pi/2;
%theta_w = 0;
%
%alpha_b = ( (m_b.*l_b + m_w.*l) .* g.*sin(theta_b) - T_m ...
%  - C_b.*omega_b + C_w.*omega_w ) ./ (I_b + m_w*l.^2);
%  
%alpha_w = ( (I_b + I_w + m_w.*l.^2).*(T_m - C_w.*omega_w) )./ ...
%  ( I_w*(I_b + m_w.*l.^2) ) - ...
%  ( m_b.*l_b + m_w.*l ) .* g.*sin(theta_b) - C_b.*omega_b ) ...
%  ( I_b + m_w.*l.^2 );
%l = .085; % Distance, wheel center to pivot point
%l_b = .075; % Distance, pendulum center of mass to pivot point
%m_b = .419; % Mass, body
%m_w = .204; % Mass, wheel
%I_b = 3.34e-3; % Moment of inertia, body
%I_w = .57e-3; % Moment of intertia, wheel