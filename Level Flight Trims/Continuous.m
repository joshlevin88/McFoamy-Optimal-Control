function phaseout = Continuous(input)

u          = input.phase.state(:,1);
w          = input.phase.state(:,2);
q          = input.phase.state(:,3);
theta      = input.phase.state(:,4);
% x          = input.phase.state(:,5);
% z          = input.phase.state(:,6);
Elev       = input.phase.state(:,7);
Thrust     = input.phase.state(:,8);

u_Elev     = input.phase.control(:,1);
u_Thrust   = input.phase.control(:,2);
%--------------------------------------------------------------------------
%                    Calculate Forces and Mo[ments
%--------------------------------------------------------------------------

LAil      = zeros(length(u),1);
Rud       = zeros(length(u),1);
v         = zeros(length(u),1);
p         = zeros(length(u),1);
r         = zeros(length(u),1);
[Fx,Fy,Fz,Mx,My,Mz] = arrayfun(@McFoamy_FM, LAil, Elev, Rud, Thrust, u, v, w, p, q, r);

%--------------------------------------------------------------------------
%                       Mass properties
%--------------------------------------------------------------------------

m        = input.auxdata.m;
g        = input.auxdata.g;
Iyy      = input.auxdata.Iyy;
V0       = input.auxdata.V0;
 
%--------------------------------------------------------------------------
%                               ODEs 
%--------------------------------------------------------------------------
udot       = Fx./m - g.*sin(theta) - q.*w;
wdot       = Fz./m + g.*cos(theta) + q.*u;

qdot       = My./Iyy;

thetadot   = q;

xdot       = u.*cos(theta)  + w.*sin(theta);
zdot       = -u.*sin(theta) + w.*cos(theta);

Elevdot    = u_Elev;
Thrustdot  = u_Thrust;

%--------------------------------------------------------------------------
%                     Objective Functional
%--------------------------------------------------------------------------

V = sqrt(u.^2 + w.^2);
phaseout.integrand = (V-V0).^2 + zdot.^2;
phaseout.dynamics  = [udot, wdot, qdot, thetadot, xdot, zdot, Elevdot, Thrustdot];
                  