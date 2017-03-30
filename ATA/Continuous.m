function phaseout = Continuous(input)

u           = input.phase.state(:,1);
v           = input.phase.state(:,2);
w           = input.phase.state(:,3);
p           = input.phase.state(:,4);
q           = input.phase.state(:,5);
r           = input.phase.state(:,6);
q1          = input.phase.state(:,7);
q2          = input.phase.state(:,8);
q3          = input.phase.state(:,9);
q4          = input.phase.state(:,10);
% x           = input.phase.state(:,11);
% y           = input.phase.state(:,12);
% z           = input.phase.state(:,13);

LAil        = input.phase.state(:,14);
Elev        = input.phase.state(:,15);
Rud         = input.phase.state(:,16);
Thrust      = input.phase.state(:,17);

u_LAil      = input.phase.control(:,1);
u_Elev      = input.phase.control(:,2);
u_Rud       = input.phase.control(:,3);
u_Thrust    = input.phase.control(:,4);

%--------------------------------------------------------------------------
%                               Aux Data
%--------------------------------------------------------------------------

m           = input.auxdata.m;
g           = input.auxdata.g;
Ixx         = input.auxdata.Ixx;
Iyy         = input.auxdata.Iyy;
Izz         = input.auxdata.Izz;
Ixz         = input.auxdata.Ixz;

%--------------------------------------------------------------------------
%                    Calculate Forces and Mo[ments
%--------------------------------------------------------------------------

[Fx,Fy,Fz,Mx,My,Mz] = arrayfun(@McFoamy_FM, LAil,Elev,Rud,Thrust,u, v, w ,p, q, r);

%--------------------------------------------------------------------------
%                               ODEs 
%--------------------------------------------------------------------------
udot       = Fx./m + g.*2.*(q2.*q4 - q1.*q3) + r.*v - q.*w;
vdot       = Fy./m + g.*2.*(q3.*q4 + q1.*q2) + p.*w - r.*u;
wdot       = Fz./m + g.*(q1.^2 - q2.^2 - q3.^2 + q4.^2) + q.*u - p.*v;

pdot       = (Izz.*Mx + Ixz.*Mz - (Ixz.*(Iyy - Ixx - Izz).*p + (Ixz.^2 + Izz.*(Izz - Iyy)).*r).*q)./(Ixx.*Izz - Ixz.^2);
qdot       = 1./Iyy.*(My - (Ixx - Izz).*p.*r - Ixz.*(p.^2 - r.^2));
rdot       = (Ixz.*Mx + Ixx.*Mz - (Ixz.*(Iyy - Ixx - Izz).*r + (Ixz.^2 + Ixx.*(Ixx - Iyy)).*p).*q)./(Ixx.*Izz - Ixz.^2);

q1dot      = -1./2.*(q2.*p + q3.*q + q4.*r);
q2dot      = 1./2.*(q1.*p + q3.*r - q4.*q);
q3dot      = 1./2.*(q1.*q + q4.*p - q2.*r);
q4dot      = 1./2.*(q1.*r + q2.*q - q3.*p);

xdot       = u.*(q1.^2 + q2.^2 - q3.^2 - q4.^2) + v.*2.*(q2.*q3 - q1.*q4)            + w.*2.*(q1.*q3 + q2.*q4);
ydot       = u.*2.*(q2.*q3 + q1.*q4)            + v.*(q1.^2 - q2.^2 + q3.^2 - q4.^2) + w.*2.*(q3.*q4 - q1.*q2);
zdot       = u.*2.*(q2.*q4 - q1.*q3)            + v.*2.*(q3.*q4 + q1.*q2)            + w.*(q1.^2 - q2.^2 - q3.^2 + q4.^2);

LAildot    = u_LAil;
Elevdot    = u_Elev;
Ruddot     = u_Rud;
Thrustdot  = u_Thrust;


%--------------------------------------------------------------------------
%                     Path Constraint
%--------------------------------------------------------------------------

phaseout.path = [q1.^2 + q2.^2 + q3.^2 + q4.^2];

%--------------------------------------------------------------------------
%                     Objective Functional
%--------------------------------------------------------------------------
phaseout.integrand = (u_LAil./5).^2 + (u_Elev./5).^2 + (u_Rud./5).^2 + (u_Thrust./1500).^2;

phaseout.dynamics  = [udot, vdot, wdot, pdot, qdot, rdot, q1dot, q2dot, q3dot, q4dot, xdot, ydot, zdot,...
                      LAildot, Elevdot, Ruddot, Thrustdot];          
