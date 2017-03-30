function phaseout = Continuous(input)

% time = input.phase.time;
u =          input.phase(1).state(:,1);
w =          input.phase(1).state(:,2);
q =          input.phase(1).state(:,3);
theta =      input.phase(1).state(:,4);
x =          input.phase(1).state(:,5);
z =          input.phase(1).state(:,6);
ElevDef =    input.phase(1).state(:,7);
wIn =        input.phase(1).state(:,8);

u_elev =     input.phase(1).control(:,1);
u_wIn =      input.phase(1).control(:,2);
%--------------------------------------------------------------------------
%                    Calculate Forces and Mo[ments
%--------------------------------------------------------------------------
% [Fx, Fz, My] = arrayfun(@FM_Waqas_Sept2016_Long, ElevDef, wIn, u, w, q);

LAilDef =   zeros(length(u),1);
RudDef =    zeros(length(u),1);
v =         zeros(length(u),1);
p =         zeros(length(u),1);
r =         zeros(length(u),1);
[Fx,Fy,Fz,Mx,My,Mz] = arrayfun(@McFoamy_FM, LAilDef,ElevDef,RudDef,wIn,u, v, w ,p, q, r);

%--------------------------------------------------------------------------
%                       Mass properties
%--------------------------------------------------------------------------

m =         input.auxdata.m;
g =         input.auxdata.g;
Iy =        input.auxdata.Iy;
 
%--------------------------------------------------------------------------
%                               ODEs 
%--------------------------------------------------------------------------
udot = Fx./m - g.*sin(theta) - q.*w;
wdot = Fz./m + g.*cos(theta) + q.*u;

qdot = My./Iy;

thetadot = q;

xdot = u.*cos(theta)  + w.*sin(theta);
zdot = -u.*sin(theta) + w.*cos(theta);

ElevDefdot = u_elev;
wIndot = u_wIn;

%--------------------------------------------------------------------------
%                     Objective Functional
%--------------------------------------------------------------------------
     
phaseout.integrand = (u_elev./2).^2 + (u_wIn./1000).^2;
phaseout.dynamics  = [udot, wdot, qdot, thetadot, xdot, zdot, ElevDefdot, wIndot];
                  