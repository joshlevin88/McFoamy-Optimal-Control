function output = Endpoint(input)

u0 = input.phase.initialstate(1);
v0 = input.phase.initialstate(2);
w0 = input.phase.initialstate(3);
q10 = input.phase.initialstate(:,7);
q20 = input.phase.initialstate(:,8);
q30 = input.phase.initialstate(:,9);
q40 = input.phase.initialstate(:,10);

V0 = sqrt(u0^2 + v0^2 + w0^2);
alpha0 = atan2(w0,u0);
beta0 = asin(v0/V0);
phi0 = atan2(2*(q10*q20 + q30*q40), q10^2 + q40^2 - q20^2 - q30^2);
theta0 = asin(2*(q10*q30 - q20*q40));
psi0 = atan2(2*(q10*q40 + q20*q30), q10^2 + q20^2 - q30^2 - q40^2);
gamma0 = cos(alpha0)*cos(beta0)*sin(theta0)-sin(beta0)*sin(phi0)*cos(theta0)...
         -sin(alpha0)*cos(beta0)*cos(theta0)*cos(phi0);

tf = input.phase.finaltime;
uf = input.phase.finalstate(1);
vf = input.phase.finalstate(2);
wf = input.phase.finalstate(3);
q1f = input.phase.finalstate(:,7);
q2f = input.phase.finalstate(:,8);
q3f = input.phase.finalstate(:,9);
q4f = input.phase.finalstate(:,10);

Vf = sqrt(uf^2 + vf^2 + wf^2);
alphaf = atan2(wf,uf);
betaf = asin(vf/Vf);
phif = atan2(2*(q1f*q2f + q3f*q4f), q1f^2 + q4f^2 - q2f^2 - q3f^2);
thetaf = asin(2*(q1f*q3f - q2f*q4f));
psif = atan2(2*(q1f*q4f + q2f*q3f), q1f^2 + q2f^2 - q3f^2 - q4f^2);
gammaf = cos(alphaf)*cos(betaf)*sin(thetaf)-sin(betaf)*sin(phif)*cos(thetaf)...
         -sin(alphaf)*cos(betaf)*cos(thetaf)*cos(phif);

output.eventgroup(1).event = [V0, alpha0, beta0, gamma0, phi0, psi0];
output.eventgroup(2).event = [Vf, alphaf, betaf, gammaf, phif, abs(psif)];
                        
output.objective = input.phase.integral + 2*tf;
