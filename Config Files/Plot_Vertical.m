close all
titlemod = '';


%%  Extract Solution from Output 

solution  = output.result.solution;
time      = solution.phase.time;

u         = solution.phase.state(:,1);
v         = zeros(length(u),1);
w         = solution.phase.state(:,2);
p         = zeros(length(u),1);
q         = solution.phase.state(:,3);
r         = zeros(length(u),1);
phi       = zeros(length(u),1);
theta     = solution.phase.state(:,4);
psi       = zeros(length(u),1);
x         = solution.phase.state(:,5);
y         = zeros(length(u),1);
z         = solution.phase.state(:,6);
Elev      = solution.phase.state(:,7);
Thrust    = solution.phase.state(:,8);

u_Elev    = solution.phase.control(:,1);
u_Thrust  = solution.phase.control(:,2);

% Wind-Axis

V = sqrt(u.^2 + v.^2 + w.^2);
alpha = atan(w./u);
beta = asin(v./V);
gamma = asin(cos(alpha).*cos(beta).*sin(theta)-sin(beta).*sin(phi).*cos(theta)...
       -sin(alpha).*cos(beta).*cos(theta).*cos(phi));


%% 3D Visualization

ManeuverTrajectory(time,x,y,z,theta,phi,psi,10,2)

%% Attitude 

% Euler Angles
figure
grid on;
sp1 = subplot(2,1,1);
plot(time, rad2deg([phi, theta, psi]));
title('Euler Angles');
legend('\phi', '\theta', '\psi');

% Wind-Axis Angles
grid on;
sp2 = subplot(2,1,2); 
plot(time, rad2deg([alpha, beta, gamma]));
title('Wind-Axis Angles');
legend('\alpha', '\beta', '\gamma');

linkaxes([sp1, sp2], 'x');

%% 3D Path

figure
grid on; hold on; 
plot3(x, y, z); 
plot3(x(1), y(1), z(1), '>');
set(gca,'Ydir','reverse'); set(gca,'Zdir','reverse');  axis equal;
hold off;
title('Path');
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');


%% Control Inputs

figure
sp1 = subplot(2,1,1); 
plot(time, rad2deg([Elev, u_Elev])); grid on;
title('Elevator');
legend('Deflection', 'Rate');

sp2 = subplot(2,1,2);
plot(time, [Thrust, u_Thrust]); grid on;
title('Thrust');
legend('Input', 'Rate');

linkaxes([sp1 sp2], 'x');

%% Speed

% Wind Velocity
figure
grid on;
sp1 = subplot(2,1,1);
plot(time, V);
title('V');

% Body-Frame Velocity
grid on;
sp2 = subplot(2,1,2); 
plot(time, [u, v, w]);
title('Body-Frame Velocity');
legend('u', 'v', 'w');

linkaxes([sp1, sp2], 'x');

