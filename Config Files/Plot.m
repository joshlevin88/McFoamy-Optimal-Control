close all
titlemod = '';


%%  Extract Solution from Output 

solution  = output.result.solution;
time      = solution.phase.time;

u         = solution.phase.state(:,1);
v         = solution.phase.state(:,2);
w         = solution.phase.state(:,3);
p         = solution.phase.state(:,4);
q         = solution.phase.state(:,5);
r         = solution.phase.state(:,6);
q1        = solution.phase.state(:,7);
q2        = solution.phase.state(:,8);
q3        = solution.phase.state(:,9);
q4        = solution.phase.state(:,10);
x         = solution.phase.state(:,11);
y         = solution.phase.state(:,12);
z         = solution.phase.state(:,13);
LAil      = solution.phase.state(:,14);
Elev      = solution.phase.state(:,15);
Rud       = solution.phase.state(:,16);
Thrust    = solution.phase.state(:,17);

u_LAil    = solution.phase.control(:,1);
u_Elev    = solution.phase.control(:,2);
u_Rud     = solution.phase.control(:,3);
u_Thrust  = solution.phase.control(:,4);

% Euler Angles

len = length(time);
phi = zeros(len,1); theta = zeros(len,1); psi = zeros(len,1);
for i = 1:len
    Eul = QuatToEul([q1(i) q2(i) q3(i) q4(i)]);
    phi(i) = Eul(1); theta(i) = Eul(2); psi(i) = Eul(3);
end

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
sp1 = subplot(4,1,1); 
plot(time, rad2deg([LAil, u_LAil])); grid on;
title('Aileron');
legend('Deflection', 'Rate');

sp2 = subplot(4,1,2); 
plot(time, rad2deg([Elev, u_Elev])); grid on;
title('Elevator');
legend('Deflection', 'Rate');

sp3 = subplot(4,1,3);
plot(time, rad2deg([Rud, u_Rud])); grid on;
title('Rudder');
legend('Deflection', 'Rate');

sp4 = subplot(4,1,4);
plot(time, [Thrust, u_Thrust]); grid on;
title('Thrust');
legend('Input', 'Rate');

linkaxes([sp1 sp2 sp3 sp4], 'x');

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

