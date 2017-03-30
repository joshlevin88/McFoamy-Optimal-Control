close all
%------------------------------%
% Extract Solution from Output %
%------------------------------%
load V03_further
solution = output.result.solution;
time = solution.phase(1).time;

u = solution.phase(1).state(:,1);
v = zeros(length(u),1);
w = solution.phase(1).state(:,2);
p = zeros(length(u),1);
q = solution.phase(1).state(:,3);
r = zeros(length(u),1);
theta = solution.phase(1).state(:,4);
x = solution.phase(1).state(:,5);
y = zeros(length(u),1);
z = solution.phase(1).state(:,6);
ElevDef = solution.phase(1).state(:,7);
wIn = solution.phase(1).state(:,8);

u_Elev = solution.phase(1). control(:,1);
u_wIn = solution.phase(1).control(:,2);

% Thrust_c = solution.phase(1).control(:,2);

phi = zeros(length(x),1);
psi = zeros(length(x),1);

pDeg = rad2deg(p);
qDeg = rad2deg(q);
rDeg = rad2deg(r);
phiDeg = rad2deg(phi);
thetaDeg = rad2deg(theta);
psiDeg = rad2deg(psi);

ElevDefDeg = rad2deg(ElevDef);

%---------------%
% Plot Solution %
%---------------%
y = zeros(length(x),1);
ManeuverTrajectory(time,x,y,z,theta,phi,psi,10,2)

% Calculate Angles

alpha = atan(w./u);
stall = 30*ones(length(alpha),1);
V = sqrt(u.^2 + v.^2 + w.^2);
beta = asin(v./V);
alphaDeg = rad2deg(alpha);
betaDeg = rad2deg(beta);

gamma = asin(cos(alpha).*cos(beta).*sin(theta)-sin(beta).*sin(phi).*cos(theta)-sin(alpha).*cos(beta).*cos(theta).*cos(phi));
gammaDeg = rad2deg(gamma);
mu = acos((sin(alpha).*sin(theta) + cos(alpha).*cos(theta).*cos(phi))./cos(gamma));
muDeg = rad2deg(mu);
sin_hdg = (cos(alpha).*cos(beta).*cos(theta).*sin(psi) + sin(beta).*(sin(phi).*sin(theta).*sin(psi) + cos(phi).*cos(psi)) + sin(alpha).*cos(beta).*(cos(phi).*sin(theta).*sin(psi) - sin(phi).*cos(psi)))./cos(gamma);
hdg = asin(sin_hdg);
% hdg = beta + psi; % approx?
hdgDeg = rad2deg(hdg);

for i = 26:length(hdgDeg)
    hdgDeg(i) = 90 + (90 - hdgDeg(i));
end

figure
plot3(x, y, z, 'LineWidth', 1.5)
hold on
plot3(x(1),y(1),z(1),'b>','LineWidth',2)
plot3(x(end),y(end),z(end),'b<','LineWidth',2)
set(gca,'Ydir','reverse')
set(gca,'Zdir','reverse')
hold off
grid on
axis equal
set(gca,'FontSize',20)
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
% saveas(gcf,'xyz.png');

figure
plot(time, alphaDeg, time, betaDeg, time, thetaDeg, time, gammaDeg, 'LineWidth',1.5)
grid on
axis normal
set(gca,'FontSize',20)
xlabel('Time [s]');
ylabel('Orientation [deg]');
legor = legend('\alpha','\beta','\theta', '\gamma', 'location', 'best');
% saveas(gcf,'angles.png');

figure
hold on
plot(time, ElevDefDeg, 'LineWidth', 1.5);
plot(time, u_Elev, 'b--', 'LineWidth', 1.5)
hold off
grid on
axis tight
set(gca,'FontSize',20)
xlabel('Time [s]');
ylabel('Elevator [deg]');
ylim([-60 60])
% saveas(gcf,'ContInp.png');

figure
plot(time, V, 'LineWidth', 1.5)
grid on
axis tight
set(gca,'FontSize',20)
xlabel('Time [s]');
ylabel('Velocity [m/s]')
% saveas(gcf,'V.png');

figure
% [Fx_T,Fy_T,Fz_T,Mx_T,My_T,Mz_T,wOut,J,Vi0_avg] = arrayfun(@Thrust_Model, Thrust, u, v, w, q, r);
hold on
plot(time, wIn, 'b', 'LineWidth', 1.5)
plot(time, u_wIn, 'b--', 'LineWidth', 1.5)
grid on
axis tight
set(gca,'FontSize',20)
xlabel('Time [s]');
legend('Thrust [RPM]')
% saveas(gcf,'T.png');

% figure
% plot(time, u, time, v, time, w, 'LineWidth', 1.5)
% grid on
% axis normal
% set(gca, 'FontSize', 20)
% xlabel('Time [s]');
% ylabel('Speed [m/s]');
% legend('u', 'v', 'w');
% % saveas(gcf, 'uvw.png');

% FlightAnime(x,y,z,phi,theta,psi)

% figure
% plot(time, hdgDeg, 'LineWidth',1.5)
% grid on
% axis tight
% set(gca,'FontSize',20)
% xlabel('Time [s]');
% ylabel('Orientation [deg]');
% set(gca,'FontSize',20)
% saveas(gcf,'hdg.png');

% figure(7)
% plot(x, -y, 'LineWidth', 1.5)
% hold on
% plot(x(1),-y(1),'b>','LineWidth',2)
% plot(x(end),-y(end),'b<','LineWidth',2)
% hold off
% grid on
% axis tight
% set(gca,'FontSize',20)
% xlabel('x [m]');
% ylabel('y [m]');
% legend('[1 1 5]', '[1 5 1]','location',[0.4 0.4 0.1 0.1]);
% saveas(gcf,'paths_xy.png');
% 
% figure(8) 
% plot(time, pDeg, time, rDeg, 'LineWidth',1.5)
% grid on
% axis tight
% set(gca,'FontSize',20)
% xlabel('Time [s]');
% ylabel('Angular Velocities [deg/s]');
% legend('p','r','location','best')
% saveas(gcf,'ang_vel.png');
