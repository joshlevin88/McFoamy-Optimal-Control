clear; close all; clc

load V03
solution = output.result.solution;
u = solution.phase.state(end,1);
w = solution.phase.state(end,2);
V3 = sqrt(u^2 + w^2);
theta3 = solution.phase.state(end,4);
ElevDef3 = solution.phase.state(end,7);
wIn3 = solution.phase.state(end,8);

load V04
solution = output.result.solution;
u = solution.phase.state(end,1);
w = solution.phase.state(end,2);
V4 = sqrt(u^2 + w^2);
theta4 = solution.phase.state(end,4);
ElevDef4 = solution.phase.state(end,7);
wIn4 = solution.phase.state(end,8);

load V05
solution = output.result.solution;
u = solution.phase.state(end,1);
w = solution.phase.state(end,2);
V5 = sqrt(u^2 + w^2);
theta5 = solution.phase.state(end,4);
ElevDef5 = solution.phase.state(end,7);
wIn5 = solution.phase.state(end,8);

load V06
solution = output.result.solution;
time = solution.phase.time;
u = solution.phase.state(end,1);
w = solution.phase.state(end,2);
V6 = sqrt(u^2 + w^2);
theta6 = solution.phase.state(end,4);
ElevDef6 = solution.phase.state(end,7);
wIn6 = solution.phase.state(end,8);

load V07
solution = output.result.solution;
time = solution.phase.time;
u = solution.phase.state(end,1);
w = solution.phase.state(end,2);
V7 = sqrt(u^2 + w^2);
theta7 = solution.phase.state(end,4);
ElevDef7 = solution.phase.state(end,7);
wIn7 = solution.phase.state(end,8);

V = [V3, V4, V5, V6, V7]';
theta = [theta3, theta4, theta5, theta6, theta7]';
ElevDef = [ElevDef3, ElevDef4, ElevDef5, ElevDef6, ElevDef7]';
Thrust = [wIn3, wIn4, wIn5, wIn6, wIn7]';

plot(V, [theta*180/pi, ElevDef*180/pi, Thrust/100]);
legend('theta', 'elev', 'thrust');

