clear; close all; clc
tic

%-------------------------------------------------------------------%
%---------- Initialize all of the data for the problem -------------%
%-------------------------------------------------------------------%
V0 = 7;

Constants 

auxdata.m   = m;    
auxdata.g   = g;    
auxdata.Iyy = Iyy;
auxdata.V0  = V0;

%----------------------------------------------------%
% Lower and Upper Limits on Time, State, and Control %
%----------------------------------------------------%


tf = 20;
uMin = -V0;                 uMax = V0;
wMin = -V0;                 wMax = V0;
qMin = deg2rad(-5);         qMax = deg2rad(5);
thetaMin = deg2rad(0);      thetaMax = deg2rad(50);
xMin = 0;                   xMax = V0*tf*1.5;
zMin = -10;                 zMax = 10;


ElevMin = -0.8*ElevSat;     ElevMax = 0.8*ElevSat;
ThrustMin = ThrustSatMin;   ThrustMax =  0.8*ThrustSatMax;


bounds.phase.initialtime.lower = 0;
bounds.phase.initialtime.upper = 0;
bounds.phase.finaltime.lower = tf;
bounds.phase.finaltime.upper = tf;

q0 = 0; x0 = 0; z0 = 0;
bounds.phase.initialstate.lower = [uMin, wMin, qMin, thetaMin, x0, z0, ElevMin, ThrustMin];
bounds.phase.initialstate.upper = [uMax, wMax, qMax, thetaMax, x0, z0, ElevMax, ThrustMax];

bounds.phase.state.lower = [uMin, wMin, qMin, thetaMin, xMin, zMin, ElevMin, ThrustMin];
bounds.phase.state.upper = [uMax, wMax, qMax, thetaMax, xMax, zMax, ElevMax, ThrustMax];

bounds.phase.finalstate.lower = [uMin, wMin, qMin, thetaMin, xMin, zMin, ElevMin, ThrustMin];
bounds.phase.finalstate.upper = [uMax, wMax, qMax, thetaMax, xMax, zMax, ElevMax, ThrustMax];

bounds.phase.control.lower = [0, 0];
bounds.phase.control.upper = [0, 0];
 
bounds.eventgroup.lower = V0; 
bounds.eventgroup.upper = V0;

%----------------------%
% Set up Initial Guess %
%----------------------%

load V07_Yak54
solution = output.result.solution;
states = solution.phase.state;
controls = solution.phase.control;

tGuess         = solution.phase.time;
uGuess         = states(:,1);
wGuess         = states(:,2);
qGuess         = states(:,3);
thetaGuess     = states(:,4);
xGuess         = states(:,5);
zGuess         = states(:,6);
ElevGuess      = states(:,7);
ThrustGuess    = states(:,8);
u_ElevGuess    = controls(:,1);
u_ThrustGuess  = controls(:,2);

guess.phase.state   = [uGuess, wGuess, qGuess, thetaGuess, xGuess, zGuess, ...
                       ElevGuess,ThrustGuess];
guess.phase.control = [u_ElevGuess,u_ThrustGuess];
guess.phase.time    = tGuess;


bounds.phase.integral.lower = 0;
bounds.phase.integral.upper = 1000;
guess.phase.integral = 10;

%-----------------------------------------%
% Set up Initial Mesh - Aditya's Way      %
%-----------------------------------------%
meshphase.colpoints = 4*ones(1,10);
meshphase.fraction = 0.1*ones(1,10);

setup.name = 'Level_Flight_Trims';
setup.functions.continuous = @Continuous;
setup.functions.endpoint   = @Endpoint;
setup.displaylevel = 2;
setup.auxdata = auxdata;
setup.mesh.phase = meshphase;
setup.bounds = bounds;
setup.guess = guess;
setup.nlp.solver = 'snopt';
setup.nlp.options.ipopt.linear_solver = 'ma57';
setup.derivatives.supplier = 'sparseCD';
setup.derivatives.derivativelevel = 'second';
setup.scales.method = 'automatic-bounds';
setup.method = 'RPM-Differentiation';
setup.mesh.tolerance = 1e-1; 
setup.mesh.colpointsmin = 4;
setup.mesh.colpointsmax = 16;

%----------------------------------%
% Solve Problem Using OptimalPrime %
%----------------------------------%

output = gpops2(setup);
toc
save('V07','output');