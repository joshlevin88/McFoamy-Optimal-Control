clear; close all; clc
tic

%-------------------------------------------------------------------%
%---------- Initialize all of the data for the problem -------------%
%-------------------------------------------------------------------%

Constants 

auxdata.m   = m;     
auxdata.g   = g;    
auxdata.Ixx = Ixx;
auxdata.Iyy = Iyy;
auxdata.Izz = Izz;
auxdata.Ixz = Ixz;

%-------------------------------------%
%             Problem Setup           %
%-------------------------------------% 

V0 = 5;

if (V0 == 3)
    theta0   = theta0_V3;
    ElevDef0 = ElevDef0_V3;
    Thrust0  = Thrust0_V3;
elseif (V0 == 5)
    theta0   = theta0_V5;
    ElevDef0 = ElevDef0_V5;
    Thrust0  = Thrust0_V5;
elseif (V0 == 7)
    theta0   = theta0_V7;
    ElevDef0 = ElevDef0_V7;
    Thrust0  = Thrust0_V7;
end

%----------------------------------------------------%
% Lower and Upper Limits on Time, State, and Control %
%----------------------------------------------------%

tfMin = 0;                  tfMax = 5;
uMin =  0;                  uMax = 20;
vMin = -20;                 vMax = 20; 
wMin = -20;                 wMax = 20;
pMin = -100;                pMax = 100;
qMin = -100;                qMax = 100;
rMin = -100;                rMax = 100;
q1Min = -1;                 q1Max = 1;
q2Min = -1;                 q2Max = 1;
q3Min = -1;                 q3Max = 1;
q4Min = -1;                 q4Max = 1;
xMin = -10;                 xMax = 10;
yMin = -10;                 yMax = 10;
zMin = -10;                 zMax = 10;

LAilMin = -0.8*AilSat;       LAilMax = 0.8*AilSat;
ElevMin = -0.8*ElevSat;      ElevMax = 0.8*ElevSat;
RudMin = -0.8*RudSat;        RudMax = 0.8*RudSat;
ThrustMin = ThrustSatMin;    ThrustMax =  0.8*ThrustSatMax;

bounds.phase.initialtime.lower = 0;
bounds.phase.initialtime.upper = 0;
bounds.phase.finaltime.lower = tfMin;
bounds.phase.finaltime.upper = tfMax;

bounds.phase.initialstate.lower = [uMin, vMin, wMin, 0, 0, 0, q1Min, q2Min, q3Min, q4Min, 0, 0, 0, 0, ElevDef0, 0, Thrust0];
bounds.phase.initialstate.upper = [uMax, vMax, wMax, 0, 0, 0, q1Max, q2Max, q3Max, q4Max, 0, 0, 0, 0, ElevDef0, 0, Thrust0];

bounds.phase.state.lower = [uMin, vMin, wMin, pMin, qMin, rMin, q1Min, q2Min, q3Min, q4Min, xMin, yMin, zMin, LAilMin, ElevMin, RudMin, ThrustMin];
bounds.phase.state.upper = [uMax, vMax, wMax, pMax, qMax, rMax, q1Max, q2Max, q3Max, q4Max, xMax, yMax, zMax, LAilMax, ElevMax, RudMax, ThrustMax];

bounds.phase.finalstate.lower = [uMin, vMin, wMin, 0, 0, 0, 0,     q2Min, q3Min, q4Min, 0, 0, 0, 0, ElevDef0, 0, Thrust0];
bounds.phase.finalstate.upper = [uMax, vMax, wMax, 0, 0, 0, q1Max, q2Max, q3Max, q4Max, 0, 0, 0, 0, ElevDef0, 0, Thrust0];

bounds.phase.control.lower = [-8.6, -6.3, -10.6, -10000];
bounds.phase.control.upper = [ 8.6,  6.3,  10.6,  10000];

bounds.phase.path.lower = [0.999]; 
bounds.phase.path.upper = [1]; 


bounds.eventgroup(1).lower = [V0, theta0, 0, 0, 0, 0];  %V0, alpha0, beta0, gamma0, phi0, psi0
bounds.eventgroup(1).upper = [V0, theta0, 0, 0, 0, 0]; 

bounds.eventgroup(2).lower = [V0, theta0, 0, 0, 0, pi]; 
bounds.eventgroup(2).upper = [V0, theta0, 0, 0, 0, pi]; 
                         
%----------------------%
% Set up Initial Guess %
%----------------------%


load V05
solution = output.result.solution;
states = solution.phase.state;
controls = solution.phase.control;

tGuess              = solution.phase.time;
uGuess              = states(:,1);
vGuess              = states(:,2);
wGuess              = states(:,3);
pGuess              = states(:,4);
qGuess              = states(:,5);
rGuess              = states(:,6);
q1Guess             = states(:,7);
q2Guess             = states(:,8);
q3Guess             = states(:,9);
q4Guess             = states(:,10);
xGuess              = states(:,11);
yGuess              = states(:,12);
zGuess              = states(:,13);
LAilGuess           = states(:,14);
ElevGuess           = states(:,15);
RudGuess            = states(:,16);
ThrustGuess         = states(:,17);
u_LAilGuess         = controls(:,1);
u_ElevGuess         = controls(:,2);
u_RudGuess          = controls(:,3);
u_ThrustGuess       = controls(:,4);

guess.phase.state   = [uGuess, vGuess, wGuess, pGuess, qGuess, rGuess, q1Guess, q2Guess, q3Guess, q4Guess, xGuess, yGuess, zGuess,...
                       LAilGuess, ElevGuess, RudGuess, ThrustGuess];
guess.phase.control = [u_LAilGuess, u_ElevGuess, u_RudGuess, u_ThrustGuess];
guess.phase.time    = tGuess;

bounds.phase.integral.lower = 0;
bounds.phase.integral.upper = 1000;
guess.phase.integral = 2;

%-----------------------------------------%
% Set up Initial Mesh - Aditya's Way      %
%-----------------------------------------%
meshphase.colpoints = 4*ones(1,10);
meshphase.fraction = 0.1*ones(1,10);

setup.name = 'Aggressive_Turn_Around';
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
% setup.mesh.method = 'hp1';
setup.mesh.tolerance = 1e-1; % default 1e-3
setup.mesh.colpointsmin = 4;
setup.mesh.colpointsmax = 16;

%----------------------------------%
% Solve Problem Using OptimalPrime %
%----------------------------------%

output = gpops2(setup);
toc
save('V05_More_Aggressive','output');