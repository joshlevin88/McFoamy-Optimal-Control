clear; close all; clc
tic

%-------------------------------------------------------------------%
%---------- Initialize all of the data for the problem -------------%
%-------------------------------------------------------------------%

Constants 

auxdata.m   = m;     
auxdata.g   = g;    
auxdata.Iyy = Iyy;

%-------------------------------------%
%             Problem Setup           %
%-------------------------------------% 

V0 = 7;

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


tfMin = 0;                   tfMax = 5;
uMin = -2*V0;                uMax = 2*V0;
wMin = -2*V0;                wMax = 2*V0;
qMin = deg2rad(-500);        qMax = deg2rad(500);
thetaMin = deg2rad(-180);    thetaMax = deg2rad(180);
xMin = 0;                    xMax = 5;
zMin = -2;                   zMax = 2;


ElevMin = -0.8*ElevSat;      ElevMax = 0.8*ElevSat;
ThrustMin = ThrustSatMin;    ThrustMax =  0.8*ThrustSatMax;


bounds.phase.initialtime.lower = 0;
bounds.phase.initialtime.upper = 0;
bounds.phase.finaltime.lower = tfMin;
bounds.phase.finaltime.upper = tfMax;

u0 = V0*cos(theta0); w0 = V0*sin(theta0); q0 = 0; x0 = 0; z0 = 0; 
bounds.phase.initialstate.lower = [u0, w0, q0, theta0, x0, z0, ElevDef0, Thrust0];
bounds.phase.initialstate.upper = [u0, w0, q0, theta0, x0, z0, ElevDef0, Thrust0];

bounds.phase.state.lower = [uMin, wMin, qMin, thetaMin, xMin, zMin, ElevMin, ThrustMin];
bounds.phase.state.upper = [uMax, wMax, qMax, thetaMax, xMax, zMax, ElevMax, ThrustMax];

bounds.phase.finalstate.lower = [0, 0, 0, deg2rad(90), xMin, zMin, deg2rad(-3), Thrust_Hover-2];
bounds.phase.finalstate.upper = [0, 0, 0, deg2rad(90), xMax, zMax, deg2rad(3),  Thrust_Hover+2];

bounds.phase.control.lower = [-6.3, -10000];
bounds.phase.control.upper = [ 6.3,  10000];
 
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

setup.name = 'Cruise_to_Hover';
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