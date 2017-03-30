%% McFoamy Constants

% Geometry File
McFoamy_Geom

g = 9.81;

% Mass Properties
m = 0.484;

% Taken at hand-measured c.g.

Ixx = 0.003922;
Iyy = 0.015940;
Izz = 0.019340;

Iyx = 0.000303;
Ixz = 0.000441;
Izy = -0.000030;


% Control Input Saturations

AilSat        = 30*pi/180;
ElevSat       = 50*pi/180;
RudSat        = 50*pi/180;
ThrustSatMin  = 1716;
ThrustSatMax  = 11500;

% Level Trim Conditions

% V0 = 3
% theta0_V3    = 0.8135;
% ElevDef0_V3  = -0.4833;
% Thrust0_V3   = 4323;

% V0 = 5
theta0_V5    = 0.4137;
ElevDef0_V5  = -0.2648;
Thrust0_V5   = 3802;

%V0 = 7;
theta0_V7    = 0.2085;
ElevDef0_V7  = -0.1169;
Thrust0_V7   = 3732;

% Hover Conditions

Thrust_Hover = 4762;