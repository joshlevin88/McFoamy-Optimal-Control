%MCFOAMY GEOMETRY

%        x     y    z
% CG = [-238.58, 0, 5.89]; %measured from propeller plane using CAD
CG = [-270, 0, 5.89]; % CG_x measured roughly by hand (picking plane up)

PropGeom = [12, 254];

%         #segs  wing root   wing tip  wing span
WingGeom = [7,    260.41,    152.40,    431.80, 0, 0, 0];
TailGeom = [3,    152.73,    139.70,    181.61, 0, 0, 0];
RudGeom =  [4,    204.01,    128.55,    215.90, 0, 0, 0];
BodyGeom = [4,    641.71,    641.71,    146.27, 0, 0, 0];

% Wing Sections 
%            Span    Area     Chord FlapChord   x     y     z
WingSec1 = [57.26, 14499.66, 253.40,      0, -217.07,  33.38, 0];
WingSec2 = [58.72, 13977.71, 237.58, 102.04, -211.39,  91.76, 0];
WingSec3 = [59.29, 13198.25, 222.69,  99.23, -215.43, 151.14, 0];
WingSec4 = [62.58, 12974.40, 207.42,  96.36, -212.18, 212.01, 0];
WingSec5 = [62.58, 11992.21, 191.74,  93.41, -208.85, 274.55, 0];
WingSec6 = [63.25, 11122.51, 175.97,  90.44, -205.50, 337.42, 0];
WingSec7 = [61.90,   9913.9, 160.28,  87.49, -202.26, 400.21, 0];

% Tail Sections 
%            Span    Area     Chord FlapChord   x     y     z
TailSec1 = [66.55, 7241.76, 117.41,  81.87, -711.27,  39.53, 0];
TailSec2 = [51.09, 7452.58, 145.64, 111.49, -719.72,  93.33, 0];
TailSec3 = [62.02, 8892.67, 143.16, 143.16, -715.15, 151.57, 0];

% Rudder Sections
%            Span    Area     Chord FlapChord   x     y     z
RudSec1 = [46.30,  9291.95, 200.62, 120.36, -733.94, 0,   23.54];
RudSec2 = [62.19, 11997.38, 192.95, 112.69, -735.81, 0,  -29.96];
RudSec3 = [69.31, 11715.82, 169.49, 103.43, -744.14, 0,  -94.63];
RudSec4 = [38.10,  5298.95, 141.14, 141.14, -760.76, 0, -148.56];

% Body Sections
%            Span    Area    Chord  FlapChord   x     y     z
BodySec1 = [35.87, 23018.14, 641.71,   0,   -205.09,  0,  53.81];
BodySec2 = [35.87, 23018.14, 641.71,   0,   -205.09,  0,  17.94];
BodySec3 = [37.26, 23910.11, 641.71,   0,   -205.09,  0, -18.63];
BodySec4 = [37.26, 23910.11, 641.71,   0,   -205.09,  0, -55.89];

Geom = [WingGeom; 
        TailGeom; 
        RudGeom; 
        BodyGeom;
        WingSec1; 
        WingSec2; 
        WingSec3; 
        WingSec4; 
        WingSec5; 
        WingSec6; 
        WingSec7;
        TailSec1; 
        TailSec2; 
        TailSec3;
        RudSec1;
        RudSec2; 
        RudSec3; 
        RudSec4;
        BodySec1; 
        BodySec2; 
        BodySec3; 
        BodySec4]';
    
% Geometric properties of components

% AIRCRAFT CG
% Measured w.r.t the nose (from propeller plane)
xCG = CG(1)*1e-3;
yCG = CG(2)*1e-3;
zCG = CG(3)*1e-3;

% THRUSTER
% Thrust is assumed to act in x direction only
xp = 0 - xCG;
yp = 0 - yCG;
zp = 0 - zCG;

% WING GEOMETRY
Nw = Geom(1,1);
Cw0 = Geom(2,1)*1e-3;
Cwe = Geom(3,1)*1e-3;
Bw = Geom(4,1)*1e-3;
AR_w = (Bw)/(0.5*(Cw0 + Cwe));  % Aspect ratio

bw = Geom(1,5:Nw+4)*1e-3;
Aw = Geom(2,5:Nw+4)*1e-6;
Cw = Geom(3,5:Nw+4)*1e-3;
Cwf = Geom(4,5:Nw+4)*1e-3;
xw = Geom(5,5:Nw+4)*1e-3- xCG;
yws = Geom(6,5:Nw+4)*1e-3 - yCG;
ywp = -yws;
zw = Geom(7,5:Nw+4)*1e-3 - zCG;

% TAIL GEOMETRY
Nt = Geom(1,2);
Ct0 = Geom(2,2)*1e-3;
Cte = Geom(3,2)*1e-3;
Bt = Geom(4,2)*1e-3;
AR_t = (Bt)/(0.5*(Ct0 + Cte));  % Aspect ratio

bt = Geom(1,Nw+5:Nw+Nt+4)*1e-3;
At = Geom(2,Nw+5:Nw+Nt+4)*1e-6;
Ct = Geom(3,Nw+5:Nw+Nt+4)*1e-3;
Ctf = Geom(4,Nw+5:Nw+Nt+4)*1e-3;
xt = Geom(5,Nw+5:Nw+Nt+4)*1e-3 - xCG;
yts = Geom(6,Nw+5:Nw+Nt+4)*1e-3 - yCG;
ytp = -yts;
zt = Geom(7,Nw+5:Nw+Nt+4)*1e-3 - zCG;
 
% RUDDER GEOMETRY
Nr = Geom(1,3);
Cr0 = Geom(2,3)*1e-3;
Cre = Geom(3,3)*1e-3;
Br = Geom(4,3)*1e-3;
AR_r = Br/(0.5*(Cr0 + Cre));  % Aspect ratio

br = Geom(1,Nw+Nt+5:Nw+Nt+Nr+4)*1e-3;
Ar = Geom(2,Nw+Nt+5:Nw+Nt+Nr+4)*1e-6;
Cr = Geom(3,Nw+Nt+5:Nw+Nt+Nr+4)*1e-3;
Crf = Geom(4,Nw+Nt+5:Nw+Nt+Nr+4)*1e-3;
xr = Geom(5,Nw+Nt+5:Nw+Nt+Nr+4)*1e-3 - xCG;
yr = Geom(6,Nw+Nt+5:Nw+Nt+Nr+4)*1e-3 - yCG;
zr = Geom(7,Nw+Nt+5:Nw+Nt+Nr+4)*1e-3 - zCG;

% BODY GEOMETRY
NB = Geom(1,4);
CB0 = Geom(2,4)*1e-3;
CBe = Geom(3,4)*1e-3;
BB = Geom(4,4)*1e-3;
AR_B = BB/(0.5*(CB0 + CBe));  % Aspect ratio

bB = Geom(1,Nw+Nt+Nr+5:Nw+Nt+Nr+NB+4)*1e-3;
AB = Geom(2,Nw+Nt+Nr+5:Nw+Nt+Nr+NB+4)*1e-6;
CB = Geom(3,Nw+Nt+Nr+5:Nw+Nt+Nr+NB+4)*1e-3;
CBf = Geom(4,Nw+Nt+Nr+5:Nw+Nt+Nr+NB+4)*1e-3;
xB = Geom(5,Nw+Nt+Nr+5:Nw+Nt+Nr+NB+4)*1e-3 - xCG;
yB = Geom(6,Nw+Nt+Nr+5:Nw+Nt+Nr+NB+4)*1e-3 - yCG;
zB = Geom(7,Nw+Nt+Nr+5:Nw+Nt+Nr+NB+4)*1e-3 - zCG;