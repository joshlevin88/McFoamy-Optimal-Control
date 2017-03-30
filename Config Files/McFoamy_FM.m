function [Fx, Fy, Fz, Mx, My, Mz] = McFoamy_FM(LAilDef,ElevDef,RudDef,wIn, u, v, w ,p, q, r)

rho = 1.225;

Vel = [u v w];
B_rate = [p q r];

%% AIRCRAFT GEOMETRY

%        x     y    z
CG = [-238.58, 0, 5.89]; %measured from propeller plane

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
WingSec7 = [61.90,  9914.07, 160.28,  87.49, -202.26, 400.21, 0];

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

Geom = [WingGeom; TailGeom; RudGeom; BodyGeom;...
        WingSec1; WingSec2; WingSec3; WingSec4; WingSec5; WingSec6; WingSec7;...
        TailSec1; TailSec2; TailSec3; TailSec4;...
        RudSec1; RudSec2; RudSec3; RudSec4;...
        BodySec1; BodySec2; BodySec3; BodySec4]';


CtrlDef = [-LAilDef, LAilDef, ElevDef, RudDef];

%% THRUSTER MODEL

D = PropGeom(2)*1e-3;
% PROPELLER MAPS FOR YAK54 (ELECTRIFLY 10X4.7 PROP)
% from thruster model
[J_pre, psiT_pre] = ndgrid(0:0.1:1,[0:10:90]*pi/180);

CFx_pre = [0.156336454	0.156336454	0.156336454	0.156336454	0.156336454	0.156336454	0.156336454	0.156336454	0.156336454	0.156336454
0.142939755	0.143274298	0.144236696	0.145715204	0.147592844	0.149730239	0.151960378	0.154120349	0.156099101	0.157836404
0.12449467	0.125294853	0.127662714	0.131491476	0.136533063	0.142227959	0.148100411	0.153647038	0.158176177	0.161723393
0.103654093	0.104978752	0.10890636	0.115283154	0.123846679	0.133924798	0.144287776	0.153875413	0.161071479	0.163852071
0.080437634	0.082320505	0.087900516	0.096990697	0.109258684	0.124116893	0.139693524	0.153953862	0.163349211	0.165797264
0.054966543	0.057426888	0.064729471	0.076657686	0.092816061	0.112598271	0.133992265	0.153533548	0.165846339	0.168474477
0.027444178	0.03049664	0.039588216	0.054473185	0.074693054	0.099537509	0.127172755	0.152570626	0.168638349	0.171699702
-0.001874328	0.001785855	0.012726044	0.030695378	0.05514811	0.085260174	0.119365045	0.151139537	0.171659606	0.175259993
-0.032728784	-0.02842523	-0.015539759	0.005645211	0.034482474	0.070037437	0.11072865	0.149362748	0.174874022	0.179027738
-0.064814458	-0.059769732	-0.044818343	-0.020318541	0.013005062	0.054122603	0.101419857	0.147360112	0.17827724	0.182929631
-0.097502535	-0.091725529	-0.074619748	-0.046776089	-0.008957689	0.037754768	0.091641937	0.145232828	0.181871525	0.18696977]';

CFy_pre = [0	0	0	0	0	0	0	0	0	0
0	0.000304035	0.000593774	0.000855707	0.001081293	0.001265997	0.001410344	0.001515466	0.001581189	0.00160638
0	0.000672049	0.001308186	0.001875984	0.002343891	0.002685232	0.002915524	0.003073033	0.003203695	0.003296003
0	0.001105525	0.002139399	0.003046099	0.003777072	0.004271303	0.004529052	0.004674445	0.004870829	0.005159087
0	0.001600104	0.003086457	0.004372415	0.005383801	0.006042688	0.006286839	0.006333996	0.006596278	0.007149694
0	0.00214306	0.004124254	0.005829393	0.007152135	0.007991019	0.008217389	0.008079318	0.008340642	0.00921631
0	0.002695169	0.005206331	0.007370573	0.009046337	0.010092937	0.010327588	0.009939438	0.010095901	0.01134185
0	0.003243617	0.0062742	0.008920992	0.010988567	0.012301024	0.012588497	0.01193205	0.011869291	0.013520997
0	0.003737499	0.007197636	0.010310416	0.012837704	0.014518175	0.014938663	0.014050094	0.013673189	0.015752967
0	0.004001698	0.007738064	0.011301405	0.014405295	0.016623841	0.017317246	0.016267158	0.015518142	0.018038805
0	0.003755281	0.007554928	0.011576152	0.015455208	0.018472368	0.019651635	0.018549771	0.017413651	0.020375896]';

CMx_pre = [-0.009379255	-0.009379255	-0.009379255	-0.009379255	-0.009379255	-0.009379255	-0.009379255	-0.009379255	-0.009379255	-0.009379255
-0.009414602	-0.009414258	-0.009412543	-0.009408081	-0.009399845	-0.009386461	-0.009367587	-0.009343565	-0.009317827	-0.009292775
-0.00922517	-0.009234779	-0.009259831	-0.009292432	-0.0093192	-0.009327092	-0.009306845	-0.009256742	-0.009186392	-0.009135945
-0.008715215	-0.008749876	-0.008846651	-0.008982547	-0.009121875	-0.009224484	-0.009256742	-0.009206982	-0.009150015	-0.009359351
-0.007778011	-0.007856254	-0.008075885	-0.008395035	-0.008745071	-0.009041916	-0.009209727	-0.009213159	-0.00927596	-0.009773903
-0.006305113	-0.0064465	-0.006846983	-0.007438956	-0.008111231	-0.008720706	-0.009127023	-0.00925331	-0.009427299	-0.01020081
-0.004198032	-0.004423154	-0.005068319	-0.006031948	-0.007148975	-0.008203888	-0.008967104	-0.009300668	-0.009571088	-0.010605068
-0.001374064	-0.001706255	-0.002664393	-0.004110867	-0.005807856	-0.007449251	-0.00869943	-0.009331211	-0.009705269	-0.010994226
0.002229593	0.001762192	0.00040563	-0.001648602	-0.004067627	-0.006438264	-0.008307869	-0.009330867	-0.009834645	-0.011382011
0.006644511	0.00599214	0.004131457	0.001338374	-0.001941328	-0.005174359	-0.007785904	-0.009294491	-0.009963678	-0.011781807
0.011790387	0.01091461	0.008431755	0.004772847	0.000516475	-0.003684646	-0.007140396	-0.009222768	-0.010097172	-0.012199449]';

CMy_pre = [0	0	0	0	0	0	0	0	0	0
0	0.000624918	0.001228902	0.001786557	0.002280383	0.002695622	0.003025754	0.003265632	0.003407706	0.003447857
0	0.00118875	0.002355195	0.003475311	0.004508604	0.005373056	0.006031948	0.006465718	0.006690153	0.006735452
0	0.001686008	0.003352112	0.004976005	0.006530234	0.007916309	0.008970193	0.009615701	0.009827095	0.009570745
0	0.00210811	0.004209014	0.006291729	0.008331891	0.010270131	0.01181784	0.012740975	0.012792794	0.012191899
0	0.002447508	0.004910802	0.007398462	0.009900534	0.012377212	0.01452513	0.015837766	0.015769475	0.014745447
0	0.002695622	0.005441004	0.008275611	0.011210425	0.014212843	0.017039214	0.018875875	0.018797288	0.017259188
0	0.002844902	0.005784863	0.008902245	0.012237884	0.015771877	0.019318224	0.021818238	0.021878979	0.019738611
0	0.002886426	0.005919044	0.009246104	0.012952713	0.017029948	0.021326814	0.024630882	0.025002195	0.022188521
0	0.00279274	0.005797904	0.009258801	0.013314074	0.017960633	0.023040962	0.027285666	0.028151834	0.024615439
0	0.002507907	0.005340455	0.008857975	0.013262255	0.018530643	0.024451059	0.029763374	0.031305935	0.027038239]';


CMz_pre = [0	0	0	0	0	0	0	0	0	0
0	0.000466715	0.000928969	0.001379898	0.001812295	0.002217926	0.002588209	0.002912507	0.003173662	0.003355543
0	0.000650999	0.001316754	0.002009277	0.002733028	0.003466731	0.004186021	0.004849031	0.005419041	0.005744369
0	0.000651342	0.001337687	0.002091295	0.002939961	0.003887461	0.004884035	0.005825014	0.00627663	0.005094056
0	0.000542556	0.001141049	0.001847642	0.002706947	0.003752251	0.004944433	0.006105387	0.005895365	0.003495558
0	0.000378176	0.000833567	0.001435835	0.002244007	0.003306126	0.004618762	0.005966058	0.005365163	0.001968096
0	0.000193549	0.00048456	0.000959511	0.001687037	0.002723076	0.004095081	0.005597491	0.004922813	0.000671589
0	7.5498E-06	0.000133151	0.000477353	0.001115998	0.002106738	0.003494871	0.005116363	0.004597142	-0.000392246
0	-0.00017193	-0.000201786	1.9904E-05	0.000571383	0.001510303	0.002888485	0.004593367	0.004365844	-0.001249149
0	-0.00033528	-0.000507209	-0.000396021	7.41253E-05	0.000958825	0.002311955	0.004069343	0.004200435	-0.001923827
0	-0.00047049	-0.000764589	-0.000754637	-0.000362734	0.000464313	0.001782782	0.003567967	0.004069686	-0.002446135]';

% Negative J data from Selig's database (for APC 10x4.7) % we don't use it
NegJ_pre = [-0.69 -0.62 -0.58 -0.52 -0.48 -0.44 -0.41 -0.37 -0.33 -0.29 -0.27 -0.26 -0.23 -0.18 -0.14];
NegCFx_pre = [0.24 0.208 0.189 0.17 0.157 0.144 0.135 0.13 0.11 0.1 0.09 0.1 0.11 0.12 0.11];

% CALCULATIONS
u = Vel(1);
v = Vel(2);
w = Vel(3);
p = B_rate(1);
q = B_rate(2);
r = B_rate(3);

% negative b/c it is from c.g. to propeller plane
xthr = -CG(1)*1e-3; 
ythr = -CG(2)*1e-3;
zthr = -CG(3)*1e-3;

vthr_x = u + q*zthr - r*ythr;
vthr_y = v + r*xthr - p*zthr;
vthr_z = w + p*ythr - q*xthr;

%-------------------------------------------------------------
V = sqrt(vthr_x^2 + vthr_y^2 + vthr_z^2); % total velocity
Vyz = sqrt(vthr_y^2 + vthr_z^2); % in-plane velocity
psiT = atan2(Vyz,abs(vthr_x)); % azimuth angle of thruster (0 to +90 deg.)
deltaT = atan2(vthr_z,vthr_y); % in-plane angle (0 to +/- 180)

if wIn < 1716 % Too slow to run the motor
    wOut = 0;
    J = 0;
    CFx = 0;
    CFy = 0;
    CFz = 0;
    CMx = 0;
    CMy = 0;
    CMz = 0;
else
    wOut = wIn;
    J = V/(wOut/60*D); % advance ratio based on total velocity
    
    if vthr_x >= 0 % Forward flight
        if J <= 1
            CFz = 0;
            g1 = griddedInterpolant(J_pre,psiT_pre,CFx_pre');
            CFx = g1(J, psiT);
            g2 = griddedInterpolant(J_pre,psiT_pre,CFy_pre');
            CFy = g2(J, psiT);
            g3 = griddedInterpolant(J_pre,psiT_pre,CMx_pre');
            CMx = g3(J, psiT);
            g4 = griddedInterpolant(J_pre,psiT_pre,CMy_pre');
            CMy = g4(J, psiT);
            g5 = griddedInterpolant(J_pre,psiT_pre,CMz_pre');
            CMz = g5(J, psiT);
        else
            FCFx = griddedInterpolant(psiT_pre(11,:), CFx_pre(:,11));
            CFx = FCFx(psiT);
            FCFy = griddedInterpolant(psiT_pre(11,:), CFy_pre(:,11));
            CFy = FCFy(psiT);
            CFz = 0;
            FCMx = griddedInterpolant(psiT_pre(11,:), CMx_pre(:,11));
            CMx = FCMx(psiT);
            FCMy = griddedInterpolant(psiT_pre(11,:), CMy_pre(:,11));
            CMy = FCMy(psiT);
            FCMz = griddedInterpolant(psiT_pre(11,:), CMz_pre(:,11));
            CMz = FCMz(psiT);
        end

    else % Descent flight
        % A/c to Bart's paper Fig.8 & 9, for descent flight, Cfx and Cmx have
        % approx. static value    
        CFx = 0.15633;
        CFy = 0;
        CFz = 0;
        CMx = -0.00938;
        g4 = griddedInterpolant(J_pre,psiT_pre,CMy_pre');
        CMy = g4(J, psiT);
        g5 = griddedInterpolant(J_pre,psiT_pre,CMz_pre');
        CMz = g5(J, psiT);
    end
end

% Transformation into UAV XYZ frame
CFX = CFx;
CFY = -CFy*cos(deltaT) + CFz*sin(deltaT);
CFZ = -CFy*sin(deltaT) - CFz*cos(deltaT);
CMX = CMx;
CMY = -CMy*cos(deltaT) + CMz*sin(deltaT);
CMZ = -CMy*sin(deltaT) - CMz*cos(deltaT);

% Aerodynamic Forces and Moments
FX = rho*(wOut/60)^2*D^4*CFX;
FY = rho*(wOut/60)^2*D^4*CFY;
FZ = rho*(wOut/60)^2*D^4*CFZ;
MX = rho*(wOut/60)^2*D^5*CMX;
MY = rho*(wOut/60)^2*D^5*CMY;
MZ = rho*(wOut/60)^2*D^5*CMZ;

% Induced Velocity at Propeller Plane
V0 = 1/2*(1.59*(wOut/60)*D*sqrt(0.15633)); % Induced velocity in hover

if vthr_x >= -0.2*V0 && CFX > 0 % if reverse velocity is greater than 20% of hover velocity
    Vi0_avg = 1/2*(1.59*(wOut/60)*D*sqrt(CFX));
    MX = 0.2*MX;
else
    Vi0_avg = 0;    
end

ThrForce = [FX,0.5.*FY,0.5.*FZ];

ThrMom = [MX,0.5.*MY,0.5.*MZ];

%% GYROSCOPIC MOMENTS
J_Rotor = 5e-5; % Rotor moment of inertia

My_Gyro = -J_Rotor*(wOut*2*pi/60)*B_rate(3);
Mz_Gyro = J_Rotor*(wOut*2*pi/60)*B_rate(2);

GyroMom = [0, My_Gyro, Mz_Gyro];

%% PROPELLER SLIPSTREAM MODEL
Rh = PropGeom(1)/2*1e-3;
Rp = PropGeom(2)/2*1e-3;

DesX = abs(Geom(5,5:end))*1e-3;
DesR = sqrt(Geom(6,5:end).^2 + Geom(7,5:end).^2)*1e-3;

VProp_Axial = zeros(1,length(DesX));
VProp_Swirl = zeros(1,length(DesX));

% Equations used from propeller slipstream paper
x0 = 1.528*Rp;   % Eq. 11
R0 = 0.74*Rp;   % Eq. 12
D0 = 2*R0;
Rm0 = 0.67*(R0 - Rh);
V0 = 2*Vi0_avg*(1.46/1.59); % Efflux velocity from Vi0_avg

for i = 1:length(DesX)
    if DesX(i) < x0
        VProp_Axial(i) = Vi0_avg*(1 + (DesX(i)/Rp)/sqrt(1 + (DesX(i)/Rp)^2));  % Eq. 1        

    elseif DesX(i) < 1.7*D0
        Vmax = V0*(1.24 - 0.0765*(DesX(i)-x0)/D0);  % Eq. 13
        Rm = Rm0*(1 - 0.1294*(DesX(i) - x0)/D0);    % Eq. 14
        VProp_Axial(i) = Vmax*exp(-((DesR(i)-Rm)/(0.8839*Rm0 + 0.1326*(DesX(i)-x0-R0)))^2);   % Eq. 15
    
    elseif DesX(i) < 4.25*D0
        Vmax = V0*(1.37 - 0.1529*(DesX(i)-x0)/D0);  % Eq. 16
        Rm = Rm0*(1.3 - 0.3059*(DesX(i) - x0)/D0);  % Eq. 17
        VProp_Axial(i) = Vmax*exp(-((DesR(i)-Rm)/(0.5176*Rm0 + 0.2295*(DesX(i)-x0-R0)))^2);   % Eq. 18
    
    else
        Vmax = V0*(0.89 - 0.04*(DesX(i)-x0)/D0);    % Eq. 19
        VProp_Axial(i) = Vmax*exp(-(DesR(i)/(0.2411*(DesX(i)-x0)))^2);    % Eq. 21
        
    end
    
    if VProp_Axial(i) < 0.01
        VProp_Axial(i) = 0; 
    end
end

%% AERODYNAMICS
V_wind = [0,0,0,0,0,0];

%--------------------------------------------------------------------------
%                Geometric properties of components
%--------------------------------------------------------------------------
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
Nw = 7; %Geom(1,1);
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
Nt = 4; %Geom(1,2);
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
Nr = 4; %Geom(1,3);
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
NB = 4; %Geom(1,4);
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

%--------------------------------------------------------------------------
%                       Aerodynamic Constants
%--------------------------------------------------------------------------
Cd0 = 0.02;
Cd90 = 1.98;
alp0 = 0;
% alpStallP = 10*pi/180;
% alpStallN = -10*pi/180;
% HighAlpStart = 22*pi/180;
% HighAlpEnd = 158*pi/180;
% StallMdl = 'FullStall';

%--------------------------------------------------------------------------
%                       Wind disturbances
%--------------------------------------------------------------------------
% Wind disturbance in body frame
Vw_u = V_wind(1);
Vw_v = V_wind(2);
Vw_w = V_wind(3);
Vw_p = V_wind(4);
Vw_q = V_wind(5);
Vw_r = V_wind(6);

%--------------------------------------------------------------------------
%     Body axis components of vehicle velocity relative to the air
%--------------------------------------------------------------------------
% Assuming wind velocity is in body frame
u = Vel(1)+ Vw_u;
v = Vel(2)+ Vw_v;
w = Vel(3)+ Vw_w;

p = B_rate(1)+ Vw_p;
q = B_rate(2)+ Vw_q;
r = B_rate(3)+ Vw_r;

%--------------------------------------------------------------------------
%              Slipstream velocities at reference points
%--------------------------------------------------------------------------
Vp_w = VProp_Axial(1:Nw);
Vp_t = VProp_Axial(Nw+1:Nw+Nt);
Vp_r = VProp_Axial(Nw+Nt+1:Nw+Nt+Nr);
Vp_B = VProp_Axial(Nw+Nt+Nr+1:Nw+Nt+Nr+NB);

Vp_w_swirl = VProp_Swirl(1:Nw);
Vp_t_swirl = VProp_Swirl(Nw+1:Nw+Nt);
Vp_r_swirl = VProp_Swirl(Nw+Nt+1:Nw+Nt+Nr);
Vp_B_swirl = VProp_Swirl(Nw+Nt+Nr+1:Nw+Nt+Nr+NB);

%--------------------------------------------------------------------------
%                             Control Inputs 
%--------------------------------------------------------------------------
RAilDef = CtrlDef(1);
LAilDef = CtrlDef(2);
ElevDef = CtrlDef(3);
RudDef = CtrlDef(4);

%--------------------------------------------------------------------------
%                   Wing, Tail, Rudder and Body velocities
%--------------------------------------------------------------------------
vws_xcomp = u + q*zw - r*yws + Vp_w;
vws_ycomp = v + r*xw - p*zw;
vws_zcomp = w + p*yws - q*xw - Vp_w_swirl;
vws = sqrt(vws_xcomp.^2 + vws_ycomp.^2 + vws_zcomp.^2);
vws_xz = sqrt(vws_xcomp.^2 + vws_zcomp.^2);

vwp_xcomp = u + q*zw - r*ywp + Vp_w;
vwp_ycomp = v + r*xw - p*zw;
vwp_zcomp = w + p*ywp - q*xw + Vp_w_swirl;
vwp = sqrt(vwp_xcomp.^2 + vwp_ycomp.^2 + vwp_zcomp.^2);
vwp_xz = sqrt(vwp_xcomp.^2 + vwp_zcomp.^2); 

vts_xcomp = u + q*zt - r*yts + Vp_t;
vts_ycomp = v + r*xt - p*zt;
vts_zcomp = w + p*yts - q*xt - Vp_t_swirl;
vts = sqrt(vts_xcomp.^2 + vts_ycomp.^2 + vts_zcomp.^2);
vts_xz = sqrt(vts_xcomp.^2 + vts_zcomp.^2);

vtp_xcomp = u + q*zt - r*ytp + Vp_t;
vtp_ycomp = v + r*xt - p*zt;
vtp_zcomp = w + p*ytp - q*xt + Vp_t_swirl;
vtp = sqrt(vtp_xcomp.^2 + vtp_ycomp.^2 + vtp_zcomp.^2);
vtp_xz = sqrt(vtp_xcomp.^2 + vtp_zcomp.^2);

vr_xcomp = u + q*zr - r*yr + Vp_r;
vr_ycomp = v + r*xr - p*zr + sign(zr + zCG).*Vp_r_swirl;
vr_zcomp = w + p*yr - q*xr;
vr = sqrt(vr_xcomp.^2 + vr_ycomp.^2 + vr_zcomp.^2);
vr_xy = sqrt(vr_xcomp.^2 + vr_ycomp.^2);

vB_xcomp = u + q*zB - r*yB + Vp_B;
vB_ycomp = v + r*xB - p*zB + sign(zB + zCG).*Vp_B_swirl;
vB_zcomp = w + p*yB - q*xB;
vB = sqrt(vB_xcomp.^2 + vB_ycomp.^2 + vB_zcomp.^2);
vB_xy = sqrt(vB_xcomp.^2 + vB_ycomp.^2);

%--------------------------------------------------------------------------
%              Wing, Tail, Rudder and Body angles of attack
%--------------------------------------------------------------------------
% Angle of attack = atan(V_zcomp / V_xcomp)
% range is -180 -> 180
a_ws = atan2(vws_zcomp,vws_xcomp);
a_wp = atan2(vwp_zcomp,vwp_xcomp);

a_ts = atan2(vts_zcomp,vts_xcomp);
a_tp = atan2(vtp_zcomp,vtp_xcomp);

% Vertical AoA for rudder and body = atan(V_ycomp / V_xcomp)
% range is -180 -> 180
a_r = atan2(vr_ycomp,vr_xcomp);
a_B  = atan2(vB_ycomp,vB_xcomp);
    
%--------------------------------------------------------------------------
%                   Wing lift and drag coefficient
%-------------------------------------------------------------------------- 
CN_ws = zeros(1,Nw);
CL_ws = zeros(1,Nw);
CD_ws = zeros(1,Nw);
CM_ws = zeros(1,Nw);

CN_wp = zeros(1,Nw);
CL_wp = zeros(1,Nw);
CD_wp = zeros(1,Nw);
CM_wp = zeros(1,Nw);

for i = 1:Nw
    [CN_ws(i),CL_ws(i),CD_ws(i),CM_ws(i)] = McFoamy_Airfoil_Simplified(i,a_ws(i),Cwf(i),Cw(i),RAilDef,AR_w,Cd0,Cd90,alp0);
    [CN_wp(i),CL_wp(i),CD_wp(i),CM_wp(i)] = McFoamy_Airfoil_Simplified(i,a_wp(i),Cwf(i),Cw(i),-RAilDef,AR_w,Cd0,Cd90,alp0);    
end

%--------------------------------------------------------------------------
%                  Tail's lift and drag coefficient
%--------------------------------------------------------------------------
CN_ts = zeros(1,Nt);
CL_ts = zeros(1,Nt);
CD_ts = zeros(1,Nt);
CM_ts = zeros(1,Nt);

CN_tp = zeros(1,Nt);
CL_tp = zeros(1,Nt);
CD_tp = zeros(1,Nt);
CM_tp = zeros(1,Nt);

for i = 1:Nt    
    [CN_ts(i),CL_ts(i),CD_ts(i),CM_ts(i)] = McFoamy_Airfoil_Simplified(i,a_ts(i),Ctf(i),Ct(i),ElevDef,AR_t,Cd0,Cd90,alp0);
    [CN_tp(i),CL_tp(i),CD_tp(i),CM_tp(i)] = McFoamy_Airfoil_Simplified(i,a_tp(i),Ctf(i),Ct(i),ElevDef,AR_t,Cd0,Cd90,alp0);
end

%--------------------------------------------------------------------------
%                  Rudder lift and drag coefficient
%--------------------------------------------------------------------------
CN_r = zeros(1,Nr);
CL_r = zeros(1,Nr);
CD_r = zeros(1,Nr);
CM_r = zeros(1,Nr);

for i = 1:Nr    % check sign of deflection into flapped code
    [CN_r(i),CL_r(i),CD_r(i),CM_r(i)] = McFoamy_Airfoil_Simplified(i,a_r(i),Crf(i),Cr(i),-RudDef,AR_r,Cd0,Cd90,alp0);
end

%--------------------------------------------------------------------------
%                  Fuselage lift and drag coefficient
%--------------------------------------------------------------------------
CN_B = zeros(1,NB);
CL_B = zeros(1,NB);
CD_B = zeros(1,NB);
CM_B = zeros(1,NB);

for i = 1:NB
     [CN_B(i),CL_B(i),CD_B(i),CM_B(i)] = McFoamy_Airfoil_Simplified(i,a_B(i),CBf(i),CB(i),0,AR_B,Cd0,Cd90,alp0);
end

%--------------------------------------------------------------------------
%                   Aerodynamic Forces & Moments
%--------------------------------------------------------------------------
% x-direction forces
Fx_ws = 0.5*rho*bw.*Cw.*vws_xz.^2.*(CL_ws.*sin(a_ws) - CD_ws.*cos(a_ws));
Fx_wp = 0.5*rho*bw.*Cw.*vwp_xz.^2.*(CL_wp.*sin(a_wp) - CD_wp.*cos(a_wp));
Fx_ts = 0.5*rho*bt.*Ct.*vts_xz.^2.*(CL_ts.*sin(a_ts) - CD_ts.*cos(a_ts));
Fx_tp = 0.5*rho*bt.*Ct.*vtp_xz.^2.*(CL_tp.*sin(a_tp) - CD_tp.*cos(a_tp));

Fx_r = 0.5*rho*br.*Cr.*vr_xy.^2.*(CL_r.*sin(a_r) - CD_r.*cos(a_r));
Fx_B = 0.5*rho*bB.*CB.*vB_xy.^2.*(CL_B.*sin(a_B) - CD_B.*cos(a_B));

% y-direction forces
Fy_ws = [0,0,0,0,0,0,0]; % INCLUDE FRICTION DRAG
Fy_wp = [0,0,0,0,0,0,0]; % INCLUDE FRICTION DRAG
Fy_ts = [0,0,0,0]; % INCLUDE FRICTION DRAG
Fy_tp = [0,0,0,0]; % INCLUDE FRICTION DRAG

Fy_r = 0.5*rho*br.*Cr.*vr_xy.^2.*(-CL_r.*cos(a_r) - CD_r.*sin(a_r));
Fy_B = 0.5*rho*bB.*CB.*vB_xy.^2.*(-CL_B.*cos(a_B) - CD_B.*sin(a_B));

% z-direction forces
Fz_ws = 0.5*rho*bw.*Cw.*vws_xz.^2.*(-CL_ws.*cos(a_ws) - CD_ws.*sin(a_ws));
Fz_wp = 0.5*rho*bw.*Cw.*vwp_xz.^2.*(-CL_wp.*cos(a_wp) - CD_wp.*sin(a_wp));
Fz_ts = 0.5*rho*bt.*Ct.*vts_xz.^2.*(-CL_ts.*cos(a_ts) - CD_ts.*sin(a_ts));
Fz_tp = 0.5*rho*bt.*Ct.*vtp_xz.^2.*(-CL_tp.*cos(a_tp) - CD_tp.*sin(a_tp));
Fz_r = [0,0,0,0]; % INCLUDE FRICTION DRAG
Fz_B = [0,0,0,0]; % INCLUDE FRICTION DRAG

% x-direction moments
Mx_ws = yws.*Fz_ws - zw.*Fy_ws;
Mx_wp = ywp.*Fz_wp - zw.*Fy_wp;
Mx_ts = yts.*Fz_ts - zt.*Fy_ts;
Mx_tp = ytp.*Fz_tp - zt.*Fy_tp;
Mx_r = yr.*Fz_r - zr.*Fy_r;
Mx_B = yB.*Fz_B - zB.*Fy_B;
Mx_Thr = yp*ThrForce(3) - zp*ThrForce(2);

% y-direction moments
My_ws = zw.*Fx_ws - xw.*Fz_ws + 0.5*rho*bw.*Cw.*vws_xz.^2.*Cw.*CM_ws;
My_wp = zw.*Fx_wp - xw.*Fz_wp + 0.5*rho*bw.*Cw.*vwp_xz.^2.*Cw.*CM_wp;
My_ts = zt.*Fx_ts - xt.*Fz_ts + 0.5*rho*bt.*Ct.*vts_xz.^2.*Ct.*CM_ts;
My_tp = zt.*Fx_tp - xt.*Fz_tp + 0.5*rho*bt.*Ct.*vtp_xz.^2.*Ct.*CM_tp;
My_r = zr.*Fx_r - xr.*Fz_r;
My_B = zB.*Fx_B - xB.*Fz_B;
My_Thr = zp*ThrForce(1) - xp*ThrForce(3);

% z-direction moments
Mz_ws = xw.*Fy_ws - yws.*Fx_ws;
Mz_wp = xw.*Fy_wp - ywp.*Fx_wp;
Mz_ts = xt.*Fy_ts - yts.*Fx_ts;
Mz_tp = xt.*Fy_tp - ytp.*Fx_tp;
Mz_r = xr.*Fy_r - yr.*Fx_r - 0.5*rho*br.*Cr.*vr_xy.^2.*Cr.*CM_r;  % WHY NEGATIVE SIGN???
Mz_B = xB.*Fy_B - yB.*Fx_B - 0.5*rho*bB.*CB.*vB_xy.^2.*CB.*CM_B;  % WHY NEGATIVE SIGN???
Mz_Thr = xp*ThrForce(2) - yp*ThrForce(1);

% Total forces and moments in x, y and z directions
Fx = sum(Fx_ws) + sum(Fx_wp) + sum(Fx_ts) + sum(Fx_tp) + sum(Fx_r) + sum(Fx_B) + ThrForce(1);
Fy = sum(Fy_ws) + sum(Fy_wp) + sum(Fy_ts) + sum(Fy_tp) + sum(Fy_r) + sum(Fy_B) + ThrForce(2);
Fz = sum(Fz_ws) + sum(Fz_wp) + sum(Fz_ts) + sum(Fz_tp) + sum(Fz_r) + sum(Fz_B) + ThrForce(3);

Mx = sum(Mx_ws) + sum(Mx_wp) + sum(Mx_ts) + sum(Mx_tp) + sum(Mx_r) + sum(Mx_B) + Mx_Thr + ThrMom(1) + GyroMom(1);
My = sum(My_ws) + sum(My_wp) + sum(My_ts) + sum(My_tp) + sum(My_r) + sum(My_B) + My_Thr + ThrMom(2) + GyroMom(2);
Mz = sum(Mz_ws) + sum(Mz_wp) + sum(Mz_ts) + sum(Mz_tp) + sum(Mz_r) + sum(Mz_B) + Mz_Thr + ThrMom(3) + GyroMom(3);
