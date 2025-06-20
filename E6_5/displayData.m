clc; clear; close all;

simulation = sim("E6_5.slx");
alphaB = simulation.yout.getElement('alphaB');
alphaB=alphaB.Values;
delta = simulation.yout.getElement('delta');
delta=delta.Values;
gamma = simulation.yout.getElement('gamma');
gamma=gamma.Values;
ni = simulation.yout.getElement('ni');
ni=ni.Values;
Xe = simulation.yout.getElement('xg');
Xe=Xe.Values;
Ye = simulation.yout.getElement('yg');
Ye=Ye.Values;
Ze = simulation.yout.getElement('hg');
Ze=Ze.Values;
T=simulation.yout.getElement('T');
T=T.Values;
L=simulation.yout.getElement('L');
L=L.Values;
V=simulation.yout.getElement('V');
V=V.Values;

Tc=simulation.yout.getElement('TComando');
Tc=Tc.Values;
Tmax=simulation.yout.getElement('Tmax');
Tmax=Tmax.Values;
D=simulation.yout.getElement('D');
D=D.Values;
Vinf=simulation.yout.getElement('Vinfinito');
Vinf=Vinf.Values;
Lc=simulation.yout.getElement('LComando');
Lc=Lc.Values;
Vc=simulation.yout.getElement('VComando');
Vc=Vc.Values;

%Devo convertire gli angoli dati dalla simulazione in phi theta e psi
%Rotazione 321
%Angoli in gradi
niInterp=@(t) interp1(ni.Time,ni.Data, t,"linear");
gammaInterp=@(t) interp1(gamma.Time,gamma.Data, t,"linear");
deltaInterp=@(t) interp1(delta.Time,delta.Data,t,"linear");
XeInterp=@(t) interp1(Xe.Time,Xe.Data,t,"linear");
YeInterp=@(t) interp1(Ye.Time,Ye.Data,t,"linear");
ZeInterp=@(t) interp1(Ze.Time,-Ze.Data,t,"linear");
LInterp=@(t) interp1(L.Time,L.Data,t,"linear");
TInterp=@(t) interp1(T.Time,T.Data,t,"linear");
VInterp=@(t) interp1(V.Time,V.Data,t,"linear");
alphaBInterp=@(t) interp1(alphaB.Time,alphaB.Data,t,"linear");
TcInterp=@(t) interp1(Tc.Time,Tc.Data,t,"linear");
TmaxInterp=Tmax.Data;
DInterp=@(t) interp1(D.Time,D.Data,t,"linear");
VinfInterp=@(t) interp1(Vinf.Time,Vinf.Data,t,"linear");
LcInterp=@(t) interp1(Lc.Time,Lc.Data,t,"linear");
VcInterp=@(t) interp1(Vc.Time,Vc.Data,t,"linear");



%% Plotting
t_fin = simulation.SimulationMetadata.ModelInfo.StopTime*0.99;
Nt=500;
timeSamplesDisplay = linspace(0,t_fin,Nt);

figure(7)
subplot(5,1,1);
title("Trust and drag");
hold on;
yline(TmaxInterp,LineStyle='--',LineWidth=1);
plot(timeSamplesDisplay, TInterp(timeSamplesDisplay),LineWidth=1);
plot(timeSamplesDisplay, TcInterp(timeSamplesDisplay),LineStyle='--',LineWidth=1);
plot(timeSamplesDisplay, DInterp(timeSamplesDisplay),LineWidth=1);
legend("Tmax","T","Tc","D");
grid on;

subplot(5,1,2);
title("Speed and airspeed");
hold on;
plot(timeSamplesDisplay, VcInterp(timeSamplesDisplay),LineStyle='--',LineWidth=1);
plot(timeSamplesDisplay, VInterp(timeSamplesDisplay),LineWidth=1);
plot(timeSamplesDisplay, VinfInterp(timeSamplesDisplay),LineWidth=1);
legend("Vc","V","Vinf");
grid on;

subplot(5,1,3);
title("Lift");
hold on;
plot(timeSamplesDisplay, LcInterp(timeSamplesDisplay),LineStyle='--',LineWidth=1);
plot(timeSamplesDisplay, LInterp(timeSamplesDisplay),LineWidth=1);
legend("Lc","L");
grid on;

subplot(5,1,4);
title("Angle of attach and gamma");
hold on;
plot(timeSamplesDisplay, rad2deg(alphaBInterp(timeSamplesDisplay)),LineWidth=1);
plot(timeSamplesDisplay, gammaInterp(timeSamplesDisplay),LineWidth=1);
legend("\alpha_B","\gamma");
grid on;

subplot(5,1,5);
title("Rate of climb");
hold on;
plot(timeSamplesDisplay, VInterp(timeSamplesDisplay).*deg2rad(gammaInterp(timeSamplesDisplay)),LineWidth=1);
grid on;

%% Setup the figure/scene
h_fig1 = figure(4);
light('Position',[1000 2000 -6000],'Style','local');
% Trick to have Ze pointing downward and correct visualization
set(gca,'XDir','reverse'); set(gca,'ZDir','reverse');
grid on; hold on;
%% Load aircraft shape
shapeScaleFactor = 1400.0;
shape = loadAircraftMAT('Embraer-Phenom.mat', shapeScaleFactor); 
%% Set the aircraft in place
% Posision in Earth axes
Times=[0, 0.2*t_fin, 0.4*t_fin, 0.6*t_fin, 0.8*t_fin,t_fin]';
vXYZe = [XeInterp(Times), YeInterp(Times), ZeInterp(Times)]; 
% psi, theta, phi -> 'ZYX'
%Il in gamma 180 serve perchè il modello è sfasato per qualche motivo di 180 gradi
vEulerAngles=deg2rad([deltaInterp(Times), gammaInterp(Times)+180, -niInterp(Times)]);
% Observer point-of-view
theView = [1005 150];
% body axes settings
bodyAxesOptions.show = true; 
bodyAxesOptions.magX = 20.0*shapeScaleFactor;
bodyAxesOptions.magY = 20.0*shapeScaleFactor;
bodyAxesOptions.magZ = 10.5*shapeScaleFactor;
bodyAxesOptions.lineWidth = 2.5;


%% Plot Earth axes
hold on;
xMax = 1;
yMax = 1;
zMax = 1; % max([abs(max(vXYZe(1))),0.18*xMax]);
vXYZ0 = [0,0,0];
vExtent = [xMax,yMax,zMax];
plotEarthAxes(h_fig1, vXYZ0, vExtent);
options.samples=[1,2,3,4,5,6];
options.theView=theView;
options.bodyAxes.show = 1; 
options.bodyAxes.magX = 2.0*shapeScaleFactor;
options.bodyAxes.magY = 2.0*shapeScaleFactor;
options.bodyAxes.magZ = 2.0*shapeScaleFactor;
options.bodyAxes.lineWidth = 2.5;
options.helperLines.show= 1;
options.helperLines.lineColor='k';
options.helperLines.lineWidth=1.0;
options.helperLines.lineStyle='-';
options.trajectory.show = 1;
options.trajectory.lineColor = 'blue';
options.trajectory.lineWidth=1.0;
options.trajectory.lineStyle = '--';
plotTrajectoryAndBodyE(h_fig1,shape,vXYZe,vEulerAngles, options);












