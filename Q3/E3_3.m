%% Esercitazione 3.3
%% Soluzione problema a valori iniziali, virata di Immlemann
clc; clear; close all;
global q p r VXE VYE VZE;
% angVel=[1,1,1];
% quat=[1,2,1,1];
% quatDer(angVel,quat)

%% Parametri simulazione


Nt=50;

%% Definizioni leggi velocità angolari
%Con questo metodo visualizzare manovre è immediato perchè si possono
%modificare i valori di pqr in maniera molto veloce nel signal editor


flightScenario = load('Scenario_Immlemann.mat');

T = max([max(flightScenario.p.Time),max(flightScenario.q.Time),max(flightScenario.r.Time)]);%Durata della simulazione
p = @(t) convangvel(interp1(flightScenario.p.Time,flightScenario.p.Data,t,'previous','extrap'),'deg/s','rad/s');
q = @(t) convangvel(interp1(flightScenario.q.Time,flightScenario.q.Data,t,'previous','extrap'),'deg/s','rad/s');
r = @(t) convangvel(interp1(flightScenario.r.Time,flightScenario.r.Data,t,'previous','extrap'),'deg/s','rad/s');
timeBrPoints=[0*T,0.25*T,0.5*T,0.75*T,1*T];
 
% qmax=convang(2.5,'deg','rad');
% qBrPoints=[0*qmax,1*qmax,1*qmax,1*qmax,0*qmax];
% q=@(t) interp1(timeBrPoints,qBrPoints,t,'pchip');
% 
% pmax=convang(5,'deg','rad');
% pBrPoints=[0*pmax,1*pmax,1*pmax,1*pmax,0*pmax];
% p=@(t) interp1(timeBrPoints,pBrPoints,t,'pchip');

% t=linspace(0,T,60);
% leggeAng=figure(1);
% plot(t,q(t));
% title("Velocità angolare q (rad/s)");

%% Definizione leggi velocità assi body
u0 = convvel(380.0,'km/h','m/s');
v0 = convvel( 0.0,'km/h','m/s');
w0 = convvel( 0.0,'km/h','m/s');

uBrPoints=[1*u0,0.7*u0,0.7*u0,0.7*u0,1*u0];
u=@(t) interp1(timeBrPoints,uBrPoints,t,'pchip');


%% Valori iniziali
%Posizioni
PositionsE0=[0, 0, -1000];
%Angoli
anglesE0=[deg2rad(0.1),0,0]; %Psi,theta,phi
anglesE0Q=angle2quat(anglesE0(1), anglesE0(2), anglesE0(3), 'ZYX');



%% Integrazione del problema
timeBrPointsRK=linspace(0,T,Nt);
VE=zeros(Nt,3);
options = odeset('RelTol',1e-15,'AbsTol',1e-15*ones(1,4));
[Times,quaternion]=ode89(@quatDer,timeBrPointsRK,anglesE0Q,options);
%Velocità
for it=1:Nt
    t=Times(it);
    q0=quaternion(it,1);
    qx=quaternion(it,2);
    qy=quaternion(it,3);
    qz=quaternion(it,4);
    MatV=zeros(3);
    MatV(1,:)=[(q0^2 + qx^2 - qy^2 - qz^2), 2*(qx*qy - q0*qz), 2*(qz*qx + q0*qy)];
    MatV(2,:)=[2*(qx*qy + q0*qz), (q0^2 - qx^2 + qy^2 - qz^2), 2*(qy*qz - q0*qx)];
    MatV(3,:)=[2*(qz*qx - q0*qy), 2*(qy*qz + q0*qx), (q0^2 - qx^2 - qy^2 + qz^2)];
    utemp=u(t);
    VB=[utemp; 0; 0];
    VE(it,:)=MatV*VB;
end

%Interpolazione e creazione delle leggi della velocità in assi terra
VXE=VE(:,1)';
VYE=VE(:,2)';
VZE=VE(:,3)';

VXE=@(t) interp1(Times,VXE,t,'');
VYE=@(t) interp1(Times,VYE,t,'pchip');
VZE=@(t) interp1(Times,VZE,t,'pchip');
%Integrazione con RK sulle VE per ottenere le posizioni

[Times,positions]=ode45(@posDer,timeBrPointsRK,PositionsE0);



%% Plotting
%Angoli
plotAngoli = figure(1);
title("Angoli assi terra")
[psi, theta, phi]=quat2angle(quaternion,'ZYX'); 



% psi=convang(psi,'rad','deg');
% theta=convang(theta,'rad','deg');
% phi=convang(phi,'rad','deg');
plot(Times,rad2deg(psi), Times,rad2deg(theta), Times, rad2deg(phi));
legend("Psi","Theta","Phi");
grid on;
%Posizioni
plotPos = figure(2);
title("Posizione assi terra")
plot(Times,positions(:,1),Times,positions(:,2),Times,-positions(:,3));
legend("X","Y","Z");
grid on;


% Setup the figure/scene
h_fig1 = figure(3);
light('Position',[300 300 -3000],'Style','local');
% Trick to have Ze pointing downward and correct visualization
set(gca,'XDir','reverse'); set(gca,'ZDir','reverse');
grid on; hold on;
% Load aircraft shape
shapeScaleFactor = 200.0;
shape = loadAircraftMAT('aircraft_mig29.mat', shapeScaleFactor); 
% Set the aircraft in place
% Posision in Earth axes

vXYZe = [positions(:,1), positions(:,2), positions(:,3)]; 
% psi, theta, phi -> 'ZYX'
vEulerAngles=[psi, theta, phi];
% Observer point-of-view
theView = [105 15];
% body axes settings
bodyAxesOptions.show = true; 
bodyAxesOptions.magX = 2.0*shapeScaleFactor;
bodyAxesOptions.magY = 2.0*shapeScaleFactor;
bodyAxesOptions.magZ = 1.5*shapeScaleFactor;
bodyAxesOptions.lineWidth = 2.5;


% Plot Earth axes
hold on;
xMax = 1;
yMax = 1;
zMax = 1; % max([abs(max(vXYZe(1))),0.18*xMax]);
vXYZ0 = [0,0,0];
vExtent = [xMax,yMax,zMax];
plotEarthAxes(h_fig1, vXYZ0, vExtent);
options.samples=[1,10,20,30,40,50];
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

%% Funzioni

function dqdt = quatDer(t,quatNorm)
    %angVel = [p, q, r]
    global q p r;
    quatNorm=quatNorm(:);
    pa=p(t);
    qa=q(t);
    ra=r(t);
    Mat=zeros(4);
    Mat(1,:)=[0, -pa, -qa, -ra];
    Mat(2,:)=[pa, 0, ra, -qa];
    Mat(3,:)=[qa,-ra,0,pa];
    Mat(4,:)=[ra,qa,-pa,0];
    dqdt=0.5*Mat*quatNorm;


end

function dPosdt = posDer(t,~)
    global VXE VYE VZE
    dPosdt=[VXE(t);VYE(t);VZE(t)];
end