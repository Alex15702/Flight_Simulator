%% Ricerca delle condizioni di trim e verifica
clear all; clc; close all;
%Gamma nulla, velocità considerata orizzontale
disp('Moto del velivolo a 6 gradi di libertà');
%% Dichiarazione delle variabili globali
global  g phi0 theta0 psi0 p0 q0 r0 V0  myAC zEG_0 beta_der0 myAC aeroDat
%  Definizione della classe DSVAircraft e dell'oggetto 'Velivolo'
aircraftDataFileName = 'CITATION.txt';
myAC = DSVAircraft(aircraftDataFileName);
aeroDat = datcomimport("CitationM025.out");
%  Costanti e condizioni iniziali   
g = 9.81; %Accelerazione di gravità [m/s^2]
xEG_0 = 0; %[m]
yEG_0 = 0; %[m]
zEG_0 = -300; %Altitudine [m]
[air_Temp0,sound_speed0,air_pressure0,rho0] = atmosisa(-zEG_0);
phi0 = convang(10.000,'deg','rad'); %Angolo di bank
theta0 = convang(5,'deg','rad'); %Angolo di elevazione
psi0 = convang(0.000,'deg','rad'); %Angolo di heading
p0 = convangvel(0.000,'deg/s','rad/s'); %Velocità angolare di rollio
q0 = convangvel(0.000,'deg/s','rad/s'); %Velocità angolare di beccheggio
r0 = convangvel(0.000,'deg/s','rad/s'); %Velocità angolare di imbardata
M0 = 0.25; %Numero di Mach
V0 = M0*sound_speed0; %Velocità lineare del baricentro
%alpha_B0 = convang(4.557754,'deg','rad'); %Angolo di attacco valutato rispetto l'asse x Body
beta_der0 = convang(0,'deg','rad'); %Angolo di derapata

    
%% Ricerca dei valori di trim
%Vanno ricavati alphaB0,Deltaa0,Deltae0,Deltas0,DeltaT0.
%Importante: Flaps ignorati

% bounds
lb =[convang(-15,'deg','rad'), ... % minimum alpha_B
convang(-20,'deg','rad'), ... % minimum elevator deflection
convang( -10,'deg','rad'), ... % minimum aerelion incidence
convang( -10,'deg','rad'), ... % minimum rudder incidence
0.2 ... % minimum thrust fraction
convang(-5,'deg','rad')
];
ub =[convang(10,'deg','rad'), ... % maximum alpha_B
convang(20,'deg','rad'), ... % maximum elevator deflection
convang( 10,'deg','rad'), ... % maximum aerelion incidence
convang( 10,'deg','rad'), ... % maximum rudder incidence
1 ... % maximum thrust fraction
convang(5,'deg','rad')
];
options = optimset('tolfun',1e-9,'Algorithm','interior-point');
% I parametri da ricercare sono alpha_B, delta_e, delta_a, delta_r, delta_T
x0=[0,0,0,0,0,0];
[trim,j_val]=fmincon(@funzioneObiettivo6DOF, x0,[],[],[],[],lb,ub,@myNonLinearConstraint, options);

display(["alpha_B: " + rad2deg(trim(1)),"Beta: " + rad2deg(trim(6)) ,"delta_e: " + rad2deg(trim(2)),"delta_a: " + rad2deg(trim(3)),"delta_r: " + rad2deg(trim(4)),"delta_T: " + trim(5)]);

%% Verifica delle condizioni di trim

%  Assegnazione delle leggi temporali dei comandi di volo
global delta_a delta_e delta_r delta_T

alpha_B0=trim(1);

delta_e = trim(2);
delta_a = trim(3);

delta_r = trim(4);                 

delta_T = trim(5);

beta_der = trim(6);









%Verifico 20s
t_fin = 20;
state_0 = [V0,trim(1),beta_der,p0,q0,r0,xEG_0,yEG_0,zEG_0,phi0,theta0,psi0]; 


%  Integrazione del sistema di equazioni differenziali
options = odeset('RelTol',1e-9,'AbsTol',1e-9*ones(1,12));
[vTime,mState] = ode45(@eqLongDynamicStickFixed_6DoF_Vabpqr_Euler,[0 t_fin],state_0,options);


vVel_u = mState(:,1)*cos(mState(2))*cos(mState(3));
vVel_v = mState(:,2)*sin(mState(3));
vVel_w = mState(:,3)*sin(mState(2))*cos(mState(3));
vVel = mState(:,1);
vAlpha_B = rad2deg(mState(:,2));
vBeta = rad2deg(mState(:,3));
v_p = convangvel(mState(:,4),'rad/s','deg/s');
v_q = convangvel(mState(:,5),'rad/s','deg/s');
v_r = convangvel(mState(:,6),'rad/s','deg/s');
vXe = mState(:,7);
vYe = mState(:,8);
vZe = mState(:,9);
vPhi = convang(mState(:,10),'rad','deg');
vTheta = convang(mState(:,11),'rad','deg');
vPsi = convang(mState(:,12),'rad','deg');
vGamma = convang(asin(cos(vAlpha_B).*cos(vBeta).*sin(vTheta)-...
                sin(vBeta).*sin(vPhi).*cos(vTheta)-...
                sin(vAlpha_B).*cos(vBeta).*cos(vPhi).*cos(vTheta)),'rad','deg');
mQuat = zeros(length(vTime),4);
for i = 1:length(vTime)
    mQuat(i,:) = angle2quat(mState(i,12),mState(i,11),mState(i,10),'ZYX');
end 
 


%% Grafica

%Storie temporali delle variabili di stato
% figure(2)
% plot(vTime,vVel_u,'-.','color',[0.6350, 0.0780, 0.1840],'LineWidth',1.5);
% hold on;
% plot(vTime,vVel_v,'--','color',[0, 0.4470, 0.7410],'LineWidth',1.5);
% plot(vTime,vVel_w,':','color',[0.4660, 0.6740, 0.1880],'LineWidth',1.5);
% grid on
% lgd = legend('$u(t)$','$v(t)$','$w(t)$');
% lgd.Interpreter = 'latex'; 
% lgd.FontSize = 11;
% xlim([0 t_fin])
% xlabel('$t (s)$','interpreter','latex','fontsize',11);
% ylim([-30 150])
% ylabel('$(m/s)$','interpreter','latex','fontsize',11)

figure(3)
subplot 611
plot(vTime,vVel,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([80 130])
ylabel('$V (m/s)$','interpreter','latex','fontsize',11)
subplot 612
plot(vTime,vAlpha_B,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-5 10])
ylabel('$\alpha_{B} (deg)$','interpreter','latex','fontsize',11)
subplot 613
plot(vTime,vBeta,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-10 20])
ylabel('$\beta (deg)$','interpreter','latex','fontsize',11)
subplot 614
plot(vTime,v_p,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-5 10])
ylabel('$p (deg/s)$','interpreter','latex','fontsize',11)
subplot 615
plot(vTime,v_q,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-5 5])
ylabel('$q (deg/s)$','interpreter','latex','fontsize',11)
subplot 616
plot(vTime,v_r,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
xlabel('$t (s)$','interpreter','latex','fontsize',11);
ylim([-8 10])
ylabel('$r (deg/s)$','interpreter','latex','fontsize',11)
 
figure(4)
subplot 611
plot(vTime,vXe,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-4000 8000])
ylabel('$x_{EG} (m)$','interpreter','latex','fontsize',11)
subplot 612
plot(vTime,vYe,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-2500 8000])
ylabel('$y_{EG} (m)$','interpreter','latex','fontsize',11)
subplot 613
plot(vTime,-(vZe - zEG_0),'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-2000 500])
ylabel('$\Delta h (m)$','interpreter','latex','fontsize',11)
subplot 614
plot(vTime,vPhi,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-10 50])
ylabel('$\phi (deg)$','interpreter','latex','fontsize',11)
subplot 615
plot(vTime,vTheta,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-20 10])
ylabel('$\theta (deg)$','interpreter','latex','fontsize',11)
subplot 616
plot(vTime,vPsi,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
xlabel('$t (s)$','interpreter','latex','fontsize',11);
ylim([-200 400])
ylabel('$\psi (deg)$','interpreter','latex','fontsize',11)

% figure(5)
% plot(vTime,mQuat(:,1),'-.','color',[0.4940, 0.1840, 0.5560],'LineWidth',1.5);
% hold on;
% plot(vTime,mQuat(:,2),'-.','color',[0.6350, 0.0780, 0.1840],'LineWidth',1.5);
% plot(vTime,mQuat(:,3),'--','color',[0, 0.4470, 0.7410],'LineWidth',1.5);
% plot(vTime,mQuat(:,4),':','color',[0.4660, 0.6740, 0.1880],'LineWidth',1.5);
% grid on
% lgd = legend('$q_{0}(t)$','$q_{x}(t)$','$q_{y}(t)$','$q_{z}(t)$');
% lgd.Interpreter = 'latex'; 
% lgd.FontSize = 11;
% xlim([0 t_fin])
% ylim([-1 1.2])
% xlabel('$t (s)$','interpreter','latex','fontsize',11);
% ylabel('')

%% Visualizzazione
%% Traiettoria e modello
lenghtVectors =  length(vXe);
% Setup the figure/scene
h_fig1 = figure(7);
light('Position',[300 300 -3000],'Style','local');
% Trick to have Ze pointing downward and correct visualization
set(gca,'XDir','reverse'); set(gca,'ZDir','reverse');
grid on; hold on;
% Load aircraft shape
shapeScaleFactor = 50.0;
shape = loadAircraftMAT('Embraer-Phenom.mat', shapeScaleFactor); 
% Set the aircraft in place
% Posision in Earth axes

vXYZe = [vXe(:), vYe(:), vZe(:)]; 
% psi, theta, phi -> 'ZYX'
%Il modello .mat è strano e per questo ci sono sommati questi angoli nel
%vettore degli angoli, l'analisi è corretta
vEulerAngles=[deg2rad(vPsi), deg2rad(vTheta+180), deg2rad(-vPhi)];
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
options.samples=[round(lenghtVectors*0.1),round(lenghtVectors*0.3),round(lenghtVectors*0.5),round(lenghtVectors*0.7),round(lenghtVectors*0.9),round(lenghtVectors)];
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

%% Non linear constraint
function [c,ceq] = myNonLinearConstraint(x)
c = []; % Compute nonlinear inequalities at x.
ceq = []; % Compute nonlinear equalities at x.
end






