%% Simulazione del moto a 6-DOf a partire da condizioni di trim
clear all; close all; clc;

disp('Moto del velivolo a 6 gradi di libertà');
%% Dichiarazione delle variabili globali
global g...                  %Accelerazione di gravità 
       myAC...
       aeroDat

%  Definizione della classe DSVAircraft e dell'oggetto 'Velivolo'
aircraftDataFileName = 'CITATION.txt';
myAC = DSVAircraft(aircraftDataFileName);
aeroDat=datcomimport("CitationM025.out");




%  Costanti e condizioni iniziali   
g = 9.81; %Accelerazione di gravità [m/s^2]
xEG_0 = 0; %[m]
yEG_0 = 0; %[m]
zEG_0 = -457; %Altitudine [m]
[air_Temp0,sound_speed0,air_pressure0,rho0] = atmosisa(-zEG_0);
phi0 = convang(0.000,'deg','rad'); %Angolo di bank
theta0 = convang(4.460,'deg','rad'); %Angolo di elevazione
psi0 = convang(0.000,'deg','rad'); %Angolo di heading
p0 = convangvel(0.000,'deg/s','rad/s'); %Velocità angolare di rollio
q0 = convangvel(0.000,'deg/s','rad/s'); %Velocità angolare di beccheggio
r0 = convangvel(0.000,'deg/s','rad/s'); %Velocità angolare di imbardata
M0 = 0.25; %Numero di Mach
V0 = M0*sound_speed0; %Velocità lineare del baricentro
alpha_B0 = convang(3,'deg','rad'); %Angolo di attacco valutato rispetto l'asse x Body
beta_der0 = convang(0.000,'deg','rad'); %Angolo di derapata
u0 = V0*cos(beta_der0)*cos(alpha_B0); %Componente della velocità lineare del baricentro lungo l'asse x Body
v0 = V0*sin(beta_der0); %Componente della velocità lineare del baricentro lungo l'asse y Body
w0 = V0*cos(beta_der0)*sin(alpha_B0); %Componente della velocità lineare del baricentro lungo l'asse z Body


    
%% Definizione delle leggi di comando
%  Assegnazione delle leggi temporali dei comandi di volo
global delta_a delta_e delta_r delta_T delta_f

%Uso il signal editor per definire i comandi
flightScenario = load('Virata.mat');

delta_a =  @(t) deg2rad(interp1(flightScenario.delta_a.Time,flightScenario.delta_a.Data,t,'previous','extrap'));
delta_e =  @(t) deg2rad(interp1(flightScenario.delta_e.Time,flightScenario.delta_e.Data,t,'previous','extrap'));
delta_r =  @(t) deg2rad(interp1(flightScenario.delta_r.Time,flightScenario.delta_r.Data,t,'previous','extrap'));
delta_f =  @(t) deg2rad(interp1(flightScenario.delta_f.Time,flightScenario.delta_f.Data,t,'previous','extrap'));
delta_T =  @(t) interp1(flightScenario.delta_t.Time,flightScenario.delta_t.Data,t,'previous','extrap');
t_fin =110;% max([flightScenario.delta_a.Time(end),flightScenario.delta_e.Time(end),flightScenario.delta_r.Time(end), ...
    %flightScenario.delta_f.Time(end),flightScenario.delta_t.Time(end)]);
%  Comandi di volo iniziali
delta_a0 = delta_a(0);
delta_e0 = delta_e(0);
delta_r0 = delta_r(0);
delta_f0 = delta_f(0);
delta_T0 = delta_T(0);

%Stato iniziale
state_0 = [V0,alpha_B0,beta_der0,p0,q0,r0,xEG_0,yEG_0,zEG_0,phi0,theta0,psi0]; 
%% Eliminato
% delta_e_exc1 = convang(0,'deg','rad');
% delta_e_exc2 = convang(0,'deg','rad');
% delta_e = @(t) interp1([0, 20, 80, 180, t_fin],...
%                        [delta_e0, delta_e0, delta_e_exc1, delta_e_exc2, delta_e_exc2],t,'linear');
% 
% delta_r_exc1 = convang(10,'deg','rad')*0;
% delta_r = @(t) interp1([0, 200, 203, 205, 208, t_fin],...
%                        [delta_r0, delta_r0, delta_r_exc1, delta_r_exc1, delta_r0, delta_r0],t,'linear');                  
% 
% delta_T_exc1 = 0.43;
% delta_T = @(t) interp1([0, t_fin],[delta_T0 delta_T_exc1],t,'linear');
% 
% delta_f_exc1 = convang(0,'deg','rad');
% delta_f = @(t) interp1([0, 40, 41, 45, 46, t_fin],...
%                        [delta_f0, delta_f0, delta_f_exc1, delta_f_exc1, delta_f0, delta_f0],t,'linear');


%  Integrazione del sistema di equazioni differenziali
options = odeset('RelTol',1e-6,'AbsTol',1e-6*ones(1,12));
[vTime,mState] = ode45(@eqLongDynamicStickFixed_6DoF_Vabpqr_Euler,[0 t_fin],state_0,options);

%% Risultati in vettori

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
%  Diagrammi dei comandi di volo
figure(1)
subplot 411
plot(vTime,convang(delta_a(vTime),'rad','deg'),'b-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-25 5]) 
ylabel('$\delta_{a} (deg)$','interpreter','latex','fontsize',11);
subplot 412
plot(vTime,convang(delta_e(vTime),'rad','deg'),'b-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-10 10])
ylabel('$\delta_{e} (deg)$','interpreter','latex','fontsize',11);
subplot 413
plot(vTime,convang(delta_r(vTime),'rad','deg'),'b-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-3 12])
ylabel('$\delta_{r} (deg)$','interpreter','latex','fontsize',11);
subplot 414
plot(vTime,convang(delta_f(vTime),'rad','deg'),'b-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-10 10]) 
ylabel('$\delta_{f} (deg)$','interpreter','latex','fontsize',11);
figure(6)
plot(vTime,delta_T(vTime),'b-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([0 1])
xlabel('$t (s)$','interpreter','latex','fontsize',11);
ylabel('$\delta_{T}$','interpreter','latex','fontsize',11);


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
ylim([80 200])
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
ylim([-2000 8000])
ylabel('$x_{EG} (m)$','interpreter','latex','fontsize',11)
subplot 612
plot(vTime,vYe,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-8000 1000])
ylabel('$y_{EG} (m)$','interpreter','latex','fontsize',11)
subplot 613
plot(vTime,-(vZe - zEG_0),'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-500 500])
ylabel('$\Delta h (m)$','interpreter','latex','fontsize',11)
subplot 614
plot(vTime,vPhi,'-.','LineWidth',1.5);
grid on
xlim([0 t_fin])
ylim([-50 50])
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
lenghtVectors = length(vXe);
% Setup the figure/scene
h_fig1 = figure(7);
light('Position',[300 300 -3000],'Style','local');
% Trick to have Ze pointing downward and correct visualization
set(gca,'XDir','reverse'); set(gca,'ZDir','reverse');
grid on; hold on;
% Load aircraft shape
shapeScaleFactor = 400.0;
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


