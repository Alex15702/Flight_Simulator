clear all; close all; clc
%% Setup the figure/scene
h_fig1 = figure(1);
light('Position',[3 3 -3],'Style','local');
% Trick to have Ze pointing downward and correct visualization
set(gca,'XDir','reverse'); set(gca,'ZDir','reverse');
grid on; hold on;
%% Load aircraft shape
shapeScaleFactor = 1.0;
shape = loadAircraftMAT('aircraft_pa24-250.mat', shapeScaleFactor); 
%% Set the aircraft in place
% Posision in Earth axes

vXYZe = [2,2,-2;10,2,-2;20,2,-2]; 
% psi, theta, phi -> 'ZYX'
eulerAngles=[20,10,0;20,0,0;0,0,-40]
vEulerAngles = convang(eulerAngles,'deg','rad'); 
% Observer point-of-view
theView = [1.8e+02,1];
% body axes settings
bodyAxesOptions.show = true; 
bodyAxesOptions.magX = 2.0*shapeScaleFactor;
bodyAxesOptions.magY = 2.0*shapeScaleFactor;
bodyAxesOptions.magZ = 1.5*shapeScaleFactor;
bodyAxesOptions.lineWidth = 2.5;

% plotBodyE(h_fig1, shape, ... 
% vXYZe, vEulerAngles, ... 
% bodyAxesOptions, theView);

%% Plot Earth axes
hold on;
xMax = 1;
yMax = 1;
zMax = 1; % max([abs(max(vXYZe(1))),0.18*xMax]);
vXYZ0 = [0,0,0];
vExtent = [xMax,yMax,zMax];
plotEarthAxes(h_fig1, vXYZ0, vExtent);
options.samples=[1,2,3];
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





