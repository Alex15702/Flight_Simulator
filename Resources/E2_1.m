clear all; close all; clc
%% Load aircraft 3D model
% see http://wpage.unina.it/agodemar/DSV-DQV/aircraft_models.zip
shapeScaleFactor = 1.0;
[V, F, C] = loadAircraftSTL('aircraft_pa24-250.stl', shapeScaleFactor);
% save shape in .mat format for future use
shape.V = V; shape.F = F; shape.C = C;
%save('aircraft_pa24-250.mat', 'shape');

%% Setup the figure/scene
h_fig1 = figure(1);
grid on
hold on
light('Position',[1 0 -2],'Style','local');
% Trick to have Ze pointing downward and correct visualization
set(gca,'XDir','reverse');
set(gca,'ZDir','reverse'); 
%% Display aircraft shape
p = patch('faces', shape.F, 'vertices', shape.V); 
set(p, 'facec', [1 0 0]);
set(p, 'EdgeColor','none');
theView = [-125 30];
view(theView); 
axis equal;
lighting phong
hold on
