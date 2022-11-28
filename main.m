%% Notations Used Below.
% 0- Origin(Earth Center)
% 1- Earth
% 2- Target Satellite
% 3- Chaser Satellite
% o- Intial
% C- 3D transformation Matrix from ECI frame to Hills frame at each time
% instant.
% State Vector [6 x].
%Time Vector [1 x]

clc;
clear;
warning off;
addpath(genpath(pwd));
% close all;

global mu m1 m2 m3 G Re ;
%% Earth:
m1 = 5.974e24;
Re = 6378;
G = 6.6742e-20;
xo10=0;
yo10=0;
zo10=0;
Vxo10=0;
Vyo10=0;
Vzo10=0;
%% First Satellite(Target Satellite )(Hill's Frame is Associated)
m2 = 100;
%Intial Position and Velocity 
%Data Taken of Geo Synchronous Satellite 
%r=42164 km from center
%speed=3.07km/hr
xo20=29814.45;%8000
yo20=0;
zo20=29814.45;%6000
Vxo20=0;
Vyo20=3.07;%7
Vzo20=0;

attitude_svo=[-0.4 0.2 0.8 0 0 0 ];
J=[14 0 0;0 17 0;0 0 20];

Ro20=[xo20 yo20 zo20]';
Vo20=[Vxo20 Vyo20 Vzo20]';
%% Second Satellite(Chaser Satellite )
m3= 200 ;
%Intial Position and Velocity wrt Inertial Frame(o) .
xo30=28000;
yo30=0;
zo30=28000;
Vxo30=0;
Vyo30=3.25;
Vzo30=0;

Ro30=[xo30 yo30 zo30]';
%% Intial Difference Between Satellites
Ro32=[xo30-xo20 yo30-yo20 zo30-zo20]';
Vo32=[Vxo30-Vxo20 Vyo30-Vyo20 Vzo30-Vzo20]';

initial32=[Ro32'  Vo32']';% Intial State Vector of Chaser Sat wrt Target Sat

ref=[0,-100,0,0,0,0]';   % Desired Value of State Vectors or Output till C =eye(6);
%% Time and Other Constants 
hours =3600;
t0 = 0;
tf =24*hours;
step_time =5;
t= t0:step_time:tf;
mu = G*(m1 + m2);

n=2*pi/(24*3600);%Data is of Geosynchronous Satellite (Omega of Target satellite)

r_tol=1e-7;%Tolerence value for Minimum distance between T and C.
fig_no=1;% To keep count of Figures
%% Calculating Orbital State Vector wrt time using Runge Kutta Method of Order-4.

%Considering Two Body System making each satellite Independent of Each
%Other.
p20=[xo20; yo20; zo20; Vxo20; Vyo20; Vzo20];
[T2,y2] = rkf4(@twobody,[t0,tf], p20 ,step_time);
p30=[xo30; yo30; zo30; Vxo30; Vyo30; Vzo30];
[T3,y3] = rkf4(@twobody,[t0,tf], p30,step_time);

%% Simulating Sat Motion around Earth.
% Draw the planet

figure(fig_no)%1
fig_no=fig_no+1;
sgtitle('Satellites motion under Earth Gravitation');
Earthplot([y2(1:3,:)],[y3(1:3,:)]);
%% Getting the relative value of Deputy(chaser) Sat-2 wrt to Leader(Target) Sat-1 in Hills frame
%  and Plotting it 

[r32h, v32h,C,Omega20] = getHills(y2, y3);

figure(fig_no)%2
fig_no=fig_no+1; 
sgtitle("Chaser State Vector in Hills Frame");
subplot2([r32h;v32h],T2)
%% Applying Lqr Control Technique to LHCW equations with constant Omega of Target Sattelite 
[lqr_LHCW_n,t_lqr_nLHCW]=lqr_lhcw_const_N(initial32,t,n);

figure(fig_no)%5
fig_no=fig_no+1; 
subplot2(lqr_LHCW_n',t_lqr_nLHCW');

figure(fig_no)%6
fig_no=fig_no+1; 
Earthplot([y2(1,:); y2(2,:) ;y2(3,:)],[y2(1,:)+lqr_LHCW_n(:,1)'; y2(2,:)+ lqr_LHCW_n(:,2)'; y2(3,:)+ lqr_LHCW_n(:,3)']);

%% Attitude Control

[lqr_Attitude,t_Attitude]=lqr_attitude(t,J,attitude_svo);

figure(fig_no)
fig_no=fig_no+1;
subplot2(lqr_Attitude',t_lqr_nLHCW');