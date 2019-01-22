%Script implementing parallel inverse kinematics for a robot with 6 passive
%revolute joints and 3 active revolute joints.

%clearing up the workspace and clearing the command window.
clear all;
clc;

%Design params for the robot
rA=170;
L=130;
rPlat=130;
rBase=290;

%Input params for the robot. Change these to  move the platform around.
alpha=-60;
C=[0; 20];

%Angular joint positions for the robot
PB1=pi/2;
PB2=pi+pi/6;
PB3=2*pi-pi/6;
PP1=pi/2;
PP2=pi+pi/6;
PP3=2*pi-pi/6;

%############CALCULTAING & IMPLEMENTING THE INVERSE KINEMATICS############
%Calculating the values of the vectors for the centre of the platform to
%the corners of the platform

CPPi=zeros(2,3);

for i=1:3
    CPPi(1,i)=-rPlat*cos((30+alpha+120*(i-1))*(pi()/180))+C(1);
    CPPi(2,i)=-rPlat*sin((30+alpha+120*(i-1))*(pi()/180))+C(2);
end

%Calculating the values of the vectors for the corners of the base to the
%centre of the base.

PBiB=zeros(2,3);

for i=1:3
    PBiB(1,i)=-rBase*cos((210+120*(i-1))*(pi()/180));
    PBiB(2,i)=-rBase*sin((210+120*(i-1))*(pi()/180));
end

%Calculating the values of the vectors for the corners of the base to the
%corners of the platform

PBiPPi=zeros(2,3);

for i=1:3
    PBiPPi(1,i)=PBiB(1,i)+CPPi(1,i);
    PBiPPi(2,i)=PBiB(2,i)+CPPi(2,i);
end

%Calculating the values of the Theta angles. The angles & simplified
%equations are originally set to an array of zeroes and then "filled in"

e1=zeros(1,3);
e2=zeros(1,3);
e3=zeros(1,3);
t=zeros(1,3);
Theta=zeros(1,3);

%The use of e1, e2 and e3 is purely to make the equations more managable
%because the inverse kineamtics produced some very lengthy equations.
for i=1:3
    e1(i)=-2*PBiPPi(2,i)*rA;
    e2(i)=-2*PBiPPi(1,i)*rA;
    e3(i)=(PBiPPi(1,i))^2+(PBiPPi(2,i))^2+rA^2-L^2;
    t(i)=(-e1(i)-sqrt((e1(i))^2+(e2(i))^2-(e3(i))^2))/(e3(i)-e2(i));
    Theta(i)=2*atan(t(i));
end

%Finding all of the joint positions so that it can be plotted correctly.

Ji=zeros(2,3);

for i=1:3
    Ji(1,i)=-PBiB(1,i)+rA*cos(Theta(i));
    Ji(2,i)=-PBiB(2,i)+rA*sin(Theta(i));
end
%############PLOTTING THE WORKSPACE FOR A GIVEN ORIENTATION############
%Assigning the platform, base and "links". Might be a good idea to sepreate
%the links into their 2 link forms, instead of making them one whole one...
plat=[CPPi(1,:) CPPi(1,1);CPPi(2,:) CPPi(2,1)];
base=[-PBiB(1,:) -PBiB(1,1);-PBiB(2,:) -PBiB(2,1)];
links1=[-PBiB(1,1) Ji(1,1) CPPi(1,1);-PBiB(2,1) Ji(2,1) CPPi(2,1)];
links2=[-PBiB(1,2) Ji(1,2) CPPi(1,2);-PBiB(2,2) Ji(2,2) CPPi(2,2)];
links3=[-PBiB(1,3) Ji(1,3) CPPi(1,3);-PBiB(2,3) Ji(2,3) CPPi(2,3)];

% Red=Platform
% Blue=Base
% Black=Links
plot(C(1),C(2),'red*');
hold on;
plot(0,0,'blue*');
line(plat(1,:),plat(2,:), 'Color', 'red');
line(base(1,:),base(2,:), 'Color', 'blue');
line(links1(1,:),links1(2,:), 'Color', 'black');
line(links2(1,:),links2(2,:), 'Color', 'black');
line(links3(1,:),links3(2,:), 'Color', 'black');

axis equal
grid on

