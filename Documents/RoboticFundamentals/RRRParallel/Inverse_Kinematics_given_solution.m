clear;
close all;
clc;

%% set constants

ra=170;
L=130;
Rb=290;
Rp=130;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% set input parameters

C=[-60;-45];
alpha=0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% calculate vectors of the COM of the platform to the platform corners

CPPi=zeros(2,3);

for i=1:3
    CPPi(1,i)=-Rp*cos((30+alpha+120*(i-1))*(pi()/180))+C(1);
    CPPi(2,i)=-Rp*sin((30+alpha+120*(i-1))*(pi()/180))+C(2);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% calculate vectors of base corners to the COM of the base

PBiB=zeros(2,3);

for i=1:3
    PBiB(1,i)=-Rb*cos((210+120*(i-1))*(pi()/180));
    PBiB(2,i)=-Rb*sin((210+120*(i-1))*(pi()/180));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% calculate vectors from base corners to respective platform corners

PBiPPi=zeros(2,3);

for i=1:3
    PBiPPi(1,i)=PBiB(1,i)+CPPi(1,i);
    PBiPPi(2,i)=PBiB(2,i)+CPPi(2,i);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% calculate theta and psi values

e1=zeros(1,3);
e2=zeros(1,3);
e3=zeros(1,3);
t=zeros(1,3);
theta=zeros(1,3);

for i=1:3
    e1(i)=-2*PBiPPi(2,i)*ra;
    e2(i)=-2*PBiPPi(1,i)*ra;
    e3(i)=(PBiPPi(1,i))^2+(PBiPPi(2,i))^2+ra^2-L^2;
    t(i)=(-e1(i)-sqrt((e1(i))^2+(e2(i))^2-(e3(i))^2))/(e3(i)-e2(i));
    theta(i)=2*atan(t(i));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% find the joint positions for the plot

Ji=zeros(2,3);

for i=1:3
    Ji(1,i)=-PBiB(1,i)+ra*cos(theta(i));
    Ji(2,i)=-PBiB(2,i)+ra*sin(theta(i));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% plot the robot's configuration

plat=[CPPi(1,:) CPPi(1,1);CPPi(2,:) CPPi(2,1)];
base=[-PBiB(1,:) -PBiB(1,1);-PBiB(2,:) -PBiB(2,1)];
links1=[-PBiB(1,1) Ji(1,1) CPPi(1,1);-PBiB(2,1) Ji(2,1) CPPi(2,1)];
links2=[-PBiB(1,2) Ji(1,2) CPPi(1,2);-PBiB(2,2) Ji(2,2) CPPi(2,2)];
links3=[-PBiB(1,3) Ji(1,3) CPPi(1,3);-PBiB(2,3) Ji(2,3) CPPi(2,3)];

plot(C(1),C(2),'black*');
hold on;
plot(0,0,'r*');
line(plat(1,:),plat(2,:), 'Color', 'black');
line(base(1,:),base(2,:), 'Color', 'r');
line(links1(1,:),links1(2,:), 'Color', 'b');
line(links2(1,:),links2(2,:), 'Color', 'b');
line(links3(1,:),links3(2,:), 'Color', 'b');

axis equal
grid on