%Script that is used to plot the Parallel robot’s workspace for a given
%orientation of alpha. In this case, alpha is just being set to zero to
%give the basic workspace.

%clearing up the workspace and clearing the command window.
clear;
close all;
clc;

% set input values

rBase=290;
rPlat=130;
rA=170;
L=130;
Alpha=0;

%Calculating the values of the vectors for the corners of the base to the
%centre of the base. Basically just pulled straight from the ParaRobotMain
%script.

PBiB=zeros(2,3);

for i=1:3
    PBiB(1,i)=-rBase*cos((210+120*(i-1))*(pi()/180));
    PBiB(2,i)=-rBase*sin((210+120*(i-1))*(pi()/180));
end

% This is the big chunk of code. This section of code is calculating 
%the workspace for the "needle" of the manipulator. The resolution is
%initially set, this detrmines the fidelity of the workspace that is
%produced.

resolution=5;%Change this value if you have a powerful computer!
                %Must be >0, closer to 0 the more accurate 
limit=162;

%Just filling up our equations that we used in the previous script with
%zeros so that they can then be filled in with the correct information
Cx=-limit:resolution:limit;
Cy=-limit:resolution:limit;
CPPi=zeros(2,3);
e1=zeros(1,3);
e2=zeros(1,3);
e3=zeros(1,3);
t=zeros(1,3);
theta=zeros(1,3);
PBiPPi=zeros(2,3);
n=1;
points=zeros(2,2*limit);

%This for loop calculates the angles for each possible centre position of
%the manipulator. Its important that all values of Theta are real otherwise
%the lot wont be accurate to what can actually be acheieved. 
for j=1:length(Cx)
    for k=1:length(Cy)
        for i=1:3 
            CPPi(1,i)=-rPlat*cos((30+Alpha+120*(i-1))*(pi()/180))+Cx(j);
            CPPi(2,i)=-rPlat*sin((30+Alpha+120*(i-1))*(pi()/180))+Cy(k);
            PBiPPi(1,i)=PBiB(1,i)+CPPi(1,i);
            PBiPPi(2,i)=PBiB(2,i)+CPPi(2,i);
            e1(i)=-2*PBiPPi(2,i)*rA;
            e2(i)=-2*PBiPPi(1,i)*rA;
            e3(i)=(PBiPPi(1,i))^2+(PBiPPi(2,i))^2+rA^2-L^2;
            t(i)=(-e1(i)-sqrt((e1(i))^2+(e2(i))^2-(e3(i))^2))/(e3(i)-e2(i));
            theta(i)=2*atan(t(i));
        end
        if isreal(theta)==1 
            points(1,n)=Cx(j);
            points(2,n)=Cy(k);
            n=n+1;
        end
    end
end

PhiY=zeros(3,3);
DetY=zeros(1,3);
PhiZ=zeros(3,3);
DetZ=zeros(1,3);

PhiY=[cos

% plot the workspace

base=[-PBiB(1,:) -PBiB(1,1);-PBiB(2,:) -PBiB(2,1)];

if points==0
else
    scatter(points(1,:),points(2,:),'r.');
    hold on;
end

line(base(1,:),base(2,:), 'Color', 'blue');
plot(0,0,'blue*');
axis equal;
