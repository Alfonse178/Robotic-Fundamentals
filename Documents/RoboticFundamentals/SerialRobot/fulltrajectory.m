%Inverse and FK animation
%To do the drawing we need to say our start point and end point then
%calculate spots in between
clear all;
clc;
%point 1 to 2
[tX,tY,tZ] = trajectory(20.5,0,-6,10.25,0,11.7535,0);

%2 to 3
[eeX,eeY,eeZ] = trajectory(10.25,0,11.7535,7.2478,7.2478,11.7535,0);

%store all the ee positions
tX = [tX eeX];
tY = [tY eeY];
tZ = [tZ eeZ];

%3 to 4
[eeX,eeY,eeZ] = trajectory(7.2478,7.2478,11.7535,0,10.25,11.7535,0);

%store all the ee positions
tX = [tX eeX];
tY = [tY eeY];
tZ = [tZ eeZ];

%4 to 5
[eeX,eeY,eeZ] = trajectory(0,10.25,11.7535,0,20.5,-6,0);

%store all the ee positions
tX = [tX eeX];
tY = [tY eeY];
tZ = [tZ eeZ];

%plot the trajectory
hold on;
plot3(tX,tY,tZ,'r-');
xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');
hold off;






%trajectory between 2 points

function [Xa,Ya,Za] = trajectory(sx,sy,sz,ex,ey,ez,dq5)


%no of mid points
points = 10;



%start joint calc
[sq1,sq2,sq3,sq4,sq5] = ik(sx,sy,sz,dq5);
[Xa(1), Ya(1), Za(1)] = fk(sq1,sq2,sq3,sq4,sq5);
pause(1);


%checks whether the x/y/z are positive or negative to get the right
%interval for each this looks at the total distance travelled
if sx> 0 && ex <0
    xinterval = (ex-sx)/points;
    %disp('stop1');
elseif sx==ex
    xinterval = 0;
    %disp('stop2');
elseif sx<0 && ex>0
    xinterval = (abs(sx)+ex)/points;
    %disp('stop3');
elseif sx<ex
    xinterval = (ex-sx)/points;
    %disp('stop4')
elseif sx>ex
    xinterval = -(sx-ex)/points;
    %disp('stop5');
end

if sy> 0 && ey <0
    yinterval = (ey-sy)/points;
elseif sy==ey
    yinterval = 0;
 
elseif sy<0 && ey>0
    yinterval = (abs(sy)+ey)/points;
 
elseif sy<ey
    yinterval = (ey-sy)/points;
  
elseif sy>ey
    yinterval = -(sy-ey)/points;
  
end

if sz> 0 && ez <0
    zinterval = (ez-sz)/points;
    %disp('stop1');
elseif sz==ez
    zinterval = 0;
   % disp('stop2');
elseif sz<0 && ez>0
    zinterval = (abs(sz)+ez)/points;
    %disp('stop3');
elseif sz<ez
    zinterval = (ez-sz)/points;
    %disp('stop4')
elseif sz>ez
    zinterval = -(sz-ez)/points;
    %disp('stop5');
end




%mid & end point(s)

for i = 1:1:points 
x = sx + xinterval*i;
y = sy + yinterval*i;
z = sz + zinterval*i;
[mq1,mq2,mq3,mq4,mq5] = ik(x,y,z,dq5);

[Xa(i+1),Ya(i+1),Za(i+1)] = fk(mq1,mq2,mq3,mq4,mq5);
pause(0.2);


end
%create array of ee positions

eearray = [Xa,Ya,Za];

%hold on;
%plot3(Xa, Ya, Za, 'r-');
%xlabel('X axis');
%ylabel('Y axis');
%zlabel('Z axis');
%hold off;
%End joint calc
%[eq1, eq2, eq3, eq4, eq5] = ik(ex,ey,ez,0);
%fk(eq1,eq2,eq3,eq4,eq5);

end

%Foward function returns end effector pos
function [XE, YE, ZE] = fk(q1,q2,q3,q4,q5)

%link lengths
L1 = 4.5;
L2 = 9.5;
L3 = 11;
L4 = 6.5;
L5 = 4;
L45 = L4 +L5;
%dh table
alphan1 = -pi/2;
alphan2 = -pi/2;
an = [0 L2 L3 0 0];
alphan = [alphan1 0 0 alphan2 0];
dn = [L1 0 0 0 L45];
thetan = [q1 q2 q3 q4 q5];



%Htms

T1 = [ cos(thetan(1)) -cos(alphan(1))*sin(thetan(1)) sin(alphan(1))*sin(thetan(1)) an(1)*cos(thetan(1));
    sin(thetan(1)) cos(alphan(1))*cos(thetan(1)) -sin(alphan(1))*cos(thetan(1)) an(1)*sin(thetan(1));
0 sin(alphan(1)) cos(alphan(1)) dn(1);
0 0 0 1];

T2 = [ cos(thetan(2)) -cos(alphan(2))*sin(thetan(2)) sin(alphan(2))*sin(thetan(2)) an(2)*cos(thetan(2));
    sin(thetan(2)) cos(alphan(2))*cos(thetan(2)) -sin(alphan(2))*cos(thetan(2)) an(2)*sin(thetan(2));
0 sin(alphan(2)) cos(alphan(2)) dn(2);
0 0 0 1];


T3 = [ cos(thetan(3)) -cos(alphan(3))*sin(thetan(3)) sin(alphan(3))*sin(thetan(3)) an(3)*cos(thetan(3));
    sin(thetan(3)) cos(alphan(3))*cos(thetan(3)) -sin(alphan(3))*cos(thetan(3)) an(3)*sin(thetan(3));
0 sin(alphan(3)) cos(alphan(3)) dn(3);
0 0 0 1];

T4 = [ cos(thetan(4)) -cos(alphan(4))*sin(thetan(4)) sin(alphan(4))*sin(thetan(4)) an(4)*cos(thetan(4));
    sin(thetan(4)) cos(alphan(4))*cos(thetan(4)) -sin(alphan(4))*cos(thetan(4)) an(4)*sin(thetan(4));
0 sin(alphan(4)) cos(alphan(4)) dn(4);
0 0 0 1];

T5 = [ cos(thetan(5)) -cos(alphan(5))*sin(thetan(5)) sin(alphan(5))*sin(thetan(5)) an(5)*cos(thetan(5));
    sin(thetan(5)) cos(alphan(5))*cos(thetan(5)) -sin(alphan(5))*cos(thetan(5)) an(5)*sin(thetan(5));
0 sin(alphan(5)) cos(alphan(5)) dn(5);
0 0 0 1];

T12 = T1*T2;
T13 = T12*T3;
T14 = T13*T4;
T = T14*T5;

%Base
XB =0;
YB =0;
ZB =0;
%1st joint
X1 = T1(1,4);
Y1= T1(2,4);
Z1 = T1(3,4);
%2nd joint
X2 = T12(1,4);
Y2= T12(2,4);
Z2 = T12(3,4);
%3rd joint
X3 = T13(1,4);
Y3= T13(2,4);
Z3 = T13(3,4);
%4th joint
X4 = T14(1,4);
Y4= T14(2,4);
Z4 = T14(3,4);
%End effector
XE = T(1,4);
YE = T(2,4);
ZE = T(3,4);

%Arrays for link drawing remember our Y is actually Z in global frame

X = [XB X1 X2 X3 X4 XE];
Y = [YB Y1 Y2 Y3 Y4 YE];
Z = [ZB Z1 Z2 Z3 Z4 ZE];

boxx = [15 15 15 15 15 4 4 4 4 4 4 15];
boxy = [15 15 4 4 4 4 4 4 15 15 15 15];
boxz = [0 8 8 0 8 8 0 8 8 0 8 8];

plot3(X, Y, Z, '-');
hold on;
%plot3(boxx,boxy,boxz, 'g-'); %object goes here
axis([-25 25 -25 25 -25 25]);
hold off;

end

%Inverse function returns joint variables
function [q1, q2, q3, q4, q5] = ik(gx, gy, gz, dq5)
%links
L1 = 4.5;
L2 = 9.5;
L3 = 11;
L4 = 6.5;
L5 = 4;
L45 = L4+L5; %Treat L4 and L5 as one link

%q1
q1 = atan2(gy,gx);

%q2+q3+q4 or phi
phi = 0; %maintains the orientation

%setup (need phi first)
a = L1-L45*cos(phi)-gz;
b = gx*cos(q1)+gy*sin(q1)+L45*sin(phi);

%q3 needs to be real
q3 = real(acos((a^2+b^2-L2^2-L3^2)/(2*L2*L3)));

%q2 need q3 first
q2 = atan2(a*(L2+L3*cos(q3))-b*L3*sin(q3), a*L3*sin(q3)+b*(L2+L3*cos(q3)));

%q4
q4 = phi - q2 -q3;

%components for q5
X5X = cos(q1)*cos(phi)*cos(dq5) + sin(q1)*sin(dq5);
X5Y = sin(q1)*cos(phi)*cos(dq5) - cos(q1)*sin(dq5);
%q5 
q5 = cos(phi)*q1 -2*atan2(X5Y, X5X);


end
