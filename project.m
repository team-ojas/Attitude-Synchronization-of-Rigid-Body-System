tspan = [0 30];  % start and end times
v0 = 0; w0 = 0.4; y0=1.2;
v1=-1.8;w1 = 4; y1 = -2.3;
v2=-1.2; w2=1.8; y2 = -0.9;
v3 = 1; w3=-2.2; y3=0.8;
v4 = 3; w4 = .01; y4=2; 
% initial values
IC = [v0 w0 y0 v1 w1 y1 v2 w2 y2 v3 w3 y3 v4 w4 y4];

[t, D] = ode45(@LF_rossler, tspan, IC);

% Extract individual solution values
x0=D(:,1);y0=D(:,2);z0=D(:,3); % leader
x1=D(:,4);y1=D(:,5);z1=D(:,6); %follower 1
x2=D(:,7);y2=D(:,8);z2=D(:,9); %follower 2
x3=D(:,10);y3=D(:,11);z3=D(:,12); %follower 3
x4=D(:,13);y4=D(:,14);z4=D(:,15); %follower 4

%error Calculation
e11=x0-x1;e12=x0-x2;e13=x0-x3;e14=x0-x4; % x axis errors of all the systems
e21=y0-y1;e22=y0-y2;e23=y0-y3;e24=y0-y4; % y axis errors of all the systems
e31=z0-z1;e32=z0-z2;e33=z0-z3;e34=z0-z4; % z axis errors of all the systems

% sum of errors
E1=e11+e12+e13+e14;
E2=e21+e22+e23+e24;
E3=e31+e32+e33+e34;

% Plot results


figure(1);     %Trajectory of followers on x, y and z axis

subplot(3,1,1)  %trajectory of follower on x-axis
plot(t,x0,'r-',t,x1,'k-',t,x2,'b-',t,x3,'g-',t,x4,'linewidth',3),grid
xlabel('t'),ylabel('attitude')


subplot(3,1,2)  %trajectory of follower on y-axis
plot(t,y0,'r-',t,y1,'k-',t,y2,'b-',t,y3,'g-',t,y4,'linewidth',3),grid
xlabel('t'),ylabel('attitude')


subplot(3,1,3)   %trajectory of follower on z-axis
plot(t,z0,'r-',t,z1,'k-',t,z2,'b-',t,z3,'g-',t,z4,'linewidth',3),grid
xlabel('t'),ylabel('attitude')


figure(2);     %Error summation of followers

subplot(3,1,1)     %error summation of all followers on x-axis
plot ( t,E1,'r:','linewidth',3);
xlabel('t'),ylabel('error');


subplot(3,1,2)   %error summation of all followers on y-axis
plot (t,E2,'b:','linewidth',3);
xlabel('t'),ylabel('error');


subplot(3,1,3) %error summation of all followers on z-axis
plot (t,E3,'g','linewidth',3);
xlabel('t'),ylabel('error');
%legend('stage 3')

% 4 random coupled lorenz system configuration 
function  [xdot] = LF_rossler(t,D)
x01=D(1);x02=D(2);x03=D(3);
x11=D(4);x12=D(5);x13=D(6);
x21=D(7);x22=D(8);x23=D(9);
x31=D(10);x32=D(11);x33=D(12);
x41=D(13);x42=D(14);x43=D(15);

% constant derived from contraction therory
al=0.2;     %Rossler system parameter
bl=0.2;     %Rossler system parameter
cl=5.7;     %Rossler system parameter
a=10;       %Lorenz system parameter
b=8/3;      %Lorenz system parameter
c=28;       %Lorenz system parameter
k1=20;k2=500; 

% controller before 5 sec
if t<5
    u1=0;u2=0;u3=0;
    u4=0;u5=0;u6=0;
    u7=0;u8=0;u9=0;
    u10=0;u11=0;u12=0;
else
    % controller after 5 sec
    u1=(k1*(x21-x11))+k2*(x01-x11);
    u2=(k1*(x22-x12))+k2*(x02-x12);
    u3=(k1*(x23-x13))+k2*(x03-x13);
    u4=(k1*(x31-x21))+k2*(x01-x21);
    u5=(k1*(x32-x22))+k2*(x02-x22);
    u6=(k1*(x33-x23))+k2*(x03-x23);
    u7=(k1*(x41-x31))+k2*(x01-x31);
    u8=(k1*(x42-x32))+k2*(x02-x32);
    u9=(k1*(x43-x33))+k2*(x03-x33);
    u10=(k1*(x11-x41))+k2*(x01-x41);
    u11=(k1*(x12-x42))+k2*(x02-x42);
    u12=(k1*(x13-x43))+k2*(x03-x43);
end

% system dynamics
xdot=[-x02-x03; % leader dynamics
    x01+al*x02;
    bl+x03*(x01-cl);
    -a*x11+a*x12+u1;
    c*x11-x12-x11*x13+u2;
    -b*x13+x11*x12+u3;
    -a*x21+a*x22+u4;
    c*x21-x22-x21*x23+u5;
    -b*x23+x21*x22+u6;
    -a*x31+a*x32+u7;
    c*x31-x32-x31*x33+u8;
    -b*x33+x31*x32+u9;
    -a*x41+a*x42+u10;
    c*x41-x42-x41*x43+u11;
    -b*x43+x41*x42+u12];
end