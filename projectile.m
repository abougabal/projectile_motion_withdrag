clc
clear
close all

global R
global xbarriers
global zbarriers
global zFinal
global mass
global dragcofficent
global gravity

solx=0; %x initial position 
gravity=9.81;

prompt=("enter the C ,drag cofficent, of the object : \n");
dragcofficent=input(prompt);% C

prompt=("enter the mass of the object : \n");
mass=input(prompt);% mass of our projectile

prompt=("enter the z initial location: \n");
solz=input(prompt);%y initial position

prompt=("enter the x location of the barrier or 0 if their is no barriers : \n");
xbarriers=input(prompt); %x loctation of the barrier

prompt=("enter the hight of the barrier or 0 if their is no barriers : \n");
zbarriers=input(prompt); % y location of the barrier 

prompt=("enter X final position : \n");
R=input(prompt); % projectile final X position

prompt=("enter Y final position : \n");
zFinal=input(prompt); % projectile final Y position

toggle=true;
vs=0; %start value for the initial velocity magnitude
ve=3*1000000000; %end value for the initial velocity magnitude
thetas=0; %start value for the initial angle
thetae=90; %end value for the initial angle

while toggle
    if (abs(ve-vs)<0.001 || abs(thetae-thetas)<0.001) %to exit when the start and end value are almost the same
        toggle=false;
        disp("no possible solution");
    else
        
        v=(ve+vs)/2.0; %initial velocity equal to the average
        theta=(thetae+thetas)/2.0; %initial angle equal to the average
        solvx=v*cosd(theta); % initial velocity in x
        solvz=v*sind(theta);% initial velocity in z
        
        y=[solx;solvx;solz;solvz];
        Opt = odeset('Events', @Event); 
        [t,sol]=ode45(@projectilefun,(0:0.01:100),y,Opt);
        
        difference = sol(end,1)-R; %to calculate the difference between the final x position and the desired range
        
        %to calculate the closest point to the barrier
        check=0;
        dist = abs(sol(:,1)-xbarriers);
        mindist = min(dist);
        idx = find(dist == mindist);
        if((sol(idx,3)<=zbarriers) && xbarriers>0 && mindist<0.1)         
            check=1;
        end
        if (check==1) %hits a barrier
            vs=0;
            ve=3*1000000000;
            thetas=theta;
        elseif (abs(difference) < 0.01) %fells at the position
            toggle=false;
        elseif (difference > 0) %fell after the range
            ve=v;
        elseif (difference <0) %fell before the range
            vs=v;
        end
    end
end

hold on;

%barrier drawing
if (xbarriers>0) && (zbarriers > 0) 
    line([xbarriers xbarriers],[0 zbarriers],'LineWidth',4);
end

%projectile drawing
yline(0);
plot(sol(:,1),sol(:,3));
title('projectile');
xlabel('x position');
ylabel('z position');

%display reach time
disp("Reach time is ");
disp(t(end));

%function to solve the 4 ODE of the accelration to get the x and y
%positions
function [xp] = projectilefun(t,sol)
global dragcofficent
global mass
global gravity
    xp=zeros(4,1);
    xp(1)=sol(2);    % dx/dt // velocity in x
    xp(2)=(-dragcofficent/mass)*sqrt( ((sol(2))^2) + ((sol(4))^2) )*sol(2);   %dvx/dt //acceleration in x
    xp(3)=sol(4);     % dz/dt //velocity in y
    xp(4)=-gravity-(dragcofficent/mass)*sqrt( ((sol(2))^2) + ((sol(4))^2) )*sol(4); %dvz/dt // acceleration in y
end


%event function to break ode45 once it reach the final y position
function [value, isterminal, direction] = Event(t, sol)
    global zFinal;
    value = ~(sol(3)<=zFinal);
    isterminal = 1;   % Stop the integration
    direction  = -1; % Negative direction only
end
