clear all
close all
global u   % turning rate
global v   % velocity
v = 0; 
u = 0;
% Initial pose
xInit=-2;yInit=1;thInit=135*pi/180;
xD=xInit;yD=yInit;thD=thInit;
% Parking point pose (headin of the pose is 0)
xPose=0;yPose=0;thPose=0;
% Emty arrays for storing the trajecory
xDRec=[];yDRec=[];thDRec=[];
% Initial distance 
r = sqrt((xPose-xD)^2+(yPose-yD)^2);
while (r > 0.001),
    phi=thD-thPose;
    theta=atan2(yPose-yD,xPose-xD)-thPose;
    theta=atan2(sin(theta),cos(theta));
    alpha=theta-phi;
    alpha=atan2(sin(alpha),cos(alpha));
    % distance 
    r = sqrt((xPose-xD)^2+(yPose-yD)^2);
    k=6.0;      %control parameter
    gamma=3.0;  %control parameter
    h=3.0;      %control parameter
    % turning rate control
    u=k*alpha+gamma*cos(alpha)*sin(alpha)/alpha*(alpha+h*theta);
    % velocity control
    v=gamma*cos(alpha)*r;
    % saturation of the velocity reference
    if (v > 0.5),
        v=0.5;
    elseif (v<-0.5),
        v=-0.5;
    end
    % saturation of the turning rate reference
    if (u > 2), 
        u = 2;
    elseif (u < -2),
        u=-2;
    end
    % compute dynamisc for 50ms using (xD, yD, thD) as the initial state
    [t sol]= ode45(@car,[0 0.05],[xD;yD;thD]);
    % Store the solution
    xDRec=[xDRec sol(end,1)];
    yDRec=[yDRec sol(end,2)];
    thDRec=[thDRec sol(end,3)];
    % Initial pose for the next pass
    xD=sol(end,1);yD=sol(end,2);thD=sol(end,3);    
    % Plots the car and the trajectory
    figure(1)
    plotcar(xDRec(1),yDRec(1),thDRec(1),0.2)
    axis([-2 2 -2 2]);
    axis equal     
    plot(xDRec,yDRec,'k','LineWidth',2), hold on;
    plotcar(xD,yD,thD,0.2)
    figure(1),hold off
end    
   