clear all
close all
global w   % turning rate
global u   % velocity
u = 0; 
w = 0;

% Initial pose (x_0,y_0,φ_0)
x_Init=0;y_Init=1;phi_Init=90*pi/180;

% Current pose (initialized as initial pose)
x_D=x_Init;y_D=y_Init;phi_D=phi_Init;

% Parking point pose (x,y,φ)
%x_Pose=0;y_Pose=-1;phi_Pose=0*pi/180;
x_Pose=3;y_Pose=1;phi_Pose=90*pi/180;

% Empty arrays for storing the trajecory
xDRec=[];yDRec=[];phiDRec=[];

% Initial distance 
e = sqrt((x_Pose-x_D)^2+(y_Pose-y_D)^2);

while (e > 0.001)
    
    % relative orientation of the car with respect to the target pose: 
    phi=phi_D-phi_Pose;

    % the angle to the target pose from the car's current position:
    theta=atan2(y_Pose-y_D,x_Pose-x_D)-phi_Pose;

    % normalize θ to be within [-π,π]:
    theta=atan2(sin(theta),cos(theta));

    % the angle between the car's current orientation and the direction towards the target pose:
    alpha=theta-phi;
    
    % normalize α to be within [-π,π]:
    alpha=atan2(sin(alpha),cos(alpha));

    % distance 
    e = sqrt((x_Pose-x_D)^2+(y_Pose-y_D)^2);

    k=6.0;      %control parameter
    gamma=3.0;  %control parameter
    h=1.0;      %control parameter
    % h=3.0;

    % turning rate control
    w=k*alpha+gamma*cos(alpha)*sin(alpha)/alpha*(alpha+h*theta); 
                            % ω = kα+γ[cos(α)sin(α)/α](α+hθ); k > 0 (9)

    % velocity control
    u=gamma*cos(alpha)*e;   % u = (γcos(α)e; γ > 0 (6)

    % saturation of the velocity reference
    if (u > 1)
        u=1;
    elseif (u<-1)
        u=-1;
    end

    % saturation of the turning rate reference
    if (w > 2) 
        w = 2;
    elseif (w < -2)
        w=-2;
    end

    % compute dynamics for 50ms using (xD, yD, thD) as the initial state
    [t, sol]= ode45(@car,[0 0.05],[x_D;y_D;phi_D]);
    % [0 0.05] specifies the time span for the ODE solver (50ms duration)
    % [x_D; y_D; phi_D] = current state vector (ẋ,̇y,̇φ)

    % Store the solution trajectory
    xDRec=[xDRec sol(end,1)];
    yDRec=[yDRec sol(end,2)];
    phiDRec=[phiDRec sol(end,3)];

    % Initial pose for the next pass
    x_D=sol(end,1);y_D=sol(end,2);phi_D=sol(end,3);  

    % Plots the car and the trajectory
    figure(1)
    plotcar(xDRec(1),yDRec(1),phiDRec(1),0.2)
    axis([-2 2 -2 2]);
    axis equal     
    plot(xDRec,yDRec,'k','LineWidth',2), hold on;
    plotcar(x_D,y_D,phi_D,0.2)
    figure(1),hold off
end    
   