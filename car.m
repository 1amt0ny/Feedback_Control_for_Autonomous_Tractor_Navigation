% The matlab script defines the ODE  
% \dot{x} = v cos \theta
% \dot{y} = v cos \theta
% \dot{\theta} = u  
%
% The time varying u is passed to the function as a global variable
%
% Kinematic Equations: including vehicle's Cartesian position x, y, and
% orientation angle theta

function dx=car(t,x)      % t: time; x: state vector
  global w;               % turn rate
  global u;               % velocity

  phi=x(3,1);             % state vector (ẋ,̇y,̇φ)
  
  dx(1,1)=u*cos(phi);     % ẋ = ucos(φ)
  dx(2,1)=u*sin(phi);     % ̇y = usin(φ)   (1)
  dx(3,1)=w;              % ̇φ = ω
end