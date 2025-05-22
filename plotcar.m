function plotcar(x,y,theta,scale)
  x0(1)= 1*scale;y0(1)=0;
  x0(2)=-0.5*scale;y0(2)=0.5*scale;
  x0(3)=-0.5*scale;y0(3)=-0.5*scale;
  T=[ cos(theta) -sin(theta) x;...
      sin(theta)  cos(theta) y;
      0 0 1
      ];
  for i=1:3,
    p(:,i)=T*[x0(i);y0(i);1];
  end
  figure(1)
  plot([p(1,:) p(1,1)],[p(2,:) p(2,1)],'b-');
  hold on
  plot(p(1,2),p(2,2),'r.')
  plot(p(1,3),p(2,3),'g.')
  p(:,4)=T*[0;0;1];
  plot(p(1,4),p(2,4),'b.')
  