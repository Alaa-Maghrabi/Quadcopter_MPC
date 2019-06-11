function h = plotQuad(quad, state, thrust)
% Plot the quad

% Rotation from body to interial frame 
roll = state.theta(1); pitch = state.theta(2); yaw = state.theta(3);
R = [1 0 0;0 cos(roll) -sin(roll);0 sin(roll) cos(roll)];
R = R*[cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
R = R*[cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];

% Position
x = state.x;

[X,Y,Z] = sphere;
X = quad.rad * X + x(1);
Y = quad.rad * Y + x(2);
Z = quad.rad * Z + x(3);
h=surf(X,Y,Z);
shading interp
set(h,'facecolor','b','linestyle','none');
lighting gouraud

% Draw the blades
L = R*quad.L;
plot3(L(1,:)+x(1),L(2,:)+x(2),L(3,:)+x(3),'.','markersize',30);
for i = 1:4
  plot3([0;L(1,i)]+x(1),[0;L(2,i)]+x(2),[0;L(3,i)]+x(3),'k-','linewidth',3)
  
  th = linspace(-pi,pi,20);
  t = R*(quad.bladeRad*[sin(th);cos(th);0*th] + quad.L(:,i)*ones(1,20));
  t = t + x*ones(1,20);
  plot3(t(1,:),t(2,:),t(3,:),'k');
end


% Plot the forces
for i = 1:4
  thrustDir = quad.thrustDir(:,i);
  t = thrustDir / norm(thrustDir) * thrust(i) / quad.thrustLimits(2,i) * norm(quad.L(:,1));
  t = R*t;
%  plot3([0;t(1)]+L(1,i)+x(1),[0;t(2)]+L(2,i)+x(2),[0;t(3)]+L(3,i)+x(3),'r-','linewidth',3);
end
