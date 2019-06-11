% Returns the outer controller as an optimizer instance
function outerController =  getOuterController(Ac, solver)

%state: x2 = [x dx y dy z]
%input: ref2 = [x,y,z]
%output  = [dz roll pitch]

%u2 = [pitch roll dz];
%x2 = [x dx  y dy  z]

%load('quadData.mat')

if(~exist('solver','var') || isempty(solver))
    solver = 'qpip';
end

n2 = 5; m2 = 3;
N2 = 30;

Ac2 = zeros(5,5);
Ac2(1,2) = 1; Ac2(3,4)=1;
Bc2 = zeros(5,3);
Bc2(2,1) = Ac(4,8); Bc2(4,2) = Ac(5,7); Bc2(5,3) = 1;
quadSys2 = ss(Ac2,Bc2,eye(n2),zeros(n2,m2));

% Discretize the system
Ts2 = 0.3;
quadSysD2 = c2d(quadSys2, Ts2); % outer subsystem
[A2,B2,~,~] = ssdata(quadSysD2);


% Constraints
sys.xMin = [-5;-5;-1];
sys.xMax = [5;5;4];
sys.angleMin = -[1;1]*(10/180)*pi;
sys.angleMax =  [1;1]*(10/180)*pi;
sys.velMin   = -1*[2;2;2];
sys.velMax   =  1*[2;2;2];

% Stage cost
C2 = [1 0 0 0 0;0 0 1 0 0;0 0 0 0 1]; % penalization of x,y,z
Q2 = diag([1,1,50]);
R2 = 100*eye(m2);

% Terminal controller
[K2, P2] = dlqr(A2,B2,C2'*Q2*C2,R2); K2 = -K2;
M = [1 0 0; zeros(1,3);0 1 0;zeros(1,3);0 0 1];

x2 = sdpvar(n2, N2);
u2 = sdpvar(m2,  N2-1);
ref2 = sdpvar(3, 1);
d2 = sdpvar(n2,1);
obj = 0;
con = [];
for i = 1:N2-1
    % Dynamics
    con = [con, x2(:,i+1) == A2*x2(:,i) + B2*u2(:,i)];
    % Input constraints
    con = [con, sys.angleMin(end:-1:1) <= u2(1:2,i) <= sys.angleMax(end:-1:1)];
    % Cost
    obj = obj + u2(:,i)'*R2*u2(:,i);
end
for i = 2:N2
    % State constraints
    con = [con, sys.xMin <= x2([1,3,5],i) <= sys.xMax ]; % x,y,z constraints
    con = [con, sys.velMin(1:2) <= x2([2,4],i) <= sys.velMax(1:2) ]; % x,y velocity constraints
    % Cost
    obj = obj + (C2*x2(:,i) - ref2)'*Q2*(C2*x2(:,i) - ref2);
end
% terminal weight
obj = obj + (x2(:,N2)-M*ref2)'*P2*(x2(:,N2)-M*ref2);


opt = sdpsettings('solver',solver);
outerController = optimizer( con, obj, opt, [x2(:,1);ref2], u2(end:-1:1,1) );

end