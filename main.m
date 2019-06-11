% % % loads:
% % %    hovering equilibrium (xs,us)
% % %    continuous time matrices Ac,Bc of the linearization
% % %    matrices sys.A, sys.B of the inner-loop discretized with sampling period sys.Ts
% % %    outer controller optimizer instance
% % clear all
% % close all
% % load('quadData.mat')
% % outerController = getOuterController(Ac, 'quadprog');
% % disp('Data successfully loaded')
% % 
% % 
% % 
% % %% %%%%%%%%%%%%%% First MPC controller %%%%%%%%%%%%%%%%%%%
% % DEG_TO_RAD = pi/180;
% % T=10;
% % x0 = [-1;10*DEG_TO_RAD;-10*DEG_TO_RAD;120*DEG_TO_RAD;0;0;0];
% % 
% % 
% % %MPC data
% % A=sys.A;
% % B=sys.B;
% % H = [1 0 0 0 0 0 0;...
% %     0 1 0 0 0 0 0;...
% %     0 0 1 0 0 0 0;...
% %     0 0 0 0 1 0 0;...
% %     0 0 0 0 0 1 0;...
% %     0 0 0 0 0 0 1;...
% %     -1 0 0 0 0 0 0;...
% %     0 -1 0 0 0 0 0;...
% %     0 0 -1 0 0 0 0;...
% %     0 0 0 0 -1 0 0;...
% %     0 0 0 0 0 -1 0;...
% %     0 0 0 0 0 0 -1];
% % h = [1;10*DEG_TO_RAD;10*DEG_TO_RAD;15*DEG_TO_RAD;15*DEG_TO_RAD;60*DEG_TO_RAD];
% % h=[h;h];
% % Q=diag([1,50,10,1,1,1,1]);
% % R=0.001*eye(4);
% % G = [1,0,0,0;...
% %     0,1,0,0;...
% %     0,0,1,0;...
% %     0,0,0,1;...
% %     -1,0,0,0;...
% %     0,-1,0,0;...
% %     0,0,-1,0;...
% %     0,0,0,-1];
% % g = [1;1;1;1;0;0;0;0];
% % N=10;
% % 
% % %Controller
% % [K,~,~]=dlqr(A,B,Q,R);
% % K=-K;
% % 
% % %Constraints area
% % %Xf=exo3(A,B,H,h,G,g,K);
% % [P,~,~] = dare(A,B,Q,R);
% % 
% % 
% % nx=7;
% % nu=4;
% % x_s=xs(6:end);
% % 
% % % Inputs u(1), ..., u(N)
% % u = sdpvar(repmat(nu,1,N),ones(1,N));
% % % States x(1), ..., x(N)
% % x = sdpvar(repmat(nx,1,N),ones(1,N));
% % 
% % constraints = [];
% % objective = 0;
% % for k = 1:N-1
% %     objective = objective + (x{k}-x_s)'*Q*(x{k}-x_s) + (u{k}-us)'*R*(u{k}-us);
% %     constraints = [constraints, x{k+1} == A*x{k} + B*u{k}];
% %     constraints = [constraints, G*(u{k}+us)<= g , H*x{k}<=h];
% % end
% % objective = objective + (x{N}-x_s)'*P*(x{N}-x_s); % Terminal weight
% % constraints = [constraints, (H*x{N}<=h)];
% % 
% % options = sdpsettings('solver','quadprog');
% % 
% % innerController_s = optimizer(constraints, objective, options, x(:,1), u(:,1));
% % 
% % %simQuad( sys, innerController_s, x0, T);
% % 
% % %pause
% % 
% % %% Reference tracking - no disturbance, no invariant sets
% % fprintf('PART II - reference tracking...\n')
% % 
% % C=[eye(4) zeros(4,3)];
% % 
% % Ns = 1;
% % nr=4;
% % 
% % % Inputs u(1), ..., u(N)
% % u_r = sdpvar(repmat(nu,1,Ns),ones(1,Ns));
% % % States x(1), ..., x(N)
% % x_r = sdpvar(repmat(nx,1,Ns),ones(1,Ns));
% % % r
% % r = sdpvar(repmat(nr,1,Ns),ones(1,Ns));
% % 
% % constraints = [];
% % objective = 0;
% % for k = 1:N-1
% %     objective = objective + (x{k}-x_r)'*Q*(x{k}-x_r) + (u{k}-u_r)'*R*(u{k}-u_r);
% %     constraints = [constraints, x{k+1} == A*x{k} + B*u{k}];
% %     constraints = [constraints, G*(u{k}+us)<= g , H*x{k}<=h];
% % end
% % constraints = [constraints, x_r == A*x_r + B*u_r];
% % constraints = [constraints, H*x_r<=h, r == C*x_r];
% % objective = objective + (x{N}-x_r)'*P*(x{N}-x_r); % Terminal weight
% % 
% % innerController = optimizer(constraints,objective,options,[x{1};r],u(:,1));
% % r_=[0.5;9;9;9];
% % r_=deg2rad(r_);
% % 
% % %simQuad( sys, innerController, x0, T, r_);
% % 
% % %pause
% % 
% % %% Changing reference
% % r_changing(:,1) = linspace(0,4);
% % r_changing(:,2) = linspace(0,10);
% % r_changing(:,3) = linspace(0,10);
% % r_changing(:,4) = linspace(0,10);
% % 
% % r_changing = deg2rad(r_changing);
% % 
% % %simQuad( sys, innerController, x0, T, r_changing');
% % 
% % 
% % %% Nonlinear model simulation - no disturbance
% % fprintf('Running the FIRST NL model simulation...\n')
% % sim('simulation1.mdl')
% % 
% % %pause
% % 
% % %% Disturbance estimation
% % %estimator
% % %Augmented model
% % A_=[A eye(7);zeros(7,7) eye(7)];
% % B_=[B;zeros(7,4)];
% % C_=[eye(7) zeros(7,7)];
% % 
% % %Observable
% % %F=linspace(0.7,0.71,14);
% % %F=0.8*ones(1,14);
% % F(1)=0.12;
% % F(2)=0.2;
% % F(3)=0.15;
% % F(4)=0.23;
% % F(5)=0.24;
% % F(6)=0.31;
% % F(7)=0.22;
% % F(8)=0.26;
% % F(9)=0.11;
% % F(10)=0.19;
% % F(11)=0.005;
% % F(12)=0.151;
% % F(13)=0.06;
% % F(14)=0.06;
% % K = place(A_',C_',F);
% % L=K';
% % A_obs = A_-L*C_;
% % B_obs = [B_ L];
% % 
% % % Inputs u(1), ..., u(N)
% % u = sdpvar(repmat(nu,1,N),ones(1,N));
% % % States x(1), ..., x(N)
% % x = sdpvar(repmat(nx,1,N),ones(1,N));
% % 
% % filter.Af=A_obs;
% % filter.Bf=B_obs;
% % 
% % % Disturbance
% % d_hat = sdpvar(repmat(nx,1,1),ones(1,1));
% % x = sdpvar(repmat(nx,1,N),ones(1,N));
% % 
% % x_f=[x{1};d_hat];
% % x_f = A_obs*x_f + B_obs*[u{1};x_f(1:7)];
% % d_hat=x_f(8:end);
% % 
% % constraints = [];
% % objective = 0;
% % for k = 1:N-1
% %     objective = objective + (x{k}-x_r)'*Q*(x{k}-x_r) + (u{k}-u_r)'*R*(u{k}-u_r);
% %     constraints = [constraints, x{k+1} == A*x{k} + B*u{k} + x_f(8:end)];
% %     constraints = [constraints, G*(u{k}+us)<= g];%, H*x{k}<=h];
% % end
% % constraints = [constraints, x_r == A*x_r + B*u_r];
% % constraints = [constraints, H*x_r<=h, r == C*x_r];
% % objective = objective + (x{N}-x_r)'*P*(x{N}-x_r); % Terminal weight
% % 
% % innerController = optimizer(constraints,objective,options,[x{1};r;d_hat],u(:,1));
% % 
% % % simQuad( sys, innerController, x0, T, r_, filter);
% % 
% % 
% % %% Final simulation
% % fprintf('Running the FINAL NL model simulation...\n')
% % %sim('simulation2.mdl')
% % %pause
% % %% BONUS - Slew rate constraints
% % % run after doing nonlinear simulations otherwise the NL simulations won't
% % % work (because of the additional controller argument)
% % fprintf('BONUS - SLEW RATE CONSTRAINTS...\n')


% loads:
%    hovering equilibrium (xs,us)
%    continuous time matrices Ac,Bc of the linearization
%    matrices sys.A, sys.B of the inner-loop discretized with sampling period sys.Ts
%    outer controller optimizer instance
clear all
close all
load('quadData.mat')
outerController = getOuterController(Ac, 'quadprog');
disp('Data successfully loaded')



%% %%%%%%%%%%%%%% First MPC controller %%%%%%%%%%%%%%%%%%%
close all
DEG_TO_RAD = pi/180;
T=10;
x0 = [-1;9.5*DEG_TO_RAD;-10*DEG_TO_RAD;120*DEG_TO_RAD;0;0;0];


%MPC data
A=sys.A;
B=sys.B;
H = [1 0 0 0 0 0 0;...
    0 1 0 0 0 0 0;...
    0 0 1 0 0 0 0;...
    0 0 0 0 1 0 0;...
    0 0 0 0 0 1 0;...
    0 0 0 0 0 0 1;...
    -1 0 0 0 0 0 0;...
    0 -1 0 0 0 0 0;...
    0 0 -1 0 0 0 0;...
    0 0 0 0 -1 0 0;...
    0 0 0 0 0 -1 0;...
    0 0 0 0 0 0 -1];
h = [1;10*DEG_TO_RAD;10*DEG_TO_RAD;15*DEG_TO_RAD;15*DEG_TO_RAD;60*DEG_TO_RAD];
h=[h;h];
% Q=diag([50 120 100 1 0 0 0 ]);
% R=0.01*eye(4);

%  Q=diag([500 2000 2000 10 1  1  1 ]);
% R=1*eye(4);
Q=diag([10 100 50 1 5 5  1 ]);
R=0.01*eye(4);

G = [1,0,0,0;...
    0,1,0,0;...
    0,0,1,0;...
    0,0,0,1;...
    -1,0,0,0;...
    0,-1,0,0;...
    0,0,-1,0;...
    0,0,0,-1];

g = [1;1;1;1;0;0;0;0];
N=25;

%Controller
[K,~,~]=dlqr(A,B,Q,R);
K=-K;

%Constraints area
%Xf=exo3(A,B,H,h,G,g,K);
[P,~,~] = dare(A,B,Q,R);


nx=7;
nu=4;
x_s=xs(6:end);

% Inputs u(1), ..., u(N)
u = sdpvar(repmat(nu,1,N),ones(1,N));
% States x(1), ..., x(N)
x = sdpvar(repmat(nx,1,N),ones(1,N));

constraints = [];
objective = 0;
for k = 1:N-1
    objective = objective + (x{k}-x_s)'*Q*(x{k}-x_s) + (u{k}-us)'*R*(u{k}-us);
    constraints = [constraints, x{k+1} == A*x{k} + B*u{k}];
    constraints = [constraints, G*(u{k}+us)<= g , H*x{k}<=h];
end
objective = objective + (x{N}-x_s)'*P*(x{N}-x_s); % Terminal weight
constraints = [constraints, (H*x{N}<=h)];

options = sdpsettings('solver','quadprog');

innerController_s = optimizer(constraints, objective, options, x(:,1), u(:,1));

simQuad( sys, innerController_s, x0, T);

pause

%% Reference tracking - no disturbance, no invariant sets
fprintf('PART II - reference tracking...\n')

C=[eye(4) zeros(4,3)];

Ns = 1;
nr=4;

% Inputs u(1), ..., u(N)
u_r = sdpvar(repmat(nu,1,Ns),ones(1,Ns));
% States x(1), ..., x(N)
x_r = sdpvar(repmat(nx,1,Ns),ones(1,Ns));
% r
r = sdpvar(repmat(nr,1,Ns),ones(1,Ns));

constraints = [];
objective = 0;
for k = 1:N-1
    objective = objective + (x{k}-x_r)'*Q*(x{k}-x_r) + (u{k}-u_r)'*R*(u{k}-u_r);
    constraints = [constraints, x{k+1} == A*x{k} + B*u{k}];
    constraints = [constraints, G*(u{k}+us)<= g , H*x{k}<=h];
end
constraints = [constraints, x_r == A*x_r + B*u_r];
constraints = [constraints, H*x_r<=h, r == C*x_r];
objective = objective + (x{N}-x_r)'*P*(x{N}-x_r); % Terminal weight

innerController = optimizer(constraints,objective,options,[x{1};r],[u{1};x_r]);
r_=[0.5;4;6;80];
r_=deg2rad(r_);
r_(1)=0.5;

anns=innerController{[x0;r_]};
%simQuad( sys, innerController, x0, T, r_);

pause

%% Changing reference
r_changing(:,1) = linspace(0,3);
r_changing(:,2) = linspace(0,9);
r_changing(:,3) = linspace(0,9);
r_changing(:,4) = linspace(0,9);

r_changing = deg2rad(r_changing);

%simQuad( sys, innerController, x0, T, r_changing');


%% Nonlinear model simulation - no disturbance
fprintf('Running the FIRST NL model simulation...\n')
sim('simulation1.mdl')

pause

%% Disturbance estimation
%estimator
%Augmented model
close all
r_=[0.5;-7;-3;40];
r_=deg2rad(r_);
r_(1)=0.5;

%Q=diag([300 1600 1200 10 1  1  1 ]);
R=0.01*eye(4);
Q=diag([10 100 50 1 4 4  1 ]);

F(1:7)=linspace(0.65,0.6,7);
F(8:13)=linspace(0.6,0.7,6);
F(14)=0.5;

A_=[A eye(7);zeros(7,7) eye(7)];
B_=[B;zeros(7,4)];
C_=[eye(7) zeros(7,7)];

%Observable
K = place(A_',C_',F);
L=K';
A_obs = A_-L*C_;
B_obs = [B_ L];

filter.Af=A_obs;
filter.Bf=B_obs;

u = sdpvar(repmat(nu,1,N),ones(1,N));
x = sdpvar(repmat(nx,1,N),ones(1,N));
% Disturbance
d_hat = sdpvar(repmat(nx,1,1),ones(1,1));

x_f=[x{1};d_hat];
x_f = A_obs*x_f + B_obs*[u{1};x_f(1:7)];
d_hat=x_f(8:end);

constraints = [];
objective = 0;
for k = 1:N-1
    objective = objective + (x{k}-x_r)'*Q*(x{k}-x_r) + (u{k}-u_r)'*R*(u{k}-u_r);
    constraints = [constraints, x{k+1} == A*x{k} + B*u{k} + x_f(8:end)];
    constraints = [constraints, G*(u{k}+us)<= g];
end
constraints = [constraints, x_r == A*x_r + B*u_r];
constraints = [constraints, H*x_r<=h, r == C*x_r];
objective = objective + (x{N}-x_r)'*P*(x{N}-x_r); % Terminal weight

innerController = optimizer(constraints,objective,options,[x{1};r;d_hat],u(:,1));

simQuad( sys, innerController, x0, T, r_, filter);


%% Offset free MPC
fprintf('PART III - OFFSET FREE / Disturbance rejection...\n')

pause

%% Final simulation
close all
fprintf('Running the FINAL NL model simulation...\n')
sim('simulation2.mdl')
pause
%% BONUS - Slew rate constraints
% run after doing nonlinear simulations otherwise the NL simulations won't
% work (because of the additional controller argument)
fprintf('BONUS - SLEW RATE CONSTRAINTS...\n')

%estimator
%Augmented model
close all
r_=[0.5;-7;-3;40];
r_=deg2rad(r_);
r_(1)=0.5;

Q=diag([300 1600 1200 10 1  1  1 ]);
R=eye(4);

Delta=0.001;

F(1:7)=linspace(0.65,0.6,7);
F(8:13)=linspace(0.6,0.7,6);
F(14)=0.5;

A_=[A eye(7);zeros(7,7) eye(7)];
B_=[B;zeros(7,4)];
C_=[eye(7) zeros(7,7)];

%Observable
K = place(A_',C_',F);
L=K';
A_obs = A_-L*C_; 
B_obs = [B_ L];

filter.Af=A_obs;
filter.Bf=B_obs;

u = sdpvar(repmat(nu,1,N),ones(1,N));
x = sdpvar(repmat(nx,1,N),ones(1,N));
% Disturbance
d_hat = sdpvar(repmat(nx,1,1),ones(1,1));
u_com = sdpvar(repmat(nu,1,1),ones(1,1));

x_f=[x{1};d_hat];
x_f = A_obs*x_f + B_obs*[u{1};x_f(1:7)];
d_hat=x_f(8:end);

constraints = [];
objective = 0;
for k = 1:N-1
    objective = objective + (x{k}-x_r)'*Q*(x{k}-x_r) + (u{k}-u_r)'*R*(u{k}-u_r);
    constraints = [constraints, x{k+1} == A*x{k} + B*u{k} + x_f(8:end)];
    constraints = [constraints, G*(u{k}+us)<= g, abs(u{k}-u_com)<=Delta];
end
constraints = [constraints, x_r == A*x_r + B*u_r];
constraints = [constraints, H*x_r<=h, r == C*x_r];
objective = objective + (x{N}-x_r)'*P*(x{N}-x_r); % Terminal weight

innerController = optimizer(constraints,objective,options,[x{1};r;u_com;d_hat],u(:,1));

simQuad( sys, innerController, x0, T, r_, filter,[],1);