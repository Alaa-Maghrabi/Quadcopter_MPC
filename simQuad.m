%INPUTS:
%   sys - a structure containing matrices A1, B1 and the sampling period Ts
%  ctrl - controller (optimizer instance) - the parameter order of the controller is:
%           [x ; ref]                   for simple tracking, where ref is the reference [x ; y ; z ; yaw] (in R^4)
%           [x ; ref ; d_est]           for tracking in the presence of disturbance, where d_est is the estimate of the disturbance    
%           [x ; ref ; u_prev ]         for tracking with slew rate constraints, where u_prev is the previous control input applied to the plant
%           [x ; ref ; u_prev ; d_est ] for tracking with slew rate constraints in the presence of disturbance    
%    x0 - initial state
%     T - simulation time (in seconds)
%   ref - constant reference (4 x 1 vector or a 4 x T*Ts1 signal)
% filter - disturbance estimator - structure containing matrices Af and Bf
%     d - disturbance - if not supplied (or empty - []) a predefined disturbance is used
%         whenever the filter structure is supplied

%OUTPUTS:
%xt - state as a function of time
%ut - input as a function of time
%t - time

function [xt ut t] =  simQuad( sys, ctrl, x0, T, ref, filter, d, useSlewRateConst)
Ts = sys.Ts;
Ns = floor(T / Ts)+1; %integer number of steps
A = sys.A;
B = sys.B;

if(exist('filter','var') && ~isempty(filter) && (~exist('d','var') || isempty(d)))
    d_mean = repmat([-0.1;0;0;0;6e-4;6e-4;3e-4],1,Ns);
    d_rand = [zeros(4,Ns);1e-2*diag([1;1;20])*(rand(3,Ns)-0.5)];
    d = 1*d_mean + 1*d_rand;
end


n1 = size(A,1); m1 = size(B,2);
x = x0; % initial state
xt = zeros(n1,Ns);
ut = zeros(m1,Ns);

if(~exist('useSlewRateConst') || isempty(useSlewRateConst))
    useSlewRateConst = 0;
end

if(exist('ref','var'))
    % padding with the last element of ref
    ref = [ref repmat(ref(:,end),1,Ns-size(ref,2))];
end

if(exist('d','var'))
    % padding with the last element of d
    d = [d repmat(d(:,end),1,Ns-size(d,2))];
    if(~exist('filter'))
        error('Need disturbance estimator!')
    end
    Af = filter.Af;
    Bf = filter.Bf;
else
    d = zeros(n1,Ns);
    filter = [];
end
t = [0:Ns-1] * Ts;

tic
u = zeros(m1,1);
xf = zeros(2*n1,1);
dest = zeros(n1,1);
xft = zeros(2*n1,Ns+1);
for i = 1 : Ns
    xt(:,i) = x;
    if(~exist('ref','var'))
        [u err] = ctrl{x};
    else
        if(useSlewRateConst)
            if(~isempty(filter))
                [u err] = ctrl{[x;ref(:,i);u;dest]}; % slew rate and disturbance
            else
                [u err] = ctrl{[x;ref(:,i);u]}; % slew rate, no disturbance
            end
        else
            if(~isempty(filter))
                [u err] = ctrl{[x;ref(:,i);dest]}; % disturbance, no slew rate
            else
                [u err] = ctrl{[x;ref(:,i)]}; % no distuerbance, no slew rate
            end
            
        end
    end
    if (err)
        yalmiperror(err)
        return
    end
    ut(:,i) = u;
    if(~isempty(filter))
        xf = Af*xf + Bf*[u; x];
        xft(:,i+1) = xf;
    end
    x = A*x + B*u + d(:,i);
    dest = xf(8:end);   
end
toc

theta = xt(2:4,:)';
omega = xt(5:7,:)';
zdot = xt(1,:)';
u = ut';


if(~exist('ref','var'))
    ref = zeros(4,Ns);
end




clear sys
sys.uMin  = -0.70071429*ones(4,1);
sys.uMax  = ones(4,1)-0.70071429*ones(4,1);
sys.angleMin = -[1;1]*(10/180)*pi;
sys.angleMax =  [1;1]*(10/180)*pi;
sys.zVelMin = -1;
sys.zVelMax = 1;
sys.angVelMin   = -[15;15;60]*pi/180;
sys.angVelMax   = [15;15;60]*pi/180;


figure(1); clf; grid on; hold on;
plot(t, theta(:,1)*180/pi, t, theta(:,2)*180/pi, 'LineWidth',1.1);
plot(t, ref(2,:)*180/pi,'--', t, ref(3,:)*180/pi,'--','LineWidth',1.0);
plot(t,repmat(sys.angleMin(1)*180/pi,Ns)','--','Color','Red','LineWidth',2)
plot(t,repmat(sys.angleMax(1)*180/pi,Ns)','--','Color','Red','LineWidth',2)
legend('Roll', 'Pitch', 'Roll ref', 'Pitch ref', ' Constraints');
ylabel('deg'); xlabel('s')


figure(2); clf; grid on; hold on;
plot(t, theta(:,3)*180/pi,'LineWidth',1.1);
plot(t,180*ref(4,:)/pi,'--')
legend('Yaw','Yaw ref');
ylabel('deg'); xlabel('s')


figure(3); clf; grid on; hold on;
plot(t,repmat(sys.uMin(1),Ns)','--','Color','Red','LineWidth',2)
plot(t,repmat(sys.uMax(1),Ns)','--','Color','Red','LineWidth',2)
plot(t, u(:,1), t, u(:,2), t, u(:,3), t, u(:,4),'LineWidth',1.1);
legend('Constraints', 'rotor speed 1', 'rotor speed 2', 'rotor speed 3', 'rotor speed 4');
axis([0,t(end),sys.uMin(1)-0.1,0.1+sys.uMax(1)])
ylabel('u'); xlabel('s')


figure(4); clf; grid on; hold on;
plot(t, zdot,'LineWidth',1.1);
plot(t,ref(1,:),'--');
plot(t,repmat(sys.zVelMin,Ns)','--','Color','Red','LineWidth',2)
plot(t,repmat(sys.zVelMax,Ns)','--','Color','Red','LineWidth',2)
legend('zdot', 'zdot ref','Constraints');
ylabel('m / s'); xlabel('s')

figure(5); clf; grid on; hold on;
plot(t, omega(:,1)*180/pi, t, omega(:,2)*180/pi,'LineWidth',1.1);
plot(t,repmat(sys.angVelMin(1)*180/pi,Ns)','--','Color','Red','LineWidth',2)
plot(t,repmat(sys.angVelMax(1)*180/pi,Ns)','--','Color','Red','LineWidth',2)
legend('Roll rate', 'Pitch rate', 'Constraints');
ylabel('deg / s'); xlabel('s')


figure(6); clf; grid on; hold on;
plot(t, omega(:,3)*180/pi,'LineWidth',1.1);
plot(t,repmat(sys.angVelMin(3)*180/pi,Ns)','--','Color','Red','LineWidth',2)
plot(t,repmat(sys.angVelMax(3)*180/pi,Ns)','--','Color','Red','LineWidth',2)
legend('Yaw rate', ' Constraints');
ylabel('deg / s'); xlabel('s')



if(~isempty(filter))
    figure(8); clf; grid on; hold on;
    plot(t, xft(8,1:end-1), 'LineWidth',1.1);
    legend('dz dist'); 
    title('z dot disturbance estimate')
    
    figure(9); clf; grid on; hold on;
    plot(t, xft(14,1:end-1), 'LineWidth',1.1);
    legend('dyaw dist'); 
    title('yaw dost disturbance estimate')
    
    figure(10); clf; grid on; hold on;
    plot(t,xft(12,1:end-1), t,xft(13,1:end-1), 'LineWidth', 1.1);
    legend('droll dist', 'dpitch dist');
    title('roll dot and pitch dot disturbance estimates')
end

% figure(7); clf; grid on; hold on;
% plot(t, u1(:,1), t, u1(:,2)*180/pi, t, u1(:,3)*180/pi);
% legend('vz_{ref}', 'roll_{ref}', 'pitch_{ref}');
%
% figure(8); clf; grid on; hold on;
% plot(t, d1(:,1), t, d1(:,2), t, d1(:,3), t, d1(:,4));
% legend('dz_{dist}', 'droll_{dist}', 'dpitch_{dist}','dyaw_{dist}');
%
% figure(9); clf; grid on; hold on;
% plot(t, d2(:,1), t, d2(:,2), t, d2(:,3), t, d2(:,4),t, d2(:,5));
% legend('x_{dist}', 'vx_{dist}', 'y_{dist}','vy_{dist}','z_{dist}');



end

