function [x,t]=lqr_attitude_nadir(tspan,J,x0,n,h)

h2=h(2);
f41=8*(J(3,3)-J(2,2))*n*n+2*n*h2;
f46=(J(1,1)-J(2,2)+J(3,3))*n+h2;
f52=6*(J(3,3)-J(1,1))*n*n;
f63=2*(J(1,1)-J(2,2))*n*n+2*h2*n;
C=[eye(3,3) zeros(3,3);zeros(3,3) J];
D=[zeros(3,3) 0.5*eye(3,3);f41 0 0 0 0 f46;...
    0 f52 0 0 0 0;0 0 f63 -f46 0 0];

A=C\D;
B=[zeros(3,3);inv(J)];

%%  Design LQR controller
Q = 1.*eye(6);
R = eye(3);

K = lqr(A,B,Q,R);

%% Simulate closed-loop system
% x0 = [-1000;-200;-4000;30;40;50];   % initial condition 
% xdes = @(t)[0; 0; 0; sin(pi*t/32); 0; cos(pi*t/32)];      % reference position  [w  q]
u=@(x,t)-K*(x - [0; 0; 0;0*sin(n*t); 0; 0*cos(n*t)]);                % control law

[t,x] = ode45(@(t,x)diff_equ(x,u(x,t),A,B),tspan,x0);
x=[x(:,4:6) x(:,1:3)];
q0=(ones(size(t,1))-x(:,4).^2-x(:,5).^2-x(:,6).^2).^0.5;
x=[x q0];
end

function dy = diff_equ(x,u,A,B)
dy = A*x+B*u;
end