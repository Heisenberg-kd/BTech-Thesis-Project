function [x,t]=lqr_attitude(tspan,J,x0,n)

A=[zeros(3,3) zeros(3,3); 0.5*eye(3,3) zeros(3,3)];
B=[inv(J) ;zeros(3,3)];

%%  Design LQR controller
Q = 1.*eye(6);
R = eye(3);

K = lqr(A,B,Q,R);

%% Simulate closed-loop system
% x0 = [-1000;-200;-4000;30;40;50];   % initial condition 
% xdes = @(t)[0; 0; 0; sin(pi*t/32); 0; cos(pi*t/32)];      % reference position  [w  q]
u=@(x,t)-K*(x - [0; 0; 0;0*sin(n*t); 0; 0*cos(n*t)]);                % control law

[t,x] = ode45(@(t,x)diff_equ(x,u(x,t),A,B),tspan,x0);
q0=(ones(size(t))-x(:,4).^2-x(:,5).^2-x(:,6).^2).^0.5;
x=[x q0];
x=x';
end

function dy = diff_equ(x,u,A,B)
dy = A*x+B*u;
end