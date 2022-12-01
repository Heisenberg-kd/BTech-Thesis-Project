function [x,t]=lqr_attitude(tspan,J,x0)
% m=6000;
% r=45000000;

A=[zeros(3,3) zeros(3,3); 0.5*eye(3,3) zeros(3,3)];
B=[inv(J) ;zeros(3,3)];

%%  Design LQR controller
Q = 1.*eye(6);
R = eye(3);

K = lqr(A,B,Q,R);

%% Simulate closed-loop system
% x0 = [-1000;-200;-4000;30;40;50];   % initial condition 
xdes = [0; -100; 0; 0; 0; 0];       % reference position  [w  q]
u=@(x)-K*(x - xdes);                % control law

[t,x] = ode45(@(t,x)diff_equ(x,u(x),J),tspan,x0);

end

function dy = diff_equ(x,u,J)
A=[zeros(3,3) zeros(3,3); 0.5*eye(3,3) zeros(3,3)];
B=[inv(J); zeros(3,3)];

dy = A*x+B*u;
end