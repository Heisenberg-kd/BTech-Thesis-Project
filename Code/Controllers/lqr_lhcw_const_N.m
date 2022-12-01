function [x,t]=lqr_lhcw_const_N(x0,tspan,n)


% m=6000;
% r=45000000;

A=[0 0 0 1 0 0; 
   0 0 0 0 1 0;
   0 0 0 0 0 1;
   3*n*n 0 0 0 2*n 0;
   0 0 0 -2*n 0 0;
   0 0 -n*n 0 0 0];
B=[0 0 0;
   0 0 0;
   0 0 0;
   1 0 0;
   0 1 0;
   0 0 1];

%%  Design LQR controller
Q = 1.*eye(6);
R = eye(3);

K = lqr(A,B,Q,R);

%% Simulate closed-loop system
% x0 = [-1000;-200;-4000;30;40;50];   % initial condition 
xdes = [0; -100; 0; 0; 0; 0];       % reference position
u=@(x)-K*(x - xdes);                % control law

[t,x] = ode45(@(t,x)diff_equ(x,u(x),n),tspan,x0);

end

function dy = diff_equ(x,u,n)
A=[0 0 0 1 0 0; 
   0 0 0 0 1 0;
   0 0 0 0 0 1;
   3*n*n 0 0 0 2*n 0;
   0 0 0 -2*n 0 0;
   0 0 -n*n 0 0 0];
B=[0 0 0;
   0 0 0;
   0 0 0;
   1 0 0;
   0 1 0;
   0 0 1];

dy = A*x+B*u;
end