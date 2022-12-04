function [x,y,t]=lqr_attitude_nadir(tspan,J,x0,n,h,C)

h2=h(2);
f41=8*(J(3,3)-J(2,2))*n*n+2*n*h2;
f46=(J(1,1)-J(2,2)+J(3,3))*n+h2;
f52=6*(J(3,3)-J(1,1))*n*n;
f63=2*(J(1,1)-J(2,2))*n*n+2*h2*n;
CC=[eye(3,3) zeros(3,3);zeros(3,3) J];
D=[zeros(3,3) 0.5*eye(3,3);f41 0 0 0 0 f46;...
    0 f52 0 0 0 0;0 0 f63 -f46 0 0];

A=CC\D;
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
q0_column=(ones(size(t))-x(:,4).^2-x(:,5).^2-x(:,6).^2).^0.5;
x=[x q0_column];
x=x';
y=zeros(3,length(t));
for i =1:size(t)
    q0=x(7,i);
    q1=x(4,i);
    q2=x(5,i);
    q3=x(6,i);
    Alb(1,:)=[2*q0*q0-1+2*q1*q1, 2*q1*q2+2*q0*q3 ,2*q1*q3-2*q0*q2];
    Alb(2,:)=[(2*q1*q2-2*q0*q3) , (2*q0*q0-1+2*q2*q2), (2*q3*q2+2*q0*q1)];
    Alb(3,:)=[2*q1*q3+2*q0*q2, 2*q2*q3-2*q0*q1, 2*q0*q0-1+2*q3*q3];
    y(1:3,i)=(C(:,:,i)*Alb)\x(1:3,i);    %y is angular velocity of BOdy Coordinate system wrt intertail frame in inertial frame.
end
end

function dy = diff_equ(x,u,A,B)
dy = A*x+B*u;
end