function[y4,t,sv32o] =Linear_HCW(init,tspan,C,Omega20,y2)
% Position Control of Chaser using Linearized HCW equations.
% Here w will be Changing for every time stamp so will implement it like
% Non Linear Model.
%% Implementing Time Varying Angular Velocity System.
[t,y4] = rkf4(@(t,sv)HCW_eq(t,sv,tspan,y2),[tspan(1),tspan(end)],init,tspan(2)-tspan(1));
sv32o=zeros(6,length(y4));
 for k=1:length(y4)
 sv32o(1:3,k)=C(:,:,k)\y4(1:3,k);
 sv32o(4:6,k)=C(:,:,k)\(y4(4:6,k)-cross(y4(1:3,k),Omega20(1:3,k)));
 end
end

function dydt= HCW_eq(t,sv,tspan,f) 
%Omega20o=interp1(tspan,Omega20,t);
f=interp1(tspan,f',t);
% f=f(fix(i),:);

R20 = f(1:3);
V20 = f(4:6);
% 
R20_mag=norm(R20);
% Omega20o=cross(R20,V20)/R20_mag^2;
% 
% exh=R20/norm(R20);%h is used for hills frame.
% ezh=cross(R20,V20)/norm(cross(R20,V20));
% eyh=cross(ezh,exh);
% C=[exh; eyh; ezh ];


% r=norm([x y z]);
% Omega20=(mu/(r^3))^0.5;
H=cross(R20,V20);
h=norm(H);
% Omega20o= (cross([x y z],[Vx Vy Vz])/r^2);
Omega20=h/R20_mag;

% A=[zeros(3), eye(3);
%    2*mu/R20_mag^3+ h^2/R20_mag^4 ,-2*dot(V20,R20)*h/R20_mag^4,0,0,2*h/R20_mag^2,0;
%    -2*dot(V20,R20)*h/R20_mag^4,h^2/R20_mag^4-mu/R20_mag^3,0,-2*h/R20_mag^2,0,0;
%    0,0 ,-mu/R20_mag^3,0,0,0 ];

A=[zeros(3), eye(3);
   3*Omega20^2 ,0, 0, 0, 2*Omega20, 0;
   0 ,0 ,0 ,-2*Omega20 ,0 ,0 ;
   0 ,0, -(Omega20^2), 0, 0, 0 ];

dydt = A*sv;
% i=i+0.25;
% dy = [Vx  Vy  Vz  ax  ay  az]'; 
end