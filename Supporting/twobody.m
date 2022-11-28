function dy = twobody (~,f)
global mu;
x=f(1);
y=f(2);
z=f(3);
Vx=f(4);
Vy=f(5);
Vz=f(6);

r=norm([x y z]);
ax = -mu*x/r^3;
ay = -mu*y/r^3;
az = -mu*z/r^3;
dy = [Vx  Vy  Vz  ax  ay  az]'; 
end