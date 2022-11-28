function [tout, yout] = rkf4(ode_function, tspan, y0,sample_time)

h=sample_time;
t0 = tspan(1);
tf = tspan(2);
i=1;
tout=zeros(1,(tf-t0)/h);
yout=zeros(6,(tf-t0)/h);

tout(i) = t0;
yout(:,i) = y0';
while tout(i) < tf
    ti = tout(i);
    yi = yout(:,i);
    k_1=feval(ode_function,ti,yi);
    k_2=feval(ode_function,ti+0.5*h,yi+0.5*h*k_1);
    k_3=feval(ode_function,ti+0.5*h,yi+0.5*h*k_2);
    k_4=feval(ode_function,ti+h,yi+h*k_3);
    yout(:,i+1) = yout(:,i) + ((1/6)*(k_1+2*k_2+2*k_3+k_4)*h);
    tout(i+1) = tout(i)+h;
    i=i+1;
end 
end