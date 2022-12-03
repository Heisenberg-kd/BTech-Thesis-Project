function subplot3(y,T)

hold off
subplot(2,1,1)
title("Plot of Satellite Parameters");
plot(T,y(1,:),'-k')
hold on
plot(T,y(2,:),'-b')
plot(T,y(3,:),'-r')
title('Angular Velocity')
legend('wx','wy','wz')
hold off

subplot(2,1,2)
plot(T,y(7,:),"-y")
hold on
plot(T,y(4,:),'-k')
plot(T,y(5,:),'-b')
plot(T,y(6,:),'-r')
title('Quaternions')
legend('q0','q1','q2','q3')
hold off

end