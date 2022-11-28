function subplot2(y,T)

hold off
subplot(2,1,1)
title("Plot of Sattlite");
plot(T,y(1,:),'-k')
hold on
plot(T,y(2,:),'-b')
plot(T,y(3,:),'-r')
title('Distance')
legend('x','y','z')
hold off

subplot(2,1,2)
plot(T,y(4,:),'-k')
hold on
plot(T,y(5,:),'-b')
plot(T,y(6,:),'-r')
title('Velocity ')
legend('Vx','Vy','Vz')
hold off

end