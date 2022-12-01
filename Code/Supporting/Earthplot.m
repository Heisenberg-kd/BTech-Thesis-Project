function Earthplot(y1,y2)
global Re ;

i1=y1(:,1);
i2=y2(:,2);
hold off
r = 0.8; g = r; b = r;
map = [r g b;0 0 0; r g b];

[xx, yy, zz] = sphere(100);
surf(Re*xx, Re*yy, Re*zz)
colormap(map)
caxis([-Re/100 Re/100])
shading flat

% Draw and label the X, Y and Z axes
line([0 2*Re], [0 0], [0 0]); text(2*Re, 0, 0, 'X')
line( [0 0], [0 2*Re], [0 0]); text( 0, 2*Re, 0, 'Y')
line( [0 0], [0 0], [0 2*Re]); text( 0, 0, 2*Re, 'Z')

if(nargin==1)
xo10=i1(1);
yo10=i1(2);
zo10=i1(3);
% Plot the orbit, draw a radial to the starting point
% and label the starting point (o) and the final point (f) for Planet-1

hold on
plot3( y1(1,:), y1(2,:), y1(3,:),'k')
line([0 xo10], [0 yo10], [0 zo10],'LineStyle','--')
text( y1(1,1), y1(2,1), y1(3,1), 'o')
text( y1(1,end), y1(2,end), y1(3,end), 'f')
line([0 y1(1,end)],[0 y1(2,end)],[0 y1(3,end)],'Color','red','LineStyle','--')
% Select a view direction (a vector directed outward from the origin)
view([1,1,.4])
hold off

else
xo10=i1(1);
yo10=i1(2);
zo10=i1(3);
xo20=i2(1);
yo20=i2(2);
zo20=i2(3);
% Plot the orbit, draw a radial to the starting point
% and label the starting point (o) and the final point (f) for Planet-1
hold on
plot3( y1(1,:), y1(2,:), y1(3,:),'k')
line([0 xo10], [0 yo10], [0 zo10],'LineStyle','--')
text( y1(1,1), y1(2,1), y1(3,1), 'o')
text( y1(1,end), y1(2,end), y1(3,end), 'f')
line([0 y1(1,end)],[0 y1(2,end)],[0 y1(3,end)],'Color','red','LineStyle','--')
% Select a view direction (a vector directed outward from the origin)
view([1,1,.4])
% Plot the orbit, draw a radial to the starting point
% and label the starting point (o) and the final point (f) for Planet-2.
plot3( y2(1,:), y2(2,:), y2(3,:),'k')
line([0 xo20], [0 yo20], [0 zo20],'Color','cyan')
text(y2(1,1), y2(2,1), y2(3,1), 'o')
text( y2(1,end), y2(2,end), y2(3,end), 'f')
line([0 y2(1,end)],[0 y2(2,end)],[0 y2(3,end)],'Color','green')
hold off
end

% Specify some properties of the graph
grid on
axis equal
xlabel('km')
ylabel('km')
zlabel('km')
hold off