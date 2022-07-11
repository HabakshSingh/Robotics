Ts=0.1;
t=0:Ts:2;
L1=0.2;L2=0.2;L3=0.2;
%we will create a path to be traced by the articulated arm from the way
%points.
x=0.2*ones(1,length(t));
y=linspace(-0.2,0.2,length(x));
z=L1*ones(1,length(x));
theta1=zeros(1,length(t));
theta2=zeros(1,length(t));
theta3=zeros(1,length(t));
TM{length(t)}=zeros(4,4);
%now we will use inverse kinematics and find the various angles of the
%actuators to trace this path in the robot.Use the method of the book
%first we need to convert it into transformation matrix.
for i=1:length(t)
    tt=[x(i) y(i) z(i)];
    TM{i}=trvec2tform(tt);
    %we have everything to find joint angles.
    theta1(1,i)=atan2(TM{i}(2,4),TM{i}(1,4))*180/pi; 
    r=sqrt(x(i)^2+y(i)^2);
    theta2(1,i)=acos(r/(0.4))*180/pi;
    theta3(i)=-2*theta2(i);
end

%now, we will use forward knimetics block and try to find the final
%position of end effector.
%measured end effector positions xm,ym and zm
xm=ones(1,length(t));
ym=ones(1,length(t));
zm=ones(1,length(t));
T30{length(t)}=zeros(4,4);
for j=1:length(t)
    T30{j}=[cosd(theta1(j))*cosd(theta2(j)+theta3(j)) -cosd(theta1(j))*sind(theta2(j)+theta3(j)) -sind(theta1(j)) cosd(theta1(j))*(L3*cosd(theta2(j)+theta3(j))+L2*cosd(theta2(j)));
        sind(theta1(j))*cosd(theta2(j)+theta3(j)) -sind(theta1(j))*sind(theta2(j)+theta3(j)) cosd(theta1(j)) sind(theta1(j))*(L3*cosd(theta2(j)+theta3(j))+L2*cosd(theta2(j)));
        sind(theta2(j)+theta3(j)) cosd(theta2(j)+theta3(j)) 0 L3*sind(theta2(j)+theta3(j))+L2*sind(theta2(j));
        0 0 0 1];
    xm(j)=T30{j}(1,4);
    ym(j)=T30{j}(2,4);
    zm(j)=0.2;
end

%we have proved both inverse and forward kinematics.Now, lets build an
%robot and animate it.
%Link 1 is from (0,0,0) to (0,0,L1)
%Link 2 is from (0,0,L1) to (0.2*cos(theta2)*cos(theta1),0.2*cos(theta2)*sin(theta1),L1+L2*sind(theta2(i)))

plot3([0 0],[0 0],[0 0.2],'Color',[0.5 0.5 0],'LineWidth',2); %Link1
axis([-0.3 0.3 -0.3 0.3 0 0.5]);
xlabel('x-axis','FontSize',10,'Color',[1,0,0]);
ylabel('y-axis','FontSize',10,'Color',[1,0,0]);
zlabel('z-axis','FontSize',10,'Color',[1,0,0]);
grid on
hold on
h1=plot3([0 0],[0 0],[0 0],'Color',[1 0 0],'LineWidth',2); %link2
h2=plot3([0 0],[0 0],[0 0],'Color',[0 1 0],'LineWidth',2); %link3
h3=plot3(x(1),y(1),z(1),'o',"Color",[0 1 0],"LineWidth",2); %text
h4=plot3(x(1),y(1),z(1),'--','Color',[0.5 0 0.5],'LineWidth',2); %trajectory
j=0;
while j<=5
    for i=(2:length(t))
        set(h1,'XData',[0 L2*cosd(theta2(i))*cosd(theta1(i))],'YData',[0 L2*cosd(theta2(i))*sind(theta1(i))],'ZData',[0.2 L1+L2*sind(theta2(i))]);
        set(h2,'XData',[L2*cosd(theta2(i))*cosd(theta1(i)) x(i)],'YData',[L2*cosd(theta2(i))*sind(theta1(i)) y(i)],'ZData',[L1+L2*sind(theta2(i)) 0.2]);
        set(h4,'XData',x(1:i),'YData',y(1:i),'ZData',z(1:i))
        %h3.XData=x(i);
        %h3.YData=y(i);
        %h3.ZData=z(i);
        s=sprintf('%0.4f,%0.4f ,%0.4f',x(i),y(i),z(i));
        h3=text(x(i),y(i),z(i),s,'Color',[1 0 0],'FontSize',10);
        pause(0.1);
        delete(h3);
    end
    j=j+1;
end
hold off

