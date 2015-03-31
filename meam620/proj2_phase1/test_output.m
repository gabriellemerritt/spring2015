 
for i = 1:864
    sensor = data(i); 
    [pos,eul]=estimate_pose_handle(sensor); 
   P(i,:) = pos; 
   E(i,:) = eul; 
   T(i) = sensor.t; 
end
figure
plot(time, vicon(1,:),'r');
hold on; 
plot(T, P(:,1),'b'); 
hold off; 
figure
hold on;
plot(time,vicon(2,:),'r'); 
plot(T,P(:,2),'b'); 
hold off;
figure
hold on
plot(time,vicon(3,:),'r');
plot(T,P(:,3),'b');