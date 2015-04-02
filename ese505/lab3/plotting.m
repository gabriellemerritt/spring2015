clear C
clear figure
filename = 'data_pid.txt'; 
fileID =fopen(filename,'r'); 
format_spec= '%f%f%f'; 
C=textscan(fileID,format_spec,'Delimiter',',');
[time,ball_error, ball_pos] =  C{1,:};  
len = min(size(time,1),size(ball_pos));
time = time - time(1); 
time = time/1000;
figure
plot(time(10:len), ball_pos(10:len),'r');
