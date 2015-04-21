clear Wp
Wp = struct; 
Wp.p0 = zeros(2,108);  
Wp.p1 = zeros(2,108);
Wp.p2 = zeros(2,108);
Wp.p3 = zeros(2,108);
Wp.p4 = zeros(2,108);  
x  = 0; 
y = -.152*2; 
tag_id = 0; 
for i = 1:9 
    x  = 0;
    if ( tag_id == 36 || tag_id == 72) 
            y = y+.152+ .178;            
    else
        y = y + .152*2;
    end
    for j = 1:12
      
        Wp.p1(:,tag_id+1) = [x+.152;y];  
        Wp.p2(:,tag_id+1)=  [x+.152;y+ .152];  
        Wp.p3(:,tag_id+1) = [x;y+ .152];  
        Wp.p4(:,tag_id+1) = [x;y];  
        Wp.p0(:,tag_id+1) = Wp.p4(:,tag_id+1) + ((Wp.p2(:,tag_id+1) - Wp.p4(:,tag_id+1))/2);      
        tag_id = tag_id+1;    
        x = x + .3040;
    end
     
end 
%     plot(Wp.p2(1,:), Wp.p2(2,:),'*b')
%     hold on 
%     plot(Wp.p4(1,:), Wp.p4(2,:),'*r'); 
%     plot(Wp.p3(1,:), Wp.p3(2,:), '*g'); 
%     plot(Wp.p1(1,:),Wp.p1(2,:), '*y');
%     axis('equal'); 