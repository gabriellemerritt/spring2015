figure;
for i=1:864 
  imagesc(data(i).img);
  hold on;
  plot(data(i).p0(1,:),data(i).p0(2,:),'*g');
    drawnow
    hold off;
  
   
end
