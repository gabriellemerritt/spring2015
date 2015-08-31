figure;
load('data/studentdata4'); 
for i=1:size(data,2) 
  imagesc(data(i).img);
  hold on;
  plot(data(i).p0(1,:),data(i).p0(2,:),'*g');
    drawnow
    hold off;
  
   
end
