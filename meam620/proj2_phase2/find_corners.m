function corners = find_corners(data) 
C = cell(size(data,2),1); 
for j= 1:size(data,2) 
    i = data(j).img; 
    c = corner(i,1000); 
    C{j} = c; 

end 
corners = C; 


% image(im/4)
% axis image
% colormap(gray)
% hold on
% plot(cs(:,1), cs(:,2), 'r.')
% plot(c(:,1), c(:,2), 'g.')
% legend('9 point FAST corners', 'nonmax-suppressed corners')
% title('9 point FAST corner detection on an image')

