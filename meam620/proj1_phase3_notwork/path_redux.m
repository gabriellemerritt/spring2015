function [reduced_path ] = path_redux(map,path,astar, min_res) 

res  = max(size(map{1,1}))/min_res; 
redux = [];
red2 = [];
temp = path(1,:);
j = 1; 
if isempty(path) 
    reduced_path = path; 
       
else
    for i=2:size(path,1)-1;
         temp = path(i,:) - path(i-1,:);
         temp2 = path(i+1,:) - path(i,:); 
         cond = norm(temp)*norm(temp2) == dot(temp,temp2); 
       if (~cond)
           redux(j,:) = path(i,:);
           j = j+1; 
       end

    end

    k =0;
    flag = 0; 
    n = size(redux,1); 
    l = 0;
    counter = 1;
    while(k < size(redux,1))
        if counter > size(redux,1)
            break;
        end
        k = k+1; 
        for j = 1:size(redux,1)
            if((size(redux,1) - j )==0)
                break;
            end
            samplex = linspace(redux(k,1),redux(end-j,1),res)'; 
            sampley = linspace(redux(k,2),redux(end-j,2),res)'; 
            samplez = linspace(redux(k,3),redux(end-j,3),res)'; 
            samples = [samplex sampley samplez]; 
            c = collide(map,samples); % logical vector 
            if(~any(c))
                if l ==k 
                    break;
                end     
               red2 =[ red2;redux(end -j,:)];
               
               if(k < n - j-1)
                     k = n -j-1; 
               end
               break;
            end
            if isempty(red2)
                continue;
            elseif (red2(end,:) == redux(end,:))
                flag =1; 
                break; 
            end 

        end
        if (flag ==1) 
            break;
        end 
         l =k;
         counter = counter+1;
         
    end 
    
    r_path = [path(1,:);red2;path(end,:)];
    for k =1:size(r_path,1) -1
      sampx = linspace(r_path(k,1),r_path(k+1,1),res)'; 
      sampy = linspace(r_path(k,2),r_path(k+1,2),res)'; 
      sampz = linspace(r_path(k,3),r_path(k+1,3),res)'; 
      s= [sampx sampy sampz]; 
      c = collide(map,s); 
      if(any(c))
          reduced_path = [path(1,:)];
          return;
      end
    end
    reduced_path = unique(r_path, 'rows','stable'); 
    

end
