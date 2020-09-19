function path = globalpathplanning_RP(map,obstacles,startpoint,points_to_go,radius)


ifSpecialCase = false; specialObIdx = [];
for i = 1:length(obstacles)
    xv = obstacles{i}(:,1);  yv = obstacles{i}(:,2);
    [in,on] = inpolygon(startpoint(1),startpoint(2),xv,yv);
    if in || on
        ifSpecialCase = true;
        specialObIdx = [specialObIdx,i];        
    end
end

[V,E] = createRoadmap(map,obstacles,[],[],false,specialObIdx); 

% for i = 1:size(E,1)
%     plotEdge(V(E(i,1),:),V(E(i,2),:));
% end

all_points = [startpoint;points_to_go];     Value_M = ones(size(all_points,1),size(all_points,1))*1000;  
scatter(all_points(:,1),all_points(:,2),'kx')
for i = 1:size(all_points,1)
    for j = 1+i:size(all_points,1)
        if i == 1
        [cost, rute_index,rute] = findPath(map,obstacles,V,E,all_points(i,:),all_points(j,:),ifSpecialCase,specialObIdx);
        visitpoints{j,i} = rute;   visitpoints{i,j} = flipud(rute); 
%                     plot(visitpoints{i,j}(:,1),visitpoints{i,j}(:,2),'g')
%                     hold on
        Value_M(j,i) = cost;    Value_M(i,j) = cost;
        else
                    [cost, rute_index,rute] = findPath(map,obstacles,V,E,all_points(i,:),all_points(j,:),false,specialObIdx);
        visitpoints{j,i} = rute;   visitpoints{i,j} = flipud(rute); 
%                     plot(visitpoints{i,j}(:,1),visitpoints{i,j}(:,2),'g')
%                     hold on
        Value_M(j,i) = cost;    Value_M(i,j) = cost;
        end
    end
end

path2 = startpoint;
[p,L2] = tspsearch(Value_M,size(Value_M,1));
index = find(p==1);  p = [p(index:end),p(1:index-1)];
for i = 1:length(p)-1
%     path2 = [path2;visitpoints{p(i),p(i+1)}(2:end,:)]; 
%     plot(visitpoints{p(i),p(i+1)}(:,1),visitpoints{p(i),p(i+1)}(:,2),'r');
     hold on
end

m = 1; path1 = startpoint; i = 1; L1 = 0;
while i < size(all_points,1)     
    [~,next_point_index] = min(Value_M(m,:));   L1 = L1 + Value_M(m,next_point_index) ; Value_M(:,m) = 1000;  
    path1 = [path1; visitpoints{m,next_point_index}(2:end,:)];
%      plot(visitpoints{m,next_point_index}(:,1),visitpoints{m,next_point_index}(:,2),'b');
%      hold on
    m = next_point_index;
    i = i+1;
end


if L1 < L2
    path = path1;
else
    path = path2;
end
path = [path, zeros(size(path,1),1)]; 
for i = 1:size(points_to_go,1)
    path(sum(path(:,1:2) - points_to_go(i,:),2)==0,3) = 1; 
end 
path = [path, zeros(size(path,1),1)];
plot(path(:,1),path(:,2),'r')
hold on


function dis = pathvalue(points)
dis = 0;
for i = 1:size(points,1)-1
    dis = norm(points(i+1,:)-points(i,:)) + dis;
end


