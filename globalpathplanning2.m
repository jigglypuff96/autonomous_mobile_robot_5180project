function path = globalpathplanning2(map,startpoint,points_to_go,radius)

V = []; E = []; E_index= [];
all_points = [startpoint;points_to_go];     Value_M = ones(size(all_points,1),size(all_points,1))*1000;       
for i = 1:size(all_points,1)
    for j = 1+i:size(all_points,1)
        [V_i, E_i,E_index_i,ifMerge,obstacles]=buildRRT(map,all_points(i,:),all_points(j,:), radius);
        V_i = [(1:size(V_i,1))',V_i];  E_index_i = [(1:size(E_index_i,1))',E_index_i];
        if ifMerge
            [dist,path] = dijkstra2(V_i,E_index_i,1,size(V_i,1));
            visitpoints{i,j} = V_i(path,2:end);   visitpoints{j,i} = flipud(V_i(path,2:end)); 
            %         plot(visitpoints{i,j}(:,1),visitpoints{i,j}(:,2),'g')
            %         hold on
            Value_M(j,i) = pathvalue(visitpoints{i,j});  Value_M(i,j) = pathvalue(visitpoints{i,j});
            V =  [V;V_i];  E = [E;E_i]; E_index = [E_index;E_index_i];
        end
    end
end

path2 = startpoint;
[p,L2] = tspsearch(Value_M,size(Value_M,1));
index = find(p==1);  p = [p(index:end),p(1:index-1)];
for i = 1:length(p)-1
    path2 = [path2;visitpoints{p(i),p(i+1)}(2:end,:)]; 
%     plot(visitpoints{p(i),p(i+1)}(:,1),visitpoints{p(i),p(i+1)}(:,2),'g');
%      hold on
end

m = 1; path1 = startpoint; i = 1; L1 = 0;
while i < size(all_points,1)     
    [~,next_point_index] = min(Value_M(m,:));   L1 = L1 + Value_M(m,next_point_index) ; Value_M(:,m) = 1000;  
    path1 = [path1; visitpoints{m,next_point_index}(2:end,:)];
%      plot(visitpoints{m,next_point_index}(:,1),visitpoints{m,next_point_index}(:,2),'g');
%      hold on
    m = next_point_index;
    i = i+1;
end


if L1 < L2
    path = path1;
else
    path = path2;
end
path = [path, zeros(size(path,1),1),zeros(size(path,1),1)]; 
for i = 1:size(points_to_go,1)
    path(sum(path(:,1:2) - points_to_go(i,:),2)==0,3) = 1; 
end 


plot(path(:,1),path(:,2),'k')
hold on


function dis = pathvalue(points)
dis = 0;
for i = 1:size(points,1)-1
    dis = norm(points(i+1,:)-points(i,:)) + dis;
end


