function [cost, rute_index,rute] = findPath(map,obstacleVerts,V,E,qstart,qgoal,ifSpecialCase,specialObIdx)
V = [qstart;V];
V = [V;qgoal];
% change E for qstart
E = [E(:,1:2)+1,E(:,3)];
i = 1;

if ~ifSpecialCase
    
    for j = 1:size(V,1)
        if i ~= j
            if isEQfree(V,obstacleVerts,i,j)
                E = [E;[i,j,norm(V(i,:)-V(j,:))]];
                E = [E;[j,i,norm(V(i,:)-V(j,:))]];
            end
        end
    end
    
    i = size(V,1);
    for j = 1:size(V,1)
        if i ~= j
            if isEQfree(V,obstacleVerts,i,j)
                E = [E;[i,j,norm(V(i,:)-V(j,:))]];
                E = [E;[j,i,norm(V(i,:)-V(j,:))]];
            end
        end
    end
else
    for j = 1:size(V,1)
        if i ~= j
            if isEQfree(V,obstacleVerts,i,j,map,specialObIdx)
                E = [E;[i,j,norm(V(i,:)-V(j,:))]];
                E = [E;[j,i,norm(V(i,:)-V(j,:))]];
            end
        end
    end
    
    i = size(V,1);
    for j = 1:size(V,1)
        if i ~= j && j == 1
            if isEQfree(V,obstacleVerts,i,j,map,specialObIdx)
                E = [E;[i,j,norm(V(i,:)-V(j,:))]];
                E = [E;[j,i,norm(V(i,:)-V(j,:))]];
            end
        end
        if i ~= j && j ~=1
            if isEQfree(V,obstacleVerts,i,j)
                E = [E;[i,j,norm(V(i,:)-V(j,:))]];
                E = [E;[j,i,norm(V(i,:)-V(j,:))]];
            end
        end
    end
end

% figure(2)
% obstacleVerts = plotMap('hw8.txt', [0 0 100 100]);
%
%
% for i = 1:size(E,1)
%     plotEdge(V(E(i,1),:),V(E(i,2),:));
% end



G = zeros(size(V,1),size(V,1));
for i = 1:size(V,1)
    for j = 1:size(V,1)
        for k = 1:size(E,1)
            if E(k,1) == i && E(k,2)==j
                G(i,j) = E(k,3);
            end
        end
    end
end
%
% nodes = [(1:size(V,1))',V];
% segments = [(1:size(E,1))',E(:,1:2)];
% [dist, path] = dijkstra2(nodes,segments,1,size(V,1));
[cost, rute_index] = dijkstra(G,1,size(V,1));

rute = V(rute_index,:);
end