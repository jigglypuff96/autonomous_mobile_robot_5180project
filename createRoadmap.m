function [V,E] = createRoadmap(map,obstacleVerts,qstart,qgoal,ifSpecialCase,specialObIdx)
if ~exist('qstart','var')
    qstart = [];
end
if ~exist('qgoal','var')
    qgoal = [];
end
%obstacleVerts = plotMap(mapFileName, [0 0 100 100]);
V = [];
for i = 1:length(obstacleVerts)
    V = [V;obstacleVerts{i}(1:end-1,:)];
    
end
V = [qstart;V];
V = [V;qgoal];
x_max = max([map(:,1);map(:,3)]);    x_min = min([map(:,1);map(:,3)]);
y_max = max([map(:,2);map(:,4)]);    y_min = min([map(:,2);map(:,4)]);

V(V(:,1)>=x_max,:) = [];  V(V(:,1)<=x_min,: )= [];
V(V(:,2)>=y_max,:) = [];  V(V(:,2)<=y_min ,:)= [];

E = [];
for i = 1:size(V,1)
    for j = 1:size(V,1)
        if i ~= j
            if (i == 1 || j ==1 ) &&  ifSpecialCase == true            
                if isEQfree(V,obstacleVerts,i,j,map,specialObIdx)
                    E = [E;[i,j,norm(V(i,:)-V(j,:))]];
                end
            else
                if isEQfree(V,obstacleVerts,i,j)
                    E = [E;[i,j,norm(V(i,:)-V(j,:))]];
                end
            end
        end
    end
end
end