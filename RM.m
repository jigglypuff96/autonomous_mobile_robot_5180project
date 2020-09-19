clear all
close all
%fileName = 'hw8.txt';
load('compMap.mat')
map = [map;optWalls(1,:)];

qstart = [-1,1];
qgoal = [2,1];
%obstacleVerts = plotMap(fileName, [0 0 100 100]);
obstacleVerts = wall2polygon(map,0.16);
obstacleVerts = wall2polygon(map,0.28);
[V,E] = createRoadmap(obstacleVerts);



for i = 1:size(E,1)
    plotEdge(V(E(i,1),:),V(E(i,2),:));
end
title('map')
xlabel('x')
ylabel('y')

[cost, rute] = findPath(obstacleVerts,V,E,qstart,qgoal);
V = [qstart;V];
V = [V;qgoal];
figure()
%obstacleVerts = plotMap(fileName, [0 0 100 100]);


for i = 1:length(rute)-1
    plotEdge(V(rute(i),:),V(rute(i+1),:));
end

title('shortest path')
xlabel('x')
ylabel('y')
