clear 
% close all
load('compMap.mat')
load('Matlab.mat')
All_map = [map;optWalls];

% Walls = {};
 points = [waypoints; ECwaypoints];
%  Walls = filterWall(possibleWalls,points);
% for i = 1:size(possibleWalls,2)
%     figure
%     obstacles = wall2polygon(possibleWalls{i},0.16);
%     path{i} =  globalpathplanning_RP(possibleWalls{i},obstacles,points(1,:),points(2:end,:),0.16);
%     if size(path{i},1)>size(points,1)
%         Walls = [Walls,possibleWalls{i}];
%         i
%     end
% end 
startpt = [2.548,0.7051];
 map = [map  ];
% radius = 0.16;
% obstacles = wall2polygon(map,radius);
%radius = 0.16*1.75;
radius = findBestBR(map)
obstacles = wall2polygon(map,radius);
path =  globalpathplanning_RP(map,obstacles,startpt,points,radius);
optWallIsecPath(path,optWalls);