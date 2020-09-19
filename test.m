clear 
close all
load('compMap.mat')
All_map = [map;optWalls];
 
points = [waypoints; ECwaypoints;[-2,-1];[1.5,1]];
startpt = [1,-1];   Points_to_go = []; 
 map = [map ; optWalls(1,:) ];
radius = 0.16;
obstacles = wall2polygon(map,radius);
radius = 0.32;
obstacles = wall2polygon(map,radius);
path =  globalpathplanning_RP(map,obstacles,startpt,points,radius);