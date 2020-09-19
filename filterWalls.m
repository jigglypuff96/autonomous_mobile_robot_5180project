function Walls = filterWalls(possibleWalls,points_to_go)
startpoint = points_to_go(1,:); 
points_to_go(1,:) = [];   radius = 0.16;
for i = size(possibleWalls,2)
    obstacles = wall2polygon(possibleWalls{i},radius);
path{i} = globalpathplanning_RP(possibleWalls{i},obstacles,startpoint,points_to_go,radius);
end
