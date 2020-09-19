function [isVote, wallToBeVoted] = isVoteOptWalls(robotPose,sensorOrigin,originalMap,optWalls,isFirstLocalization)

if isFirstLocalization == 1
    isVote = 0;
    wallToBeVoted = 0;
    return 
end


maxR = 10;

angle = 0;
endPos = [maxR*cos(angle)+sensorOrigin(1),maxR*sin(angle)+sensorOrigin(2)]; %line 1 end point in robot frame
endPos = robot2global(robotPose,endPos);                                    %line 1 end point in initial frame
startPos = robot2global(robotPose,sensorOrigin);                            %line 1 start point in initial frame
minD = maxR;                       %maximum range
wallID = 1;
map = [originalMap;optWalls];
for j = 1:size(map,1)
    wall = map(j,:);
    x1 = startPos(1);
    y1 = startPos(2);
    x2 = endPos(1);
    y2 = endPos(2);
    x3 = wall(1);
    y3 = wall(2);
    x4 = wall(3);
    y4 = wall(4);
    [ISECT,X,Y,UA] = intersectPoint(x1,y1,x2,y2,x3,y3,x4,y4);
    if ISECT == false
        d = maxR;
    else
        d = UA*maxR;
    end
    
    if d < minD
        minD = d;
        wallID = j;
    end
end
wallID = wallID - size(originalMap,1);
if wallID > 0
    isVote = 1;
    wallToBeVoted = optWalls(wallID,:);

    
else
   isVote = 0;
   wallToBeVoted = 0;
    
end


end


