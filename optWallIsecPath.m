function index = optWallIsecPath(path,optWalls)
index = [];
for i = 1:size(path,1)-1
    x1 = path(i,1); y1 = path(i,2);  x2 = path(i+1,1); y2 = path(i+1,2);
    for j = 1:size(optWalls,1)
        x3 = optWalls(j,1);  y3 = optWalls(j,2); x4 = optWalls(j,3); y4 = optWalls(j,4);
        [isect,x,y,ua] = intersectPoint(x1,y1,x2,y2,x3,y3,x4,y4);
        if isect
            index = [index; i,i+1,j];
        end
    end
end